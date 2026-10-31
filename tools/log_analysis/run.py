"""Download recent WPILOGs on request, then analyze autonomous runs offline."""
from __future__ import annotations
import argparse
import csv
import datetime as dt
import glob
import hashlib
import html
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import time
import numpy as np
import core

ROOT=Path(__file__).resolve().parents[2]

def run_command(args, timeout=30):
    result=subprocess.run(args,capture_output=True,text=True,timeout=timeout)
    if result.returncode: raise RuntimeError(f"{args[0]} failed: {result.stderr.strip() or result.stdout.strip()}")
    return result.stdout

class RobotLogs:
    def __init__(self,host,root):
        if not re.fullmatch(r"[A-Za-z0-9_.-]+",host): raise ValueError("invalid host name")
        self.host=host;self.root=root.resolve();self.root.mkdir(parents=True,exist_ok=True)
        self.options=["-o","BatchMode=yes","-o","ConnectTimeout=5","-o","StrictHostKeyChecking=accept-new","-o",f"UserKnownHostsFile={self.root/'known_hosts'}"]
        for exe in ("ssh","scp"):
            if not shutil.which(exe): raise RuntimeError(f"{exe} is not installed/on PATH")

    def ssh(self,command): return run_command(["ssh",*self.options,f"admin@{self.host}",command])

    def list(self):
        command="find /home/lvuser/logs -type f -name '*.wpilog' -exec stat -c '%Y|%s|%n' {} ';'"
        result=[]
        for line in self.ssh(command).splitlines():
            fields=line.split("|",2)
            if len(fields)!=3 or not fields[0].isdigit() or not fields[1].isdigit(): continue
            stamp,size,path=int(fields[0]),int(fields[1]),fields[2]
            # Remote output is data: only permit ordinary log paths, no shell syntax or traversal.
            if not re.fullmatch(r"/home/lvuser/logs/[A-Za-z0-9_./-]+\.wpilog",path): continue
            if any(part in (".","..") for part in PurePosixPath(path).parts): continue
            result.append(dict(mtime=stamp,size=size,path=path))
        return sorted(result,key=lambda item:(item["mtime"],item["path"]),reverse=True)

    def fetch(self,item):
        key=hashlib.sha256(f"{self.host}|{item['path']}|{item['mtime']}|{item['size']}".encode()).hexdigest()[:16]
        folder=self.root/key;folder.mkdir(exist_ok=True);dest=folder/PurePosixPath(item["path"]).name
        metadata=folder/"download.json"
        if dest.exists() and metadata.exists():
            previous=json.loads(metadata.read_text())
            if previous.get("stable") and dest.stat().st_size==item["size"]:
                return dest,True,previous
        part=folder/(dest.name+".part")
        print(f"Downloading {PurePosixPath(item['path']).name} ({item['size']/1e6:.1f} MB)",flush=True)
        run_command(["scp","-O","-C",*self.options,f"admin@{self.host}:{item['path']}",str(part)],timeout=300)
        after=self.ssh("stat -c '%Y|%s' '"+item["path"]+"'").strip().splitlines()[-1]
        stable=after==f"{item['mtime']}|{item['size']}" and part.stat().st_size==item["size"]
        # A growing file is a point-in-time snapshot, not a stable cached copy. Refresh it next time.
        part.replace(dest)
        data=dict(item,host=self.host,stable=stable,downloaded_utc=dt.datetime.now(dt.timezone.utc).isoformat(),local_bytes=dest.stat().st_size)
        metadata.write_text(json.dumps(data,indent=2))
        return dest,False,data

def optional_chart(path,summary,arrays):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except (ImportError,AttributeError): return []
    output=[]
    if not arrays: return output
    t=arrays["t"];speed=np.linalg.norm(arrays["speeds"][:,:2],axis=1)
    for index,r in enumerate(summary["runs"]):
        start,end=r["start"],r["end"];m=(t>=start)&(t<=end)
        fig,axes=plt.subplots(3,1,figsize=(11,7),sharex=True,layout="constrained")
        axes[0].plot(t[m]-start,speed[m]);axes[0].set_ylabel("Speed (m/s)")
        if r["vision_available"]:
            accepted=arrays["submissions"];accepted=accepted[(accepted>=start)&(accepted<=end)]
            stamps=np.r_[start,accepted];ids=np.searchsorted(stamps,t,side="right")-1
            age=t-stamps[np.maximum(ids,0)];axes[1].plot(t[m]-start,age[m],color="#7c3aed")
        else: axes[1].text(.5,.5,"Vision diagnostics unavailable",transform=axes[1].transAxes,ha="center")
        axes[1].set_ylabel("Fusion gap (s)")
        sm=m[1:]&arrays["valid"];axes[2].plot(t[1:][sm]-start,arrays["residual"][sm],color="#ea580c")
        axes[2].set_ylabel("Correction proxy (m)");axes[2].set_xlabel("Seconds after autonomous enable")
        for ax in axes:
            ax.grid(alpha=.2);ax.set_xlim(0,max(.1,end-start))
            for gap in r["fusion_gaps"] or []:
                if gap["duration_s"]>=1: ax.axvspan(gap["start_s"],gap["end_s"],color="#f59e0b",alpha=.1)
        fig.suptitle(f"{r['auto']} | {Path(summary['file']).name} | run {index+1}")
        chart=path/f"run-{index+1}.png";fig.savefig(chart,dpi=140);plt.close(fig);output.append(chart.name)
    return output

def cached_analyze(path,out,refresh=False):
    identity=f"{core.VERSION}|{path.resolve()}|{path.stat().st_size}|{path.stat().st_mtime_ns}"
    key=hashlib.sha256(identity.encode()).hexdigest()[:16]
    folder=out/"files"/key;folder.mkdir(parents=True,exist_ok=True);json_path=folder/"summary.json"
    if json_path.exists() and not refresh:
        summary=json.loads(json_path.read_text());print(f"Cached {path.name}: {len(summary['runs'])} autonomous run(s)",flush=True);return summary,folder
    started=time.perf_counter();summary,arrays=core.analyze(path)
    summary["analysis_seconds"]=time.perf_counter()-started
    summary["charts"]=optional_chart(folder,summary,arrays)
    json_path.write_text(json.dumps(summary,indent=2,allow_nan=False))
    print(f"Analyzed {path.name}: {len(summary['runs'])} autonomous run(s), {summary['analysis_seconds']:.1f} s",flush=True)
    return summary,folder

def report(out,results,downloads):
    rows=[];allruns=[];notes=[];charts=[]
    for summary,folder in results:
        filename=Path(summary["file"]).name
        for warning in summary["warnings"]: notes.append(f"{filename}: {warning}")
        if summary["integrity"]["truncated_tail"]: notes.append(f"{filename}: incomplete trailing record; complete prefix analyzed.")
        for r in summary["runs"]:
            longest=max((g["duration_s"] for g in r["fusion_gaps"] or []),default=0) if r["vision_available"] else None
            item=dict(file=filename,auto=r["auto"],start_fpga_s=r["start"],duration_s=r["duration_s"],peak_speed_mps=(r["speed_mps"] or {}).get("max"),vision_submissions=r["vision_submissions"],longest_gap_s=longest,tracking_p95_m=(r["path_tracking_error_m"] or {}).get("p95"),correction_step_max_m=(r["pose_step_residual_m"] or {}).get("max"),alerts="; ".join(r["alerts"]))
            allruns.append(item)
            def value(v): return "unavailable" if v is None else f"{v:.3f}" if isinstance(v,float) else str(v)
            rows.append("<tr>"+"".join("<td>"+html.escape(value(v))+"</td>" for v in item.values())+"</tr>")
        charts.extend(f"<img alt='Autonomous diagnostic chart' src='{html.escape((folder/chart).relative_to(out).as_posix())}'>" for chart in summary.get("charts",[]))
    (out/"runs.json").write_text(json.dumps(dict(runs=allruns,downloads=downloads),indent=2))
    with (out/"runs.csv").open("w",newline="") as handle:
        fields=list(allruns[0]) if allruns else ["file","auto","start_fpga_s","duration_s","peak_speed_mps","vision_submissions","longest_gap_s","tracking_p95_m","correction_step_max_m","alerts"]
        writer=csv.DictWriter(handle,fieldnames=fields);writer.writeheader();writer.writerows(allruns)
    headers=("Log","Auto","FPGA start (s)","Duration (s)","Peak speed (m/s)","Vision submissions","Longest gap (s)","Tracking p95 (m)","Largest correction step (m)","Flags")
    content="""<!doctype html><html lang=en><meta charset=utf-8><title>Autonomous log analysis</title><style>body{font:15px/1.6 system-ui;margin:32px;color:#172238}table{border-collapse:collapse;font-size:12px}td,th{padding:8px;border:1px solid #ddd;text-align:left}img{width:100%;max-width:1100px}.scroll{overflow:auto}</style><h1>Autonomous log analysis</h1>"""
    content+=f"<p>{len(results)} log(s); {len(allruns)} autonomous-enable interval(s). Generated {dt.datetime.now(dt.timezone.utc).isoformat()}.</p>"
    content+="<div class=scroll><table><tr>"+"".join(f"<th>{h}</th>" for h in headers)+"</tr>"+"".join(rows)+"</table></div>"
    content+="<h2>Coverage and limitations</h2><ul>"+"".join("<li>"+html.escape(n)+"</li>" for n in notes)+"</ul>"
    content+="<p>Autonomous intervals use enabled AND autonomous flags. FUSION_SUBMITTED means submitted to CTRE, not confirmed native-estimator acceptance. Vision gaps include run-clipped initial/final intervals. A missing diagnostics channel is reported as unavailable, never zero accepted vision. Corrections subtract integrated reported wheel motion from estimated pose changes; they are not independent physical ground truth. Nonpositive/≥100 ms intervals, samples before speed telemetry exists and ±300 ms around explicit pose resets are excluded. NT records are change-only, so record age alone does not prove that an unchanged speed or voltage is stale. Target error uses activePath and targets fresher than 100 ms, excluding the first 150 ms. Module errors compare signed velocity vectors, respecting equivalent optimized states. Low-speed steering samples are excluded. Logs from different clocks are not automatically merged. Alert thresholds are screening heuristics, not pass/fail calibration criteria.</p>"
    content+="<p>No robot configuration, code, motor command, or remote file is changed by this workflow. Downloads and reports are local. Per-file JSON includes all metrics, rack channel inventory, faults, power and rejection counts.</p>"+"".join(charts)+"</html>"
    (out/"report.html").write_text(content,encoding="utf-8")
    print(f"Report: {out/'report.html'}\nAutonomous runs: {len(allruns)}",flush=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files",nargs="*",help="Offline .wpilog paths, directories, or globs")
    parser.add_argument("--fetch",action="store_true",help="Download recent logs from robot, read-only")
    parser.add_argument("--host",default="10.6.96.2")
    parser.add_argument("--latest",type=int,default=3,help="Initial newest log count (default 3)")
    parser.add_argument("--lookback",type=int,default=12,help="Look further back if initial batch has no autonomous runs")
    parser.add_argument("--download-dir",type=Path,default=ROOT/"logs/downloads/latest")
    parser.add_argument("--out",type=Path,default=ROOT/"artifacts/log_analysis/latest")
    parser.add_argument("--refresh",action="store_true",help="Reanalyze cached results")
    args=parser.parse_args()
    if not args.fetch and not args.files: parser.error("provide --fetch or offline files")
    if args.latest<1 or args.lookback<args.latest: parser.error("require 1 <= --latest <= --lookback")
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=True);results=[];downloads=[];failures=[]
    paths=[]
    for value in args.files:
        p=Path(value)
        paths.extend(sorted(p.glob("*.wpilog")) if p.is_dir() else map(Path,glob.glob(value)))
    for path in dict.fromkeys(paths):
        try: results.append(cached_analyze(path,out,args.refresh))
        except (ValueError,OSError,TypeError) as error: failures.append(f"{path}: {error}")
    if args.fetch:
        robot=RobotLogs(args.host,args.download_dir);listing=robot.list()
        for i,item in enumerate(listing[:args.lookback]):
            if i>=args.latest and any(summary["runs"] for summary,_ in results): break
            try:
                path,cached,metadata=robot.fetch(item);downloads.append(metadata)
                if not metadata["stable"]: print(f"Warning: {path.name} changed while copying; analyzing a snapshot, will refresh next time.",flush=True)
                results.append(cached_analyze(path,out,args.refresh))
            except (ValueError,OSError,RuntimeError,subprocess.TimeoutExpired) as error: failures.append(f"{item['path']}: {error}")
    report(out,results,downloads)
    (out/"errors.txt").write_text("\n".join(failures))
    if failures: print("\n".join(failures),file=sys.stderr)
    if not results: return 1
    return 0

if __name__=="__main__":
    try: raise SystemExit(main())
    except (RuntimeError,subprocess.TimeoutExpired) as error: print(error,file=sys.stderr);raise SystemExit(1)
