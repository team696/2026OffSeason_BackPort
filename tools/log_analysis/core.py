"""Read-only WPILOG diagnostics. Estimates are not independent physical ground truth."""
from collections import Counter, defaultdict
import mmap
from pathlib import Path
import struct
import numpy as np

VERSION = "2.0"
KNOWN_STRUCTS = {"struct:Pose2d", "struct:Pose2d[]", "struct:ChassisSpeeds", "struct:SwerveModuleState[]", "struct:SwerveModulePosition[]"}

def decode(kind, data):
    if kind in ("string", "json"): return data.decode("utf-8", errors="replace")
    if kind == "boolean" and len(data) == 1: return bool(data[0])
    if kind == "int64" and len(data) == 8: return int.from_bytes(data, "little", signed=True)
    if kind == "double" and len(data) == 8: return struct.unpack("<d", data)[0]
    if kind == "float" and len(data) == 4: return struct.unpack("<f", data)[0]
    if kind == "string[]":
        if len(data) < 4: raise ValueError("short string-array header")
        count = int.from_bytes(data[:4], "little"); p = 4; values = []
        if count > len(data)//4: raise ValueError("invalid string-array count")
        for _ in range(count):
            if p+4 > len(data): raise ValueError("short string length")
            n = int.from_bytes(data[p:p+4], "little"); p += 4
            if p+n > len(data): raise ValueError("short string payload")
            values.append(data[p:p+n].decode("utf-8", errors="replace")); p += n
        return values
    if kind == "double[]" or kind in KNOWN_STRUCTS:
        if len(data)%8: raise ValueError("invalid double/known-struct payload size")
        return list(struct.unpack("<"+"d"*(len(data)//8), data))
    if kind == "int64[]":
        if len(data)%8: raise ValueError("invalid integer-array size")
        return list(struct.unpack("<"+"q"*(len(data)//8), data))
    return None

def selected(name):
    if name.startswith("DS:"): return name in {"DS:enabled", "DS:autonomous", "DS:test"}
    if any(s in name for s in ("Rack", "Pinion", "Pinon")): return True
    return any(s in name for s in ("DriveState/", "/PathPlanner/", "Auto Chooser/", "/Localization/Reset/", "/DecisionEvent", "/DecisionCodes", "/DecisionEventColumns", "/Voltage", "/TotalCurrent", "/SupplyVoltage", "/Fault_BridgeBrownout", "/Fault_BootDuringEnable", "RobotEnable", "RobotMode"))

def read(path):
    """Skip unused payloads; tolerate a partial final record, reject invalid headers."""
    path=Path(path); series=defaultdict(list); inventory={}; entries={}; malformed=0; truncated=False
    with path.open("rb") as handle:
        if path.stat().st_size<12: raise ValueError("WPILOG header missing/incomplete")
        with mmap.mmap(handle.fileno(),0,access=mmap.ACCESS_READ) as raw:
            if raw[:6]!=b"WPILOG": raise ValueError("not a WPILOG file")
            if int.from_bytes(raw[6:8],"little")!=0x100: raise ValueError("unsupported WPILOG version")
            p=12+int.from_bytes(raw[8:12],"little")
            if p>len(raw): raise ValueError("incomplete extra header")
            while p<len(raw):
                flag=raw[p];p+=1;ne,ns,nt=(flag&3)+1,((flag>>2)&3)+1,((flag>>4)&7)+1
                if p+ne+ns+nt>len(raw): truncated=True;break
                eid=int.from_bytes(raw[p:p+ne],"little");p+=ne
                size=int.from_bytes(raw[p:p+ns],"little");p+=ns
                stamp=int.from_bytes(raw[p:p+nt],"little");p+=nt
                stamp=(stamp if stamp<2**63 else stamp-2**64)/1e6;end=p+size
                if end>len(raw): truncated=True;break
                if eid==0:
                    data=raw[p:end]
                    if data and data[0]==0:
                        try:
                            if len(data)<5: raise ValueError("short declaration")
                            ident=int.from_bytes(data[1:5],"little");q=5;fields=[]
                            for _ in range(3):
                                if q+4>len(data): raise ValueError("short declaration field")
                                n=int.from_bytes(data[q:q+4],"little");q+=4
                                if q+n>len(data): raise ValueError("short declaration string")
                                fields.append(data[q:q+n].decode("utf-8",errors="replace"));q+=n
                            entries[ident]=fields
                        except ValueError: malformed+=1
                elif eid in entries:
                    name,kind,_=entries[eid]
                    entry=inventory.setdefault(name,dict(type=kind,count=0,first=stamp,last=stamp));entry["count"]+=1;entry["last"]=stamp
                    if selected(name):
                        try: value=decode(kind,raw[p:end])
                        except (ValueError,struct.error): malformed+=1;value=None
                        if value is not None: series[name].append((stamp,value))
                p=end
    for rows in series.values(): rows.sort(key=lambda r:r[0])
    return dict(series),inventory,dict(truncated_tail=truncated,malformed_records=malformed)

def at(rows,times,default=0):
    if not rows: return np.full(len(times),default)
    stamps=np.array([r[0] for r in rows]);values=np.asarray([r[1] for r in rows]);ids=np.searchsorted(stamps,times,side="right")-1
    result=values[np.maximum(ids,0)].copy();result[ids<0]=default;return result

def stats(values):
    values=np.asarray(values);values=values[np.isfinite(values)]
    return dict(zip(("min","p50","p95","p99","max"),map(float,np.quantile(values,[0,.5,.95,.99,1])))) if len(values) else None

def wrap(value): return np.arctan2(np.sin(value),np.cos(value))

def intervals(t,mask):
    starts=np.flatnonzero(mask&~np.r_[False,mask[:-1]]);ends=np.flatnonzero(mask&~np.r_[mask[1:],False])
    return [(float(t[a]),float(t[b])) for a,b in zip(starts,ends)]

def code_name(codes,value):
    index=int(value);return codes[index] if 0<=index<len(codes) else f"UNKNOWN_{index}"

def analyze(path):
    series,inventory,integrity=read(path)
    result=dict(version=VERSION,file=str(Path(path).resolve()),bytes=Path(path).stat().st_size,integrity=integrity,warnings=[],runs=[],entries=inventory,rack_channels=[k for k in inventory if any(s in k for s in ("Rack","Pinion","Pinon"))])
    pose_key=next((k for k in ("NT:/DriveState/Pose","DriveState/Pose") if k in series),None)
    if pose_key is None:
        result["warnings"].append("No drivetrain pose telemetry; autonomous analysis unavailable.");return result,{}
    rows=[r for r in series[pose_key] if len(r[1])==3 and np.all(np.isfinite(r[1]))]
    if len(rows)<2:
        result["warnings"].append("Fewer than two finite drivetrain poses.");return result,{}
    t=np.array([r[0] for r in rows]);pose=np.asarray([r[1] for r in rows]);prefix="NT:/" if pose_key.startswith("NT:") else ""
    speed_rows=[r for r in series.get(prefix+"DriveState/Speeds",[]) if len(r[1])==3]
    speeds=at(speed_rows,t) if speed_rows else np.full_like(pose,np.nan)
    if not speed_rows: result["warnings"].append("No chassis speeds; correction proxy unavailable.")
    if "DS:enabled" in series and "DS:autonomous" in series:
        enabled=at(series["DS:enabled"],t).astype(bool);auto=enabled&at(series["DS:autonomous"],t).astype(bool);mode_source="Driver Station flags"
    elif "RobotEnable" in series and "RobotMode" in series:
        enabled=at(series["RobotEnable"],t).astype(bool);auto=enabled&(at(series["RobotMode"],t,default="Disabled")=="Autonomous");mode_source="CTRE mode flags (this log's clock only)"
    else:
        enabled=np.zeros(len(t),bool);auto=enabled.copy();mode_source="unavailable";result["warnings"].append("No enabled/autonomous flags; runs cannot be identified.")
    result.update(mode_source=mode_source,enabled_intervals=intervals(t,enabled),time_range=[float(t[0]),float(t[-1])])
    delta=np.diff(t);theta=pose[:-1,2]+wrap(np.diff(pose[:,2]))/2;vx=(speeds[:-1,0]+speeds[1:,0])/2;vy=(speeds[:-1,1]+speeds[1:,1])/2
    predicted=np.column_stack(((vx*np.cos(theta)-vy*np.sin(theta))*delta,(vx*np.sin(theta)+vy*np.cos(theta))*delta))
    residual_vectors=np.diff(pose[:,:2],axis=0)-predicted;residual=np.linalg.norm(residual_vectors,axis=1);speed=np.linalg.norm(speeds[:,:2],axis=1)
    # NT log records are change-only: an old speed record can mean a steady value,
    # not stale device data. Do not infer transport freshness from its record age.
    have_speed=(t>=speed_rows[0][0]) if speed_rows else np.zeros(len(t),bool)
    valid=(delta>0)&(delta<.1)&have_speed[:-1]&have_speed[1:]&np.all(np.isfinite(predicted),axis=1)
    reset_times=np.array([float(v) for _,v in series.get("NT:/Localization/Reset/FpgaSeconds",[]) if np.isfinite(v)]);near_reset=np.zeros(len(t),bool)
    for stamp in reset_times: near_reset|=abs(t-stamp)<.3
    valid&=~near_reset[:-1]&~near_reset[1:]
    target_rows=[r for r in series.get("NT:/PathPlanner/targetPose",[]) if len(r[1])==3];target=at(target_rows,t) if target_rows else None
    active_rows=series.get("NT:/PathPlanner/activePath",[]);active=at([(stamp,bool(v)) for stamp,v in active_rows],t).astype(bool)
    target_age=t-at([(stamp,stamp) for stamp,_ in target_rows],t,default=-1e9)
    if target_rows and not active_rows: result["warnings"].append("Missing activePath; tracking errors omitted to avoid stale targets.")
    cameras=[]
    for key,events in series.items():
        if not key.endswith("/DecisionEvent"): continue
        base=key[:-len("DecisionEvent")];code_rows=series.get(base+"DecisionCodes",[]);column_rows=series.get(base+"DecisionEventColumns",[])
        codes=code_rows[-1][1] if code_rows else [];columns=column_rows[-1][1] if column_rows else []
        expected={2:"decisionFpgaSeconds",3:"captureFpgaSeconds",11:"tagCount",12:"averageTagDistance_m",14:"residualToCurrentPose_m"}
        if any(i>=len(columns) or columns[i]!=name for i,name in expected.items()): result["warnings"].append(f"Unsupported/missing vision schema: {base}");continue
        values=np.asarray([v for _,v in events if len(v)>=19 and np.isfinite(v[2])])
        if values.ndim!=2 or "FUSION_SUBMITTED" not in codes: continue
        values=values[np.argsort(values[:,2])];cameras.append(dict(name=base,codes=codes,times=values[:,2],values=values,submitted=values[:,1]==codes.index("FUSION_SUBMITTED")))
    if not cameras: result["warnings"].append("Vision diagnostics unavailable; missing telemetry is not a measured vision dropout.")
    submission_times=np.unique(np.concatenate([c["times"][c["submitted"]] for c in cameras])) if cameras else np.array([])
    selection=series.get("NT:/SmartDashboard/Auto Chooser/selected",[])
    state_rows=[r for r in series.get(prefix+"DriveState/ModuleStates",[]) if len(r[1])==8];demand_rows=[r for r in series.get(prefix+"DriveState/ModuleTargets",[]) if len(r[1])==8]
    actual=at(state_rows,t) if state_rows else None;demanded=at(demand_rows,t) if demand_rows else None
    power_keys=[k for k in series if "PowerDistribution" in k and k.endswith("/Voltage")]
    arrays=dict(t=t,pose=pose,speeds=speeds,auto=auto,residual=residual,valid=valid,submissions=submission_times)
    if target is not None: arrays["target"]=target
    for start,end in intervals(t,auto):
        mask=(t>=start)&(t<=end);smask=mask[1:]&valid;name=next((v for stamp,v in reversed(selection) if stamp<=start),"unknown")
        run=dict(start=start,end=end,duration_s=end-start,auto=name,speed_mps=stats(speed[mask]),pose_step_residual_m=stats(residual[smask]),vision_available=bool(cameras),vision_submissions=None,fusion_gaps=None,cameras=[],reset_times=[float(x) for x in reset_times if start-.3<=x<=end+.3],path_tracking_error_m=None,modules=[],power={},alerts=[],correction_bursts=[])
        following=mask&active&(target_age<.1)&~near_reset&(t>start+.15)
        if target is not None and np.any(following): run["path_tracking_error_m"]=stats(np.linalg.norm(pose[:,:2]-target[:,:2],axis=1)[following])
        if cameras:
            atimes=submission_times[(submission_times>=start)&(submission_times<=end)];run["vision_submissions"]=len(atimes);bounds=np.r_[start,atimes,end];run["fusion_gaps"]=[]
            for i,(lo,hi) in enumerate(zip(bounds,bounds[1:])):
                if hi-lo<.5: continue
                gm=(t>=lo)&(t<=hi);run["fusion_gaps"].append(dict(start_s=float(lo-start),end_s=float(hi-start),duration_s=float(hi-lo),reacquired=i<len(atimes),max_speed_mps=stats(speed[gm])))
            for camera in cameras:
                et,ev,submitted=camera["times"],camera["values"],camera["submitted"];emask=(et>=start)&(et<=end)
                run["cameras"].append(dict(name=camera["name"],decisions=dict(Counter(code_name(camera["codes"],v) for v in ev[emask,1])),submitted_disagreement_m=stats(ev[emask&submitted,14]),submitted_frame_age_s=stats(ev[emask&submitted,4])))
                for row in ev[emask&submitted&(ev[:,14]>.3)]:
                    stamp=float(row[2])
                    if any(abs(stamp-b["fpga_s"])<.3 for b in run["correction_bursts"]): continue
                    bm=(t[1:]>=stamp-.025)&(t[1:]<=stamp+.25)&valid
                    if np.any(bm): run["correction_bursts"].append(dict(fpga_s=stamp,relative_s=stamp-start,camera_disagreement_m=float(row[14]),net_correction_proxy_m=float(np.linalg.norm(np.sum(residual_vectors[bm],axis=0))),largest_step_m=float(np.max(residual[bm]))))
            longest=max((g["duration_s"] for g in run["fusion_gaps"]),default=0)
            if longest>=1: run["alerts"].append(f"No submitted vision for up to {longest:.2f} s (includes clipped initial/final gaps).")
        if run["pose_step_residual_m"] and run["pose_step_residual_m"]["max"]>.1: run["alerts"].append("Pose step exceeds wheel-derived travel by >10 cm outside reset windows.")
        if run["path_tracking_error_m"] and run["path_tracking_error_m"]["p95"]>.3: run["alerts"].append("Path-active estimated-position error p95 exceeds 30 cm.")
        if actual is not None and demanded is not None:
            for m in range(4):
                av=actual[:,2*m];aa=actual[:,2*m+1];dv=demanded[:,2*m];da=demanded[:,2*m+1]
                error=np.hypot(av*np.cos(aa)-dv*np.cos(da),av*np.sin(aa)-dv*np.sin(da));moving=mask&((abs(av)>.25)|(abs(dv)>.25));angle=abs(wrap(aa-da));angle=np.minimum(angle,np.pi-angle)
                run["modules"].append(dict(module=m,vector_error_mps=stats(error[moving]),angle_error_deg=stats(np.degrees(angle[moving]))))
        for key in power_keys: run["power"][key]=stats(at(series[key],t)[mask])
        for key in series:
            if ("Fault_BridgeBrownout" in key or "Fault_BootDuringEnable" in key) and np.any(at(series[key],t)[mask]): run["alerts"].append(f"Asserted hardware fault: {key}")
        result["runs"].append(run)
    return result,arrays
