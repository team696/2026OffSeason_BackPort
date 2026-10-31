Team 696 autonomous log analysis
================================

From the repository root, when asked to pull and inspect the latest logs:

  .\tools\log_analysis\Analyze-Logs.ps1 -Fetch

Offline, using already downloaded files:

  .\tools\log_analysis\Analyze-Logs.ps1 -LogPath 'logs/downloads/2026-10-30-audit/*.wpilog'

Options: -RobotHost 10.6.96.2 -Latest 3 -Lookback 12 -OutputDirectory <folder>
         -PythonExecutable <python.exe> -Refresh

Equivalent portable Python CLI:

  python tools/log_analysis/run.py --fetch --latest 3 --lookback 12
  python tools/log_analysis/run.py <file.wpilog> <another.wpilog> --out artifacts/log_analysis/latest

Python 3.10+ and numpy are required; OpenSSH ssh/scp must be on PATH for downloads.
The PowerShell wrapper tries this workstation's bundled Python, then normal PATH.
Install dependencies only if missing: python -m pip install -r tools/log_analysis/requirements.txt
PNG charts are optional: install matplotlib into that interpreter. Existing reports
remain useful without it. Run -Refresh if charts become available after analysis.

Behavior
--------
* Download is explicit (--fetch); offline commands never contact the robot.
* Lists files under /home/lvuser/logs using SSH as admin with noninteractive auth.
* Starts with three newest WPILOGs. If none contains autonomous, walks further
  back up to twelve files by default. Change -Latest/-Lookback as needed.
* Caches stable downloads by remote path, modification time and byte length.
  Changed/growing files become new point-in-time snapshots and refresh next time.
* Checks remote metadata after copying. An actively changing log may have a
  partial tail; only complete records are analyzed and this limitation is shown.
* Trusts the initial SSH host key once, then uses logs/downloads/latest/known_hosts.
  A changed key fails rather than silently accepting a replacement.
* Does not delete remote files, deploy code, change configs or command motors.
* Never automatically merges CTRE and WPILib clocks or selects a robot mode from
  a filename. Converted HOOT logs are analyzed in their own clock using mode flags.

Output
------
artifacts/log_analysis/latest/report.html  human-readable summary, optional plots
artifacts/log_analysis/latest/runs.csv     compact autonomous run comparison
artifacts/log_analysis/latest/runs.json    current run list and download provenance
artifacts/log_analysis/latest/files/*/summary.json  per-file metrics and entry inventory
artifacts/log_analysis/latest/errors.txt   errors for the current invocation
logs/downloads/latest/*                   cached raw logs and metadata

The default logs/ and artifacts/ destinations are ignored by Git. Code here is not.
Per-file analysis caches use parser version, absolute path, size and local mtime.
An unchanged cached run can be summarized without parsing its log again.

What is checked
---------------
Enabled AND autonomous windows; selected routine; wheel-derived chassis speed;
pose-step residual after subtracting wheel-derived travel; explicit pose resets;
camera submission gaps and rejection reasons; submitted camera disagreement and
frame age; active-path target error; swerve module velocity-vector and steering
error; PDH voltage and supported reset/brownout fault channels if present.
All rack/pinion channel names are inventoried for mechanism investigation.

Interpretation safeguards
------------------------
FUSION_SUBMITTED is not proof the native estimator accepted that measurement.
Missing camera telemetry is marked unavailable, never called zero vision updates.
Correction proxies exclude nonpositive/100+ ms steps, samples before speed
telemetry exists, and 300 ms around explicit pose resets. Larger gaps remain
explicit coverage limits. NT records are change-only, so an old unchanged speed
or voltage record is not automatically considered stale device data.
Path error requires activePath and a target fresher than 100 ms; startup's first
150 ms is excluded. This avoids counting stale shooting targets as path error.
Module velocity vectors respect equivalent optimized speed/angle combinations;
steering samples with both measured and demanded speed below 0.25 m/s are omitted.
Wheel and pose estimates are not independent physical ground truth. A correction
does not establish which estimate is physically correct. Duplicate camera polls
are not by themselves evidence of a disconnected camera. Threshold alerts are
screening heuristics (1 s submission gap, 10 cm pose-step residual, 30 cm tracking
p95), not acceptance criteria for a competition robot.

Additional analysis methods reviewed
-----------------------------------
1. AdvantageScope synchronized line graphs + 2D field: overlay estimated pose,
   PathPlanner target and raw camera pose; use the same selected timestamp in all
   tabs to inspect what precedes a correction. Graph camera decisions and frame
   age on the same timeline as speed, yaw/pitch and commanded/measured modules.
   https://docs.advantagescope.org/tab-reference/line-graph/
   https://docs.advantagescope.org/tab-reference/2d-field/

2. Swerve tab: commanded versus measured module states, especially at path joins.
   Velocity-vector comparisons avoid falsely flagging optimized 180-degree flips.
   https://docs.advantagescope.org/tab-reference/swerve/

3. Synchronized video: distinguish an estimator correction from actual wheel
   slip, contact, occlusion or a large physical steering response. Do not infer
   physical truth from the estimator trace alone.
   https://docs.advantagescope.org/tab-reference/video/

4. CTRE Owlet/Tuner: download both rio and CANivore HOOT files from the same
   session when power, resets, voltage/current limits, hardware faults or Pigeon
   behavior need investigation. Scan and selectively export only required signals;
   trim to enabled periods with an appropriate buffer. Use the compatible Owlet
   version and preserve raw HOOT files. Do not align only by directory dates.
   https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tools/log-extractor.html
   https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tools/log-analysis.html

5. Hoot Replay: useful for testing estimator/filter changes against recorded
   input rather than repeated robot tests. It requires a Pro-licensed device,
   a replay vendordep and explicit playback of custom inputs. It is not currently
   installed in the robot project; do not replace the production vendordep as
   part of a routine log check.
   https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/hoot-replay.html

6. Controlled comparisons: repeat the same route with fixed gains at lower speed
   and acceleration, then change one variable at a time. Compare error percentiles,
   longest loss of submitted vision, recovery correction size, per-module lag and
   battery voltage. Separate camera visibility from quality-filter rejections.

7. Rack diagnostics: position/target/mode already exist, but reliable diagnosis
   needs timestamped zero requests/results, hasZeroed, hard-stop evidence, motor
   reset/brownout flags, supply/motor voltage and current. External position or
   video is needed to distinguish encoder-reference loss from mechanical slip.

Local regression tests, when retained on this workstation, live under the ignored
artifacts/local_tests/ directory and are not part of the repository.

Verified archive and robot disk cleanup (separate explicit operation)
------------------------------------------------------------------
  .\tools\log_analysis\Archive-RobotLogs.ps1 -DryRun
  .\tools\log_analysis\Archive-RobotLogs.ps1

The second command downloads all eligible closed WPILOG and HOOT files, checks
their local and remote SHA256, and immediately deletes each verified unchanged
remote source. It retains files listed as open by lsof and files modified within
the last 60 seconds. It checks open handles, metadata and hash again immediately
before deletion. Failure to inspect/verify aborts or retains the file. No remote
temporary file is created and no recursive/wildcard deletion is performed.

Archive options: -RobotHost 10.6.96.2 -ArchiveDirectory <folder>
                 -MinimumAgeSeconds 60 -DownloadOnly -DryRun
Archives default to logs/archive/<host>/<UTC-session> with manifest.json.
Archives preserve original subdirectories and a SHA256/deletion-status manifest.
An interrupted copy is left as .partial and its remote source is retained.
This cleanup is deliberately not part of the normal Analyze-Logs.ps1 -Fetch path.

Local archive guard checks are retained under artifacts/local_tests/log_analysis/.
