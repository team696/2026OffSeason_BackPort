[CmdletBinding()]
param(
    [switch]$Fetch,
    [string[]]$LogPath,
    [string]$RobotHost = '10.6.96.2',
    [int]$Latest = 3,
    [int]$Lookback = 12,
    [string]$OutputDirectory,
    [string]$PythonExecutable,
    [switch]$Refresh
)
$ErrorActionPreference = 'Stop'
if (-not $PythonExecutable) {
    $candidates = @(
        (Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'),
        (Join-Path $env:USERPROFILE 'AppData\Local\Programs\Python\Python312\python.exe')
    )
    $PythonExecutable = $candidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    if (-not $PythonExecutable) {
        $command = Get-Command python -ErrorAction SilentlyContinue
        if ($command) { $PythonExecutable = $command.Source }
    }
}
if (-not $PythonExecutable) { throw 'Python was not found. Pass -PythonExecutable with a Python 3.10+ interpreter containing numpy.' }
$scriptPath = Join-Path $PSScriptRoot 'run.py'
$arguments = @($scriptPath, '--host', $RobotHost, '--latest', "$Latest", '--lookback', "$Lookback")
if ($Fetch) { $arguments += '--fetch' }
if ($Refresh) { $arguments += '--refresh' }
if ($OutputDirectory) { $arguments += @('--out', $OutputDirectory) }
if ($LogPath) { $arguments += $LogPath }
& $PythonExecutable @arguments
exit $LASTEXITCODE
