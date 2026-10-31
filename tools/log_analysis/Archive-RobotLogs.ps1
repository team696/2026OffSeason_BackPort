# Archives closed robot logs, verifies SHA256, then removes each verified source.
# Does not deploy code, stop the robot program, or recursively delete directories.
[CmdletBinding()]
param(
    [ValidatePattern('^[A-Za-z0-9_.-]+$')][string]$RobotHost = '10.6.96.2',
    [string]$ArchiveDirectory,
    [ValidateRange(0,86400)][int]$MinimumAgeSeconds = 60,
    [switch]$DownloadOnly,
    [switch]$DryRun
)
$ErrorActionPreference = 'Stop'
foreach ($exe in @('ssh','scp')) {
    if (-not (Get-Command $exe -ErrorAction SilentlyContinue)) { throw "$exe is not installed or not on PATH." }
}
$repositoryRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
if (-not $ArchiveDirectory) { $ArchiveDirectory = Join-Path $repositoryRoot 'logs\archive' }
$archiveRoot = [IO.Path]::GetFullPath($ArchiveDirectory)
$sessionName = (Get-Date).ToUniversalTime().ToString('yyyyMMdd-HHmmss') + '-' + [Guid]::NewGuid().ToString('N').Substring(0,8)
$sessionRoot = Join-Path (Join-Path $archiveRoot $RobotHost) $sessionName
New-Item -ItemType Directory -Path $sessionRoot -Force | Out-Null
$sessionRoot = [IO.Path]::GetFullPath($sessionRoot)
$knownHosts = Join-Path $archiveRoot 'known_hosts'
$sshOptions = @('-o','BatchMode=yes','-o','ConnectionAttempts=1','-o','ConnectTimeout=5',
    '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=3','-o','StrictHostKeyChecking=accept-new',
    '-o', ('UserKnownHostsFile="' + $knownHosts.Replace('\','/') + '"'))
$destination = "admin@$RobotHost"
$manifest = [Collections.Generic.List[object]]::new()
$manifestPath = Join-Path $sessionRoot 'manifest.json'
[long]$freedBytes = 0
[long]$copiedBytes = 0

function Invoke-RobotSsh([string]$Command) {
    $Command = $Command.Replace("`r`n","`n").Replace("`r","`n")
    $previousPreference = $ErrorActionPreference
    try {
        # Windows PowerShell 5.1 can turn native stderr banners into errors even
        # when SSH succeeds. Decide success from the native exit code instead.
        $ErrorActionPreference = 'Continue'
        $output = @(& ssh @sshOptions $destination $Command 2>&1)
        $code = $LASTEXITCODE
    } finally { $ErrorActionPreference = $previousPreference }
    if ($code -ne 0) { throw "SSH failed (exit $code): $($output -join [Environment]::NewLine)" }
    return @($output | ForEach-Object { $_.ToString() })
}
function Save-Manifest {
    ConvertTo-Json -InputObject @($manifest.ToArray()) -Depth 6 | Set-Content -LiteralPath $manifestPath -Encoding UTF8
}
function Test-LogPath([string]$Path) {
    # A deliberately narrow filename alphabet also makes literal shell quoting safe.
    return $Path -cmatch '^/home/lvuser/logs/(?:[A-Za-z0-9_-]+/)*[A-Za-z0-9_.-]+\.(?:wpilog|hoot)$'
}

Write-Host "Archive destination: $sessionRoot"
Write-Host 'Inspecting logs and open file handles...'
$inventoryCommand = @'
command -v lsof >/dev/null 2>&1 || exit 71
command -v sha256sum >/dev/null 2>&1 || exit 72
command -v awk >/dev/null 2>&1 || exit 81
printf '__NOW|%s\n' "$(date +%s)"
find /home/lvuser/logs -type f \( -name '*.wpilog' -o -name '*.hoot' \) -exec stat -c '__FILE|%Y|%s|%n' {} ';'
printf '__OPEN_BEGIN\n'
lsof 2>/dev/null || exit 73
printf '__OPEN_END\n'
df -k /home/lvuser
'@
$inventory = @(Invoke-RobotSsh $inventoryCommand)
$nowLine = @($inventory | Where-Object { $_ -match '^__NOW\|[0-9]+$' })
if ($nowLine.Count -ne 1) { throw 'Could not determine robot time; nothing deleted.' }
[long]$robotNow = $nowLine[0].Substring(6)
$openBegin = [Array]::IndexOf($inventory,'__OPEN_BEGIN')
$openEnd = [Array]::IndexOf($inventory,'__OPEN_END')
if ($openBegin -lt 0 -or $openEnd -le $openBegin) { throw 'Open-file inventory failed; nothing deleted.' }
$openRows = @($inventory[($openBegin+1)..($openEnd-1)])
$items = @($inventory | ForEach-Object {
    if ($_ -match '^__FILE\|([0-9]+)\|([0-9]+)\|(.+)$') {
        [pscustomobject]@{ Modified = [long]$Matches[1]; Bytes = [long]$Matches[2]; Path = $Matches[3] }
    }
} | Sort-Object Modified,Path)
Write-Host "Found $($items.Count) log files. Active files and logs younger than $MinimumAgeSeconds seconds will be retained."

try {
    foreach ($item in $items) {
        $remotePath = $item.Path
        if (-not (Test-LogPath $remotePath)) { Write-Warning "Skipping unsupported log path: $remotePath"; continue }
        $isOpen = @($openRows | Where-Object { $_ -match ('(?:^|\s)' + [regex]::Escape($remotePath) + '(?:\s+\(deleted\))?$') }).Count -gt 0
        if ($isOpen -or ($robotNow-$item.Modified) -lt $MinimumAgeSeconds) {
            Write-Host "Retaining active/recent log: $remotePath"; continue
        }
        if ($DryRun) { Write-Host "Would archive and $(if ($DownloadOnly) {'retain'} else {'delete after verification'}): $remotePath ($($item.Bytes) bytes)"; continue }
        $relative = $remotePath.Substring('/home/lvuser/logs/'.Length).Replace('/',[IO.Path]::DirectorySeparatorChar)
        $localPath = [IO.Path]::GetFullPath((Join-Path $sessionRoot $relative))
        if (-not $localPath.StartsWith($sessionRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) {
            throw "Archive path escaped the session directory: $localPath"
        }
        New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($localPath)) -Force | Out-Null
        $partialPath = $localPath + '.' + [Guid]::NewGuid().ToString('N') + '.partial'
        Write-Host "Copying $remotePath ($([Math]::Round($item.Bytes/1MB,1)) MiB)..."
        $previousPreference = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            & scp -O -C @sshOptions ($destination + ':' + $remotePath) $partialPath
            $copyExit = $LASTEXITCODE
        } finally { $ErrorActionPreference = $previousPreference }
        if ($copyExit -ne 0) { throw "Copy failed for $remotePath; source retained." }
        $localBytes = (Get-Item -LiteralPath $partialPath).Length
        $localHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $partialPath).Hash.ToLowerInvariant()
        $checkCommand = "[ -f '$remotePath' ] && [ ! -L '$remotePath' ] || exit 74; stat -c '__META|%Y|%s' '$remotePath'; sha256sum '$remotePath'"
        $check = @(Invoke-RobotSsh $checkCommand)
        $metadata = @($check | Where-Object { $_ -match '^__META\|[0-9]+\|[0-9]+$' })
        $hashRows = @($check | Where-Object { $_ -match '^[0-9a-fA-F]{64}\s+' })
        if ($metadata.Count -ne 1 -or $hashRows.Count -ne 1) { throw "Incomplete remote verification for $remotePath; source retained." }
        $remoteHash = $hashRows[0].Substring(0,64).ToLowerInvariant()
        $expectedMeta = "__META|$($item.Modified)|$($item.Bytes)"
        $verified = $metadata[0] -eq $expectedMeta -and $localBytes -eq $item.Bytes -and $localHash -eq $remoteHash
        if (-not $verified) {
            $manifest.Add([pscustomobject]@{ Remote=$remotePath; Local=$partialPath; SHA256=$localHash; Bytes=$localBytes; Status='ChangedOrUnverified-Retained' })
            Save-Manifest
            Write-Warning "Log changed during copying or checksum did not match. Source retained: $remotePath"
            continue
        }
        Move-Item -LiteralPath $partialPath -Destination $localPath
        $copiedBytes += $localBytes
        $entry = [pscustomobject]@{ Remote=$remotePath; Local=$localPath; SHA256=$localHash; Bytes=$localBytes; Status='Verified-RemoteRetained' }
        $manifest.Add($entry)
        Save-Manifest
        if ($DownloadOnly) { Write-Host "Verified and retained remotely: $remotePath"; continue }
        # No remote temporary files: disk can be full. Every guard runs again in
        # the same SSH command as the single literal-file deletion.
        $deleteTemplate = @'
p='__PATH__'
[ -f "$p" ] && [ ! -L "$p" ] || exit 74
[ "$(readlink -f "$p")" = "$p" ] || exit 75
meta=$(stat -c '%Y|%s' "$p") || exit 76
[ "$meta" = '__MTIME__|__SIZE__' ] || { printf '__RETAIN|changed\n'; exit 0; }
now=$(date +%s) || exit 77
[ "$((now - __MTIME__))" -ge __AGE__ ] || { printf '__RETAIN|recent\n'; exit 0; }
hash=$(sha256sum "$p") || exit 78
hash=${hash%% *}
[ "$hash" = '__HASH__' ] || { printf '__RETAIN|checksum-changed\n'; exit 0; }
opened=$(lsof 2>/dev/null) || exit 79
printf '%s\n' "$opened" | awk -v p="$p" '$NF == p {found=1} END {exit !found}'
open_status=$?
case "$open_status" in
    0) printf '__RETAIN|open\n'; exit 0 ;;
    1) ;;
    *) exit 81 ;;
esac
[ "$(stat -c '%Y|%s' "$p")" = '__MTIME__|__SIZE__' ] || { printf '__RETAIN|changed\n'; exit 0; }
rm -- "$p" || exit 80
printf '__DELETED|__SIZE__\n'
'@
        $deleteCommand = $deleteTemplate.Replace('__PATH__',$remotePath).Replace('__MTIME__',"$($item.Modified)").Replace('__SIZE__',"$($item.Bytes)").Replace('__AGE__',"$MinimumAgeSeconds").Replace('__HASH__',$localHash)
        $entry.Status = 'Verified-DeleteRequested-ConfirmationPending'
        Save-Manifest
        $deletion = @(Invoke-RobotSsh $deleteCommand)
        if ($deletion -contains "__DELETED|$($item.Bytes)") {
            $entry.Status = 'Verified-Deleted'; $freedBytes += $item.Bytes
            Write-Host "Verified, archived and deleted: $remotePath"
        } elseif (@($deletion | Where-Object { $_ -match '^__RETAIN\|' }).Count -gt 0) {
            $entry.Status = 'Verified-RemoteRetained'
            Write-Host "Verified locally; source became active or changed, so retained: $remotePath"
        } else { throw "Deletion status was not confirmed for $remotePath. Check manifest before retrying." }
        Save-Manifest
    }
} finally {
    Save-Manifest
    Write-Host "Local archive session: $sessionRoot"
    Write-Host "Copied $([Math]::Round($copiedBytes/1GB,3)) GiB; freed $([Math]::Round($freedBytes/1GB,3)) GiB."
    Write-Host "Manifest: $manifestPath"
}
Write-Host 'Final robot disk usage:'
Invoke-RobotSsh 'df -k /home/lvuser' | ForEach-Object { Write-Host $_ }
