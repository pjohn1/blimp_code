# Attach-CP210x-To-WSL-Fix.ps1
param(
    [string]$Distro = "",        # e.g. "Ubuntu-24.04"
    [switch]$Watch,              # Keep watching for new USBs
    [int]$PollIntervalSeconds = 5
)

function Ensure-Admin {
    $isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
    if (-not $isAdmin) {
        Write-Error "This script must be run as Administrator. Right-click PowerShell and 'Run as administrator'."
        exit 1
    }
}

# Parse only the Connected: table of 'usbipd list' output.
# Returns objects: @{ BusId='4-2'; VidPid='10c4:ea60'; Desc='...'; State='Shared' }
function Get-UsbConnectedDevices {
    $out = & usbipd list 2>&1
    if ($LASTEXITCODE -ne 0 -and -not $out) {
        Write-Error "usbipd returned an error. Make sure usbipd is installed and available in PATH."
        return @()
    }

    $lines = $out
    $devices = @()

    $inConnected = $false
    foreach ($line in $lines) {
        $trim = $line.TrimEnd()
        if ($trim -match '^Connected:') {
            $inConnected = $true
            continue
        }
        if ($trim -match '^Persisted:') {
            # Stop parsing Connected section when Persisted: starts
            break
        }
        if (-not $inConnected) { continue }

        # match a Connected table row that begins with BUSID (e.g., "4-2    10c4:ea60  Silicon Labs ...  Shared")
        # We expect: BUSID  VID:PID    DEVICE (may contain spaces)    STATE (Attached/Shared/--)
        $m = [regex]::Match($line, '^\s*(\d+-\d+)\s+([0-9A-Fa-f]{4}:[0-9A-Fa-f]{4})\s+(.+?)\s{2,}(\w+)\s*$')
        if ($m.Success) {
            $bus = $m.Groups[1].Value
            $vidpid = $m.Groups[2].Value.ToLower()
            $desc = $m.Groups[3].Value.Trim()
            $state = $m.Groups[4].Value.Trim()
            $devices += [PSCustomObject]@{ BusId = $bus; VidPid = $vidpid; Desc = $desc; State = $state }
            continue
        }

        # some installs may not separate state column with two spaces; try a fallback that captures bus + vidpid + rest
        $m2 = [regex]::Match($line, '^\s*(\d+-\d+)\s+([0-9A-Fa-f]{4}:[0-9A-Fa-f]{4})\s+(.+)$')
        if ($m2.Success) {
            $bus = $m2.Groups[1].Value
            $vidpid = $m2.Groups[2].Value.ToLower()
            $rest = $m2.Groups[3].Value.Trim()
            # attempt to parse trailing state token if present (Attached|Shared|Available)
            $state = ''
            if ($rest -match '\s+(Attached|Shared|Available)\s*$') {
                $state = $Matches[1]
                $desc = $rest -replace '\s+(Attached|Shared|Available)\s*$',''
            } else {
                $desc = $rest
            }
            $devices += [PSCustomObject]@{ BusId = $bus; VidPid = $vidpid; Desc = $desc; State = $state }
        }
    }

    return $devices
}

function Is-CP210x {
    param($vidpid, $desc)
    if ($null -ne $vidpid -and $vidpid -eq '10c4:ea60') { return $true }
    if ($desc) {
        $d = $desc.ToLower()
        if ($d -match 'cp210' -or $d -match 'silicon labs' -or $d -match 'silabs') { return $true }
    }
    return $false
}

function Attach-BusIdToWsl {
    param($busid, $distro, $desc, $state)

    Write-Host "Processing BUSID: $busid  ($state) -- $desc"

    if ($state -and $state.ToLower() -eq 'attached') {
        Write-Host "  -> Already attached. Skipping."
        return
    }

    try {
        # bind first (safe even if already bound)
        & usbipd bind --busid $busid 2>&1 | Out-Null
    } catch {
        Write-Verbose "bind may have failed or already bound: $_"
    }

    # Build attach command
    if ($distro -and $distro.Trim() -ne "") {
        Write-Host ("Attaching {0} to WSL distro {1}..." -f $busid, $distro)
        $attachOut = & usbipd attach --wsl --distro $distro --busid $busid 2>&1
    } else {
        Write-Host ("Attaching {0} to default WSL instance..." -f $busid)
        $attachOut = & usbipd attach --wsl --busid $busid 2>&1
    }

    if ($LASTEXITCODE -ne 0 -or ($attachOut -match 'error')) {
        Write-Warning "usbipd attach failed for busid $busid. usbipd output:"
        $attachOut | ForEach-Object { Write-Host "  $_" }
        Write-Host "Current usbipd list for debugging:"
        & usbipd list | ForEach-Object { Write-Host "  $_" }
    } else {
        Write-Host "Attach succeeded for $busid"
    }
}

# --- main ---
Ensure-Admin

$seen = [System.Collections.Generic.HashSet[string]]::new()

do {
    $devices = Get-UsbConnectedDevices
    foreach ($dev in $devices) {
        $bus = $dev.BusId
        $vidpid = $dev.VidPid
        $desc = $dev.Desc
        $state = $dev.State

        if ($seen.Contains($bus)) { continue }

        if (-not (Is-CP210x -vidpid $vidpid -desc $desc)) {
            Write-Host "Skipping non-CP210x device: $bus  -- $vidpid  -- $desc ($state)"
            $seen.Add($bus) | Out-Null
            continue
        }

        Attach-BusIdToWsl -busid $bus -distro $Distro -desc $desc -state $state
        $seen.Add($bus) | Out-Null
    }

    if ($Watch) {
        Start-Sleep -Seconds $PollIntervalSeconds
    }
} while ($Watch)

Write-Host "✅ Done scanning and attaching CP210x devices."
