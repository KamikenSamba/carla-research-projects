param(
    [ValidateSet("coop", "ego", "mask", "spectator")]
    [string]$Mode = "coop",

    [string]$Scenario = "scenario_A",

    [switch]$StopCarlaAfterRun
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$DataRoot = "D:\CARLA_DATA"
$CarlaHost = "127.0.0.1"
$CarlaPort = 2000
$CarlaReadyTimeoutSec = 60

# Edit these arrays if CARLA or the Python virtual environment is installed elsewhere.
$CarlaExecutableCandidates = @(
    "C:\CARLA\CarlaUnreal.exe",
    "C:\CARLA\CarlaUE4.exe"
)

$PythonExecutableCandidates = @(
    (Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\python.exe"),
    "C:\CARLA\PythonAPI\examples\venv312\Scripts\python.exe",
    "C:\CARLA\PythonAPI\venv312\Scripts\python.exe"
)

function Get-CarlaExecutable {
    foreach ($candidate in $CarlaExecutableCandidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return $candidate
        }
    }

    throw @"
CARLA executable was not found.
Checked:
  $($CarlaExecutableCandidates -join "`n  ")
Please install CARLA under C:\CARLA or update `$CarlaExecutableCandidates in run_experiment.ps1.
"@
}

function Get-PythonExecutable {
    foreach ($candidate in $PythonExecutableCandidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return $candidate
        }
    }

    throw @"
Python virtual environment executable was not found.
Checked:
  $($PythonExecutableCandidates -join "`n  ")
Please set the CARLA Python 3.12 virtual environment path in `$PythonExecutableCandidates near the top of run_experiment.ps1.
"@
}

function Test-CarlaPort {
    param(
        [string]$HostName = $CarlaHost,
        [int]$Port = $CarlaPort,
        [int]$TimeoutMs = 1000
    )

    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $async = $client.BeginConnect($HostName, $Port, $null, $null)
        if (-not $async.AsyncWaitHandle.WaitOne($TimeoutMs, $false)) {
            return $false
        }
        $client.EndConnect($async)
        return $true
    }
    catch {
        return $false
    }
    finally {
        $client.Close()
    }
}

function Start-CarlaServer {
    param([string]$CarlaExe)

    if (Test-CarlaPort) {
        Write-Host "[INFO] CARLA server already appears to be running on port $CarlaPort. Reusing it."
        return $null
    }

    Write-Host "[INFO] Starting CARLA server on port $CarlaPort..."
    return Start-Process `
        -FilePath $CarlaExe `
        -ArgumentList "-carla-port=$CarlaPort" `
        -WorkingDirectory (Split-Path -Parent $CarlaExe) `
        -WindowStyle Hidden `
        -PassThru
}

function Wait-CarlaReady {
    param([int]$TimeoutSec = $CarlaReadyTimeoutSec)

    Write-Host "[INFO] Waiting for CARLA connection..."
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        if (Test-CarlaPort) {
            Write-Host "[INFO] CARLA server is ready."
            return
        }
        Start-Sleep -Seconds 2
    }

    throw "Timed out after $TimeoutSec seconds waiting for CARLA at $CarlaHost`:$CarlaPort."
}

function Initialize-DataDirectories {
    $env:CARLA_DATA_ROOT = $DataRoot
    Write-Host "[INFO] CARLA_DATA_ROOT = $env:CARLA_DATA_ROOT"

    $directories = @(
        (Join-Path $DataRoot "outputs"),
        (Join-Path $DataRoot "outputs\coop_comm"),
        (Join-Path $DataRoot "outputs\ego_ogm"),
        (Join-Path $DataRoot "masks"),
        (Join-Path $DataRoot "logs")
    )

    foreach ($directory in $directories) {
        if (-not (Test-Path -LiteralPath $directory -PathType Container)) {
            New-Item -ItemType Directory -Path $directory -Force | Out-Null
        }
    }
}

function Invoke-ResearchScript {
    param(
        [string]$PythonExe,
        [string]$RunMode,
        [string]$ScenarioName
    )

    [string[]]$scriptArgs = switch ($RunMode) {
        "coop" {
            @("scripts\run_coop_comm.py", "--scenario-file", "configs\scenarios.json", "--scenario", $ScenarioName)
        }
        "ego" {
            @("scripts\run_ego_ogm.py", "--scenario-file", "configs\scenarios.json", "--scenario", $ScenarioName)
        }
        "mask" {
            , "scripts\build_static_mask.py"
        }
        "spectator" {
            , "scripts\show_spectator_pose.py"
        }
    }

    Write-Host "[INFO] Running mode: $RunMode"
    if ($RunMode -eq "coop" -or $RunMode -eq "ego") {
        Write-Host "[INFO] Scenario: $ScenarioName"
    }
    Write-Host "[INFO] Python: $PythonExe"

    & $PythonExe @scriptArgs
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw "Research script failed with exit code $exitCode."
    }
}

$startedCarlaProcess = $null

try {
    Initialize-DataDirectories

    $carlaExe = Get-CarlaExecutable
    Write-Host "[INFO] CARLA executable = $carlaExe"

    $pythonExe = Get-PythonExecutable

    $startedCarlaProcess = Start-CarlaServer -CarlaExe $carlaExe
    Wait-CarlaReady

    Push-Location $ProjectRoot
    try {
        Invoke-ResearchScript -PythonExe $pythonExe -RunMode $Mode -ScenarioName $Scenario
    }
    finally {
        Pop-Location
    }

    Write-Host "[INFO] Experiment finished successfully."
}
catch {
    Write-Error $_
    exit 1
}
finally {
    if ($StopCarlaAfterRun -and $null -ne $startedCarlaProcess) {
        try {
            if (-not $startedCarlaProcess.HasExited) {
                Write-Host "[INFO] Stopping CARLA process started by this script..."
                Stop-Process -Id $startedCarlaProcess.Id -Force
            }
        }
        catch {
            Write-Warning "Failed to stop CARLA process: $_"
        }
    }
}
