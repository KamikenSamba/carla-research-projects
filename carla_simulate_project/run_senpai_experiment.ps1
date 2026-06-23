param(
    [ValidateSet("smoke", "record", "convert", "replay", "future-lstm", "plot", "animate", "pipeline-smoke")]
    [string]$Mode = "smoke",
    [string]$RawCsv = "",
    [string]$ReducedCsv = "",
    [string]$OutputDir = "",
    [string]$HostName = "127.0.0.1",
    [int]$CarlaPort = 2000,
    [int]$UdpPort = 5005,
    [double]$DurationSec = 10.0,
    [int]$StartFrame = -1,
    [int]$EndFrame = -1,
    [int]$SwitchPayloadFrame = -1,
    [int]$FutureDurationTicks = 20,
    [switch]$StopCarlaAfterRun
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$PythonExecutableCandidates = @(
    (Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\python.exe"),
    "C:\CARLA\PythonAPI\examples\venv312\Scripts\python.exe",
    "C:\CARLA\PythonAPI\venv312\Scripts\python.exe"
)
$CarlaExecutableCandidates = @(
    "C:\CARLA\CarlaUE4.exe",
    "C:\CARLA\CarlaUnreal.exe"
)
$LstmModelPath = Join-Path $ProjectRoot "scripts\udp_replay\traj_lstm.pt"

function Get-PythonExecutable {
    foreach ($candidate in $PythonExecutableCandidates) {
        if (Test-Path -LiteralPath $candidate) {
            return $candidate
        }
    }
    throw "Python executable was not found. Update `$PythonExecutableCandidates near the top of this script."
}

function Get-CarlaExecutable {
    foreach ($candidate in $CarlaExecutableCandidates) {
        if (Test-Path -LiteralPath $candidate) {
            return $candidate
        }
    }
    throw "CARLA executable was not found. Expected C:\CARLA\CarlaUE4.exe or C:\CARLA\CarlaUnreal.exe."
}

function Test-CarlaPort {
    param([string]$HostName, [int]$Port)
    return (Test-NetConnection -ComputerName $HostName -Port $Port -WarningAction SilentlyContinue).TcpTestSucceeded
}

function Start-CarlaServer {
    param([int]$Port)
    if (Test-CarlaPort -HostName "127.0.0.1" -Port $Port) {
        Write-Host "[INFO] Reusing existing CARLA server on port $Port."
        return $null
    }
    $exe = Get-CarlaExecutable
    Write-Host "[INFO] Starting CARLA: $exe -carla-port=$Port"
    return Start-Process -FilePath $exe -ArgumentList "-carla-port=$Port" -PassThru
}

function Wait-CarlaReady {
    param([string]$HostName, [int]$Port, [int]$TimeoutSec = 60)
    Write-Host "[INFO] Waiting for CARLA connection..."
    for ($i = 0; $i -lt $TimeoutSec; $i++) {
        if (Test-CarlaPort -HostName $HostName -Port $Port) {
            Write-Host "[INFO] CARLA server is ready."
            return
        }
        Start-Sleep -Seconds 1
    }
    throw "CARLA server did not become ready within $TimeoutSec seconds."
}

function New-RunDirectory {
    param([string]$BaseDir)
    if ([string]::IsNullOrWhiteSpace($BaseDir)) {
        $tag = Get-Date -Format "yyyyMMdd_HHmmss"
        $BaseDir = Join-Path $ProjectRoot "runs\$tag"
    }
    New-Item -ItemType Directory -Force -Path $BaseDir | Out-Null
    return (Resolve-Path -LiteralPath $BaseDir).Path
}

function Invoke-Python {
    param([string[]]$Arguments)
    $python = Get-PythonExecutable
    Write-Host "[INFO] Python = $python"
    Write-Host "[INFO] Running: python $($Arguments -join ' ')"
    Push-Location $ProjectRoot
    try {
        & $python @Arguments
        if ($LASTEXITCODE -ne 0) {
            throw "Python command failed with exit code $LASTEXITCODE."
        }
    }
    finally {
        Pop-Location
    }
}

function Invoke-Smoke {
    param([double]$DurationSec)
    Invoke-Python @(
        "scripts\autopilot_simulation.py",
        "--host", $HostName,
        "--port", "$CarlaPort",
        "--duration", "$DurationSec",
        "--seed", "42"
    )
}

function Invoke-Record {
    param([string]$DestinationCsv, [double]$DurationSec)
    $python = Get-PythonExecutable
    $traffic = $null
    Push-Location $ProjectRoot
    try {
        Write-Host "[INFO] Recording to $DestinationCsv for $DurationSec seconds."
        $trafficArgs = @(
            "scripts\autopilot_simulation.py",
            "--host", $HostName,
            "--port", "$CarlaPort",
            "--duration", "$([Math]::Ceiling($DurationSec + 5))",
            "--seed", "42"
        )
        $traffic = Start-Process -FilePath $python -ArgumentList $trafficArgs -WorkingDirectory $ProjectRoot -PassThru -WindowStyle Hidden
        Start-Sleep -Seconds 2

        $streamArgs = @(
            "scripts\vehicle_state_stream.py",
            "--host", $HostName,
            "--port", "$CarlaPort",
            "--mode", "wait",
            "--include-velocity",
            "--frame-elapsed",
            "--wall-clock",
            "--output", $DestinationCsv
        )
        $stream = Start-Process -FilePath $python -ArgumentList $streamArgs -WorkingDirectory $ProjectRoot -PassThru -WindowStyle Hidden
        Start-Sleep -Seconds ([int][Math]::Ceiling($DurationSec))
        if (-not $stream.HasExited) {
            Stop-Process -Id $stream.Id -Force
        }
        if ($null -ne $traffic) {
            Wait-Process -Id $traffic.Id -Timeout 15 -ErrorAction SilentlyContinue
            if (-not $traffic.HasExited) {
                Stop-Process -Id $traffic.Id -Force
            }
        }
    }
    finally {
        Pop-Location
    }
}

function Invoke-Convert {
    param([string]$SourceCsv, [string]$DestinationCsv)
    Invoke-Python @(
        "scripts\convert_vehicle_state_csv.py",
        $SourceCsv,
        $DestinationCsv
    )
}

function Wait-TextInFile {
    param([string]$Path, [string]$Pattern, [int]$TimeoutSec = 20)
    for ($i = 0; $i -lt $TimeoutSec; $i++) {
        if ((Test-Path -LiteralPath $Path) -and ((Get-Content -LiteralPath $Path -Raw -ErrorAction SilentlyContinue) -match $Pattern)) {
            return $true
        }
        Start-Sleep -Seconds 1
    }
    return $false
}

function Get-CsvFrameBounds {
    param([string]$CsvPath)
    $first = $null
    $last = $null
    Import-Csv -LiteralPath $CsvPath | ForEach-Object {
        $frame = [int]$_.frame
        if ($null -eq $first) {
            $first = $frame
        }
        $last = $frame
    }
    if ($null -eq $first -or $null -eq $last) {
        throw "No frames found in $CsvPath"
    }
    return [PSCustomObject]@{ First = $first; Last = $last }
}

function Invoke-ReplayPair {
    param(
        [string]$ReplayScript,
        [string]$CsvPath,
        [string]$RunDir,
        [string[]]$ReplayExtraArgs,
        [string]$LogPrefix = "replay",
        [int]$SenderStartFrame = -1,
        [int]$SenderEndFrame = -1
    )
    if (-not (Test-Path -LiteralPath $CsvPath)) {
        throw "Input CSV does not exist: $CsvPath"
    }

    $python = Get-PythonExecutable
    $receiverOut = Join-Path $RunDir "$($LogPrefix)_receiver_stdout.txt"
    $receiverErr = Join-Path $RunDir "$($LogPrefix)_receiver_stderr.txt"
    $senderOut = Join-Path $RunDir "$($LogPrefix)_sender_stdout.txt"
    $senderErr = Join-Path $RunDir "$($LogPrefix)_sender_stderr.txt"
    Remove-Item -LiteralPath $receiverOut,$receiverErr,$senderOut,$senderErr -Force -ErrorAction SilentlyContinue

    $receiverArgs = @(
        $ReplayScript,
        "--carla-host", $HostName,
        "--carla-port", "$CarlaPort",
        "--listen-port", "$UdpPort",
        "--enable-completion",
        "--log-level", "INFO"
    ) + $ReplayExtraArgs

    Push-Location $ProjectRoot
    try {
        $receiver = Start-Process -FilePath $python -ArgumentList $receiverArgs -WorkingDirectory $ProjectRoot -RedirectStandardOutput $receiverOut -RedirectStandardError $receiverErr -PassThru -WindowStyle Hidden
        if (-not (Wait-TextInFile -Path $receiverErr -Pattern "Listening for UDP packets" -TimeoutSec 25)) {
            throw "Replay receiver did not start listening. See $receiverErr"
        }

        $senderArgs = @(
            "send_data\send_udp_frames_from_csv.py",
            $CsvPath,
            "--host", $HostName,
            "--port", "$UdpPort",
            "--interval", "0.05",
            "--log-level", "INFO"
        )
        if ($SenderStartFrame -ge 0) {
            $senderArgs += @("--start-frame", "$SenderStartFrame")
        }
        if ($SenderEndFrame -ge 0) {
            $senderArgs += @("--end-frame", "$SenderEndFrame")
        }

        $sender = Start-Process -FilePath $python -ArgumentList $senderArgs -WorkingDirectory $ProjectRoot -RedirectStandardOutput $senderOut -RedirectStandardError $senderErr -PassThru -WindowStyle Hidden
        Wait-Process -Id $sender.Id -Timeout 300
        Wait-Process -Id $receiver.Id -Timeout 300
    }
    finally {
        Pop-Location
    }

    Write-Host "[INFO] Receiver log: $receiverErr"
    Write-Host "[INFO] Sender log:   $senderErr"
}

function Invoke-Replay {
    param([string]$CsvPath, [string]$RunDir)
    Invoke-ReplayPair `
        -ReplayScript "scripts\udp_replay\replay_from_udp.py" `
        -CsvPath $CsvPath `
        -RunDir $RunDir `
        -ReplayExtraArgs @("--max-runtime", "$DurationSec") `
        -LogPrefix "replay" `
        -SenderStartFrame $StartFrame `
        -SenderEndFrame $EndFrame
}

function Invoke-FutureLstm {
    param([string]$CsvPath, [string]$RunDir)
    if (-not (Test-Path -LiteralPath $LstmModelPath)) {
        throw "LSTM model does not exist: $LstmModelPath"
    }
    $bounds = Get-CsvFrameBounds -CsvPath $CsvPath
    $effectiveSwitchFrame = $SwitchPayloadFrame
    $effectiveEndFrame = $EndFrame
    if ($effectiveSwitchFrame -lt 0) {
        $effectiveSwitchFrame = [Math]::Min($bounds.First + 10, $bounds.Last)
    }
    if ($effectiveEndFrame -lt 0) {
        $effectiveEndFrame = [Math]::Min($effectiveSwitchFrame + 20, $bounds.Last)
    }

    $metadata = Join-Path $RunDir "future_metadata.json"
    $collisions = Join-Path $RunDir "future_collisions.csv"
    $actorLog = Join-Path $RunDir "future_actor_log.csv"
    $idMap = Join-Path $RunDir "future_id_map.csv"
    $extra = @(
        "--max-runtime", "$DurationSec",
        "--future-mode", "lstm",
        "--lstm-model", $LstmModelPath,
        "--lstm-device", "cpu",
        "--future-duration-ticks", "$FutureDurationTicks",
        "--metadata-output", $metadata,
        "--collision-log", $collisions,
        "--actor-log", $actorLog,
        "--id-map-file", $idMap
    )
    $extra += @("--switch-payload-frame", "$effectiveSwitchFrame")
    $extra += @("--end-payload-frame", "$effectiveEndFrame")

    Invoke-ReplayPair `
        -ReplayScript "scripts\udp_replay\replay_from_udp_future_exp.py" `
        -CsvPath $CsvPath `
        -RunDir $RunDir `
        -ReplayExtraArgs $extra `
        -LogPrefix "future_lstm" `
        -SenderStartFrame $StartFrame `
        -SenderEndFrame $effectiveEndFrame
}

function Invoke-Plot {
    param([string]$SourceCsv, [string]$RunDir)
    $out = Join-Path $RunDir "trajectories.png"
    Invoke-Python @(
        "scripts\plot_vehicle_trajectories.py",
        $SourceCsv,
        "--paper",
        "--save", $out,
        "--title", "CARLA trajectories"
    )
}

function Invoke-Animate {
    param([string]$SourceCsv, [string]$RunDir)
    $out = Join-Path $RunDir "trajectories.gif"
    Invoke-Python @(
        "scripts\animate_vehicle_trajectories.py",
        $SourceCsv,
        $out,
        "--fps", "10",
        "--history", "60"
    )
}

$carlaProcess = $null
try {
    $carlaProcess = Start-CarlaServer -Port $CarlaPort
    Wait-CarlaReady -HostName $HostName -Port $CarlaPort

    $runDir = New-RunDirectory -BaseDir $OutputDir
    Write-Host "[INFO] ProjectRoot = $ProjectRoot"
    Write-Host "[INFO] OutputDir   = $runDir"

    if ([string]::IsNullOrWhiteSpace($RawCsv)) {
        $RawCsv = Join-Path $runDir "vehicle_states.csv"
    }
    if ([string]::IsNullOrWhiteSpace($ReducedCsv)) {
        $ReducedCsv = Join-Path $runDir "vehicle_states_reduced.csv"
    }

    switch ($Mode) {
        "smoke" {
            Invoke-Smoke -DurationSec $DurationSec
        }
        "record" {
            Invoke-Record -DestinationCsv $RawCsv -DurationSec $DurationSec
        }
        "convert" {
            Invoke-Convert -SourceCsv $RawCsv -DestinationCsv $ReducedCsv
        }
        "replay" {
            Invoke-Replay -CsvPath $ReducedCsv -RunDir $runDir
        }
        "future-lstm" {
            Invoke-FutureLstm -CsvPath $ReducedCsv -RunDir $runDir
        }
        "plot" {
            Invoke-Plot -SourceCsv $RawCsv -RunDir $runDir
        }
        "animate" {
            Invoke-Animate -SourceCsv $RawCsv -RunDir $runDir
        }
        "pipeline-smoke" {
            $raw = Join-Path $runDir "vehicle_states.csv"
            $reduced = Join-Path $runDir "vehicle_states_reduced.csv"
            Invoke-Record -DestinationCsv $raw -DurationSec $DurationSec
            Invoke-Convert -SourceCsv $raw -DestinationCsv $reduced
            $bounds = Get-CsvFrameBounds -CsvPath $reduced
            $smokeEndFrame = [Math]::Min($bounds.First + 30, $bounds.Last)
            Invoke-ReplayPair `
                -ReplayScript "scripts\udp_replay\replay_from_udp.py" `
                -CsvPath $reduced `
                -RunDir $runDir `
                -ReplayExtraArgs @("--max-runtime", "$DurationSec") `
                -LogPrefix "replay" `
                -SenderStartFrame $bounds.First `
                -SenderEndFrame $smokeEndFrame
            Invoke-FutureLstm -CsvPath $reduced -RunDir $runDir
            Invoke-Plot -SourceCsv $raw -RunDir $runDir
            Invoke-Animate -SourceCsv $raw -RunDir $runDir
        }
    }

    Write-Host "[INFO] Completed mode: $Mode"
}
finally {
    if ($StopCarlaAfterRun -and $null -ne $carlaProcess -and -not $carlaProcess.HasExited) {
        Write-Host "[INFO] Stopping CARLA process started by this script: $($carlaProcess.Id)"
        Stop-Process -Id $carlaProcess.Id -Force
    }
}
