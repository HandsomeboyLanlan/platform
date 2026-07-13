param(
    [string]$Port = "COM15",
    [int]$BaudRate = 115200,
    [int]$Seconds = 10,
    [string]$Axis = "",
    [double]$Kp = -1.0,
    [double]$Ki = -1.0,
    [double]$Kd = -1.0,
    [string]$LogPath = ""
)

# VOFA CSV列定义:
# 0 time_ms, 1 target_valid, 2 pixel_dx, 3 pixel_dy,
# 4 yaw_cmd_speed, 5 pitch_cmd_speed,
# 6 yaw_motor_speed, 7 pitch_motor_speed,
# 8 yaw_angle, 9 pitch_angle, 10 yaw_current, 11 pitch_current

$ErrorActionPreference = "Stop"
$culture = [System.Globalization.CultureInfo]::InvariantCulture

function Parse-Float {
    param([string]$Text)
    $value = 0.0
    $ok = [double]::TryParse($Text, [System.Globalization.NumberStyles]::Float, $culture, [ref]$value)
    if ($ok) {
        return $value
    }
    return $null
}

function New-Summary {
    return @{
        Count = 0
        MeanAbsError = 0.0
        MaxAbsError = 0.0
        MeanAbsCmd = 0.0
        MeanAbsMotor = 0.0
        MeanAbsSpeedDiff = 0.0
        ErrorSignChanges = 0
        LastErrorSign = 0
    }
}

function Update-Summary {
    param(
        [hashtable]$Summary,
        [double]$ErrorValue,
        [double]$CmdSpeed,
        [double]$MotorSpeed
    )

    $absErr = [Math]::Abs($ErrorValue)
    $absCmd = [Math]::Abs($CmdSpeed)
    $absMotor = [Math]::Abs($MotorSpeed)
    $absDiff = [Math]::Abs($CmdSpeed - $MotorSpeed)
    $count = [double]$Summary["Count"]

    $Summary["Count"]++
    $Summary["MeanAbsError"] = ($Summary["MeanAbsError"] * $count + $absErr) / ($count + 1.0)
    $Summary["MeanAbsCmd"] = ($Summary["MeanAbsCmd"] * $count + $absCmd) / ($count + 1.0)
    $Summary["MeanAbsMotor"] = ($Summary["MeanAbsMotor"] * $count + $absMotor) / ($count + 1.0)
    $Summary["MeanAbsSpeedDiff"] = ($Summary["MeanAbsSpeedDiff"] * $count + $absDiff) / ($count + 1.0)

    if ($absErr -gt $Summary["MaxAbsError"]) {
        $Summary["MaxAbsError"] = $absErr
    }

    $sign = 0
    if ($ErrorValue -gt 0.0) {
        $sign = 1
    } elseif ($ErrorValue -lt 0.0) {
        $sign = -1
    }

    if ($sign -ne 0 -and $Summary["LastErrorSign"] -ne 0 -and $sign -ne $Summary["LastErrorSign"]) {
        $Summary["ErrorSignChanges"]++
    }
    if ($sign -ne 0) {
        $Summary["LastErrorSign"] = $sign
    }
}

function Print-Summary {
    param(
        [string]$Name,
        [hashtable]$Summary
    )

    if ($Summary["Count"] -eq 0) {
        Write-Host ("{0}: no valid target samples" -f $Name)
        return
    }

    Write-Host ("{0}: samples={1}, mean_abs_error={2:F2}px, max_abs_error={3:F2}px, mean_abs_cmd={4:F2}rpm, mean_abs_motor={5:F2}rpm, mean_abs_speed_diff={6:F2}rpm, error_sign_changes={7}" -f
        $Name,
        $Summary["Count"],
        $Summary["MeanAbsError"],
        $Summary["MaxAbsError"],
        $Summary["MeanAbsCmd"],
        $Summary["MeanAbsMotor"],
        $Summary["MeanAbsSpeedDiff"],
        $Summary["ErrorSignChanges"])
}

$serial = [System.IO.Ports.SerialPort]::new($Port, $BaudRate, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 200
$serial.WriteTimeout = 500
$serial.NewLine = "`n"

$yaw = New-Summary
$pitch = New-Summary
$rawLines = New-Object System.Collections.Generic.List[string]

try {
    $serial.Open()
    Start-Sleep -Milliseconds 300
    $serial.DiscardInBuffer()

    if ($Axis -ne "") {
        $axisUpper = $Axis.ToUpperInvariant()
        if ($axisUpper -ne "YAW" -and $axisUpper -ne "PITCH") {
            throw "Axis must be YAW or PITCH"
        }
        if ($Kp -lt 0.0 -or $Ki -lt 0.0 -or $Kd -lt 0.0) {
            throw "Kp/Ki/Kd must be >= 0"
        }

        # 向单片机发送在线PID调参命令，命令格式: PID,YAW,kp,ki,kd
        $cmd = [string]::Format($culture, "PID,{0},{1:F6},{2:F6},{3:F6}`r`n", $axisUpper, $Kp, $Ki, $Kd)
        $serial.Write($cmd)
        Write-Host ("sent: {0}" -f $cmd.Trim())
        Start-Sleep -Milliseconds 200
    }

    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    Write-Host "listening $Port at $BaudRate baud for $Seconds seconds..."

    while ([DateTime]::UtcNow -lt $deadline) {
        try {
            $line = $serial.ReadLine().Trim()
        } catch [System.TimeoutException] {
            continue
        }

        if ($line.Length -eq 0) {
            continue
        }

        $rawLines.Add($line)
        $parts = $line.Split(",")
        if ($parts.Count -lt 12) {
            continue
        }

        $targetValid = Parse-Float $parts[1]
        if ($null -eq $targetValid -or $targetValid -lt 0.5) {
            continue
        }

        $pixelDx = Parse-Float $parts[2]
        $pixelDy = Parse-Float $parts[3]
        $yawCmd = Parse-Float $parts[4]
        $pitchCmd = Parse-Float $parts[5]
        $yawMotor = Parse-Float $parts[6]
        $pitchMotor = Parse-Float $parts[7]

        if ($null -ne $pixelDx -and $null -ne $yawCmd -and $null -ne $yawMotor) {
            Update-Summary $yaw $pixelDx $yawCmd $yawMotor
        }
        if ($null -ne $pixelDy -and $null -ne $pitchCmd -and $null -ne $pitchMotor) {
            Update-Summary $pitch $pixelDy $pitchCmd $pitchMotor
        }
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}

if ($LogPath -ne "") {
    $rawLines | Set-Content -Path $LogPath -Encoding UTF8
    Write-Host "saved log: $LogPath"
}

Print-Summary "yaw" $yaw
Print-Summary "pitch" $pitch
