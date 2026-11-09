param([string]$Port,[switch]$StartCalibration,[switch]$SelfTest,[switch]$ClearCalibrationFault)
$ErrorActionPreference='Stop'
$calibrationPort=$Port;$calibrationSelfTest=$SelfTest
. (Join-Path $PSScriptRoot 'check-device-basics.ps1') -LibraryOnly
$Port=$calibrationPort;$SelfTest=$calibrationSelfTest
function New-CalibrationFrame([uint32]$Token,[uint16]$Seq,[uint16]$Cmd,[byte[]]$Payload=@()) {
    if($Cmd -notin @(1,2,3,4,0x0106,0x0108,0x0301,0x0302)){throw 'Command not allowed'}
    if($Cmd -eq 1){return ,(New-ProbeFrame $Token $Seq $Cmd)}
    $bytes=[Collections.Generic.List[byte]]::new()
    $bytes.AddRange([byte[]](170,85,1,1))
    $bytes.AddRange([BitConverter]::GetBytes([uint16]$Payload.Length))
    $bytes.AddRange([BitConverter]::GetBytes($Token))
    $bytes.AddRange([BitConverter]::GetBytes($Seq))
    $bytes.AddRange([BitConverter]::GetBytes($Cmd));$bytes.AddRange($Payload)
    $bytes.AddRange([BitConverter]::GetBytes((Get-ProbeCrc $bytes.ToArray() 2 ($bytes.Count-2))))
    return ,$bytes.ToArray()
}
function Exchange-Calibration([uint16]$Cmd,[byte[]]$Payload=@(),[uint16]$Expected=0) {
    $script:Sequence++
    [byte[]]$tx=New-CalibrationFrame $script:Token $script:Sequence $Cmd $Payload
    $script:Serial.Write($tx,0,$tx.Length)
    $watch=[Diagnostics.Stopwatch]::StartNew()
    while($watch.ElapsedMilliseconds -lt 200) {
        if($script:Serial.BytesToRead) {
            $bytes=[byte[]]::new([Math]::Min(256,$script:Serial.BytesToRead))
            $count=$script:Serial.Read($bytes,0,$bytes.Length)
            for($i=0;$i -lt $count;$i++){$script:Buffer.Add($bytes[$i])}
            while($null -ne ($f=Take-ProbeFrame $script:Buffer)) {
                if($f.Session -ne $script:Token){continue}
                if($f.Type -eq 3){Write-Host "EVENT cmd=$('{0:X4}' -f $f.Command) payload=$([BitConverter]::ToString($f.Payload))";continue}
                if($f.Type -ne 2 -or $f.Sequence -ne $script:Sequence -or $f.Command -ne $Cmd){continue}
                if($f.Payload.Length -lt 2){throw 'Missing response status'}
                $status=[BitConverter]::ToUInt16($f.Payload,0)
                if($status -ne $Expected){throw "cmd=$('{0:X4}' -f $Cmd) status=$status expected=$Expected"}
                return $f
            }
        }
        Start-Sleep -Milliseconds 1
    }
    throw "Timeout cmd=$('{0:X4}' -f $Cmd); result unknown, no retry"
}
if($SelfTest) {
    [byte[]]$bytes=New-CalibrationFrame 1 2 0x0302 ([BitConverter]::GetBytes([uint32]7))
    $b=[Collections.Generic.List[byte]]::new();$b.AddRange($bytes)
    $f=Take-ProbeFrame $b
    if($null -eq $f -or $f.Command -ne 0x0302 -or [BitConverter]::ToUInt32($f.Payload,0) -ne 7){throw 'Query vector failed'}
    $blocked=$false;try{New-CalibrationFrame 1 1 0x0101 | Out-Null}catch{$blocked=$true}
    if(!$blocked){throw 'ENABLE must be rejected'}
    'PASS: calibration query encoding and ENABLE rejection; no serial opened'
    return
}
if(!$Port -or !$StartCalibration){throw 'Requires explicit -Port COMx -StartCalibration; this moves the motor'}
$script:Token=[uint32](Get-Random -Minimum 1 -Maximum 2147483647)
$script:Sequence=[uint16]0
$script:Buffer=[Collections.Generic.List[byte]]::new()
$script:Serial=[IO.Ports.SerialPort]::new($Port,115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$script:Serial.Handshake=[IO.Ports.Handshake]::None
$script:Serial.DtrEnable=$false;$script:Serial.RtsEnable=$false
$script:Serial.ReadTimeout=50;$script:Serial.WriteTimeout=200
$ready=$false
try {
    $script:Serial.Open();$script:Serial.DiscardInBuffer()
    $null=Exchange-Calibration 1;$ready=$true
    $s=Exchange-Calibration 0x0106
    if($s.Payload.Length -ne 50 -or $s.Payload[43] -ne 3 -or $s.Payload[45] -ne 1){throw 'STOP/sensor prerequisite failed'}
    $fault=[BitConverter]::ToUInt32($s.Payload,38)
    if($fault -eq 4){$null=Exchange-Calibration 0x0108}
    elseif($ClearCalibrationFault -and $fault -in @(32,36)) {
        Write-Host "Explicitly clearing previous calibration failure fault=$fault after STOP/sensor checks"
        $null=Exchange-Calibration 0x0108
    }
    elseif($fault -ne 0){throw "Uncleared non-communication fault $fault"}
    $null=Exchange-Calibration 4
    $info=Exchange-Calibration 2
    if($info.Payload.Length -ne 52){throw 'Unexpected info size'}
    $cap=[BitConverter]::ToUInt32($info.Payload,24)
    $current=[BitConverter]::ToUInt32($info.Payload,36)
    if(($cap -band 32) -eq 0 -or $current -eq 0 -or $current -gt 1000){throw 'Calibration capability/current prerequisite failed'}
    $s=Exchange-Calibration 3
    if($s.Payload.Length -ne 50 -or $s.Payload[42] -ne 0 -or $s.Payload[43] -ne 3 -or $s.Payload[45] -ne 1 -or [BitConverter]::ToUInt32($s.Payload,38) -ne 0){throw 'Not fault-free disabled with valid encoder'}
    Write-Host "START calibration command-current=${current}mA (not measured phase current)"
    $accepted=Exchange-Calibration 0x0301 @() 1
    if($accepted.Payload.Length -ne 6){throw 'Invalid ACCEPTED response'}
    $id=[BitConverter]::ToUInt32($accepted.Payload,2);if(!$id){throw 'Invalid task ID'}
    Write-Host "ACCEPTED task=$id"
    $watch=[Diagnostics.Stopwatch]::StartNew();$last='';$done=$false
    while($watch.ElapsedMilliseconds -lt 60000) {
        $null=Exchange-Calibration 4
        $q=Exchange-Calibration 0x0302 ([BitConverter]::GetBytes($id))
        if($q.Payload.Length -ne 15 -or [BitConverter]::ToUInt32($q.Payload,2) -ne $id){throw 'Invalid query response'}
        $phase=$q.Payload[6];$pct=$q.Payload[7];$result=[BitConverter]::ToUInt16($q.Payload,8);$verified=$q.Payload[10]
        $line="task=$id phase=$phase percent=$pct result=$result verified=$verified elapsedMs=$($watch.ElapsedMilliseconds)"
        $key="$phase/$pct/$result/$verified"
        if($key -ne $last){Write-Host $line;$last=$key}
        if($phase -eq 6){throw "Calibration failed/cancelled: $line"}
        if($phase -eq 5) {
            if($pct -ne 100 -or $result -ne 0 -or $verified -ne 1){throw 'Unverified completion'}
            $done=$true;break
        }
        Start-Sleep -Milliseconds 50
    }
    if(!$done){throw 'Calibration duration limit reached'}
    $null=Exchange-Calibration 0x0106
    $s=Exchange-Calibration 3
    if($s.Payload.Length -ne 50 -or $s.Payload[42] -ne 0 -or $s.Payload[43] -ne 3 -or $s.Payload[44] -ne 1 -or $s.Payload[45] -ne 1 -or [BitConverter]::ToUInt32($s.Payload,38) -ne 0){throw 'Final calibrated disabled state invalid'}
    'PASS: calibration verified and disabled; no normal ENABLE/motion-target command sent'
} finally {
    if($script:Serial.IsOpen) {
        if($ready){try{$null=Exchange-Calibration 0x0106;Write-Host 'FINAL STOP acknowledged'}catch{Write-Warning "FINAL STOP unknown: $_ ; cut motor power if needed"}}
        $script:Serial.Close()
    }
    $script:Serial.Dispose()
}
