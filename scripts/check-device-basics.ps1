param([string]$Port,[switch]$SelfTest,[switch]$LibraryOnly)
$ErrorActionPreference='Stop'
# Intentionally no generic command option, ENABLE, CLEAR_FAULT or calibration.
function Get-ProbeCrc([byte[]]$Bytes,[int]$Start,[int]$Count) {
    [uint32]$crc=65535
    for($i=$Start;$i -lt $Start+$Count;$i++) {
        $crc=$crc -bxor ([uint32]$Bytes[$i] -shl 8)
        for($bit=0;$bit -lt 8;$bit++) {
            if($crc -band 32768){$crc=(($crc -shl 1) -bxor 4129) -band 65535}
            else{$crc=($crc -shl 1) -band 65535}
        }
    }
    return [uint16]$crc
}
function New-ProbeFrame([uint32]$Token,[uint16]$Seq,[uint16]$Cmd) {
    if($Cmd -notin @(1,2,3,4,0x0106)){throw 'Command not permitted by basic probe'}
    $hello=$Cmd -eq 1
    $bytes=[Collections.Generic.List[byte]]::new()
    $bytes.AddRange([byte[]](170,85,1,1))
    $bytes.AddRange([BitConverter]::GetBytes([uint16]$(if($hello){4}else{0})))
    $bytes.AddRange([BitConverter]::GetBytes([uint32]$(if($hello){0}else{$Token})))
    $bytes.AddRange([BitConverter]::GetBytes($Seq))
    $bytes.AddRange([BitConverter]::GetBytes($Cmd))
    if($hello){$bytes.AddRange([BitConverter]::GetBytes($Token))}
    $bytes.AddRange([BitConverter]::GetBytes((Get-ProbeCrc $bytes.ToArray() 2 ($bytes.Count-2))))
    return ,$bytes.ToArray()
}
function Take-ProbeFrame($Buffer) {
    while($Buffer.Count -ge 2) {
        if($Buffer[0] -ne 170 -or $Buffer[1] -ne 85){$Buffer.RemoveAt(0);continue}
        if($Buffer.Count -lt 14){return}
        $data=$Buffer.ToArray()
        $length=[BitConverter]::ToUInt16($data,4)
        if($data[2] -ne 1 -or $data[3] -notin @(1,2,3,4) -or $length -gt 128){$Buffer.RemoveAt(0);continue}
        $total=16+$length
        if($Buffer.Count -lt $total){return}
        if((Get-ProbeCrc $data 2 ($total-4)) -ne [BitConverter]::ToUInt16($data,$total-2)){$Buffer.RemoveAt(0);continue}
        $frame=[pscustomobject]@{Type=$data[3];Session=[BitConverter]::ToUInt32($data,6);Sequence=[BitConverter]::ToUInt16($data,10);Command=[BitConverter]::ToUInt16($data,12);Payload=[byte[]]::new($length)}
        if($length){[Array]::Copy($data,14,$frame.Payload,0,$length)}
        $Buffer.RemoveRange(0,$total)
        return $frame
    }
}
function Invoke-ProbeRequest($Serial,[uint32]$Token,[uint16]$Seq,[uint16]$Cmd) {
    [byte[]]$tx=New-ProbeFrame $Token $Seq $Cmd
    Write-Host "TX cmd=$('{0:X4}' -f $Cmd) seq=$Seq $([BitConverter]::ToString($tx))"
    $Serial.Write($tx,0,$tx.Length)
    $buffer=[Collections.Generic.List[byte]]::new()
    $watch=[Diagnostics.Stopwatch]::StartNew()
    while($watch.ElapsedMilliseconds -lt 200) {
        if($Serial.BytesToRead) {
            $bytes=[byte[]]::new([Math]::Min(256,$Serial.BytesToRead))
            $count=$Serial.Read($bytes,0,$bytes.Length)
            for($i=0;$i -lt $count;$i++){$buffer.Add($bytes[$i])}
            while($null -ne ($f=Take-ProbeFrame $buffer)) {
                if($f.Type -ne 2 -or $f.Session -ne $Token -or $f.Sequence -ne $Seq -or $f.Command -ne $Cmd){continue}
                if($f.Payload.Length -lt 2){throw 'Response missing status'}
                $status=[BitConverter]::ToUInt16($f.Payload,0)
                Write-Host "RX cmd=$('{0:X4}' -f $Cmd) status=$status bytes=$($f.Payload.Length)"
                if($status -ne 0){throw "Device rejected command: $status"}
                return $f
            }
        }
        Start-Sleep -Milliseconds 2
    }
    throw "Response timeout cmd=$('{0:X4}' -f $Cmd); result unknown, no automatic retry"
}
if($LibraryOnly){return}
if($SelfTest) {
    [byte[]]$vector=[Text.Encoding]::ASCII.GetBytes('123456789')
    if((Get-ProbeCrc $vector 0 9) -ne 0x29b1){throw 'CRC vector failed'}
    $buffer=[Collections.Generic.List[byte]]::new()
    [byte[]]$frame=New-ProbeFrame 0x12345678 7 1
    $buffer.AddRange([byte[]](0,170,0));$buffer.AddRange([byte[]]$frame[0..8])
    if($null -ne (Take-ProbeFrame $buffer)){throw 'Partial frame accepted'}
    $buffer.AddRange([byte[]]$frame[9..($frame.Length-1)])
    $f=Take-ProbeFrame $buffer
    if($null -eq $f -or $f.Sequence -ne 7 -or $f.Command -ne 1 -or $f.Payload.Length -ne 4){throw 'Frame parsing failed'}
    $frame[$frame.Length-1]=$frame[$frame.Length-1] -bxor 1
    $buffer.AddRange($frame);$buffer.AddRange((New-ProbeFrame 1 8 3))
    $f=Take-ProbeFrame $buffer
    if($null -eq $f -or $f.Sequence -ne 8){throw 'CRC resynchronization failed'}
    $blocked=$false
    try{New-ProbeFrame 1 1 0x0301 | Out-Null}catch{$blocked=$true}
    if(!$blocked){throw 'Unsafe command accepted'}
    'PASS: CRC vector, partial/noise/CRC recovery and command allowlist; no serial opened'
    return
}
if(!$Port){throw 'Specify -Port COMx, or use -SelfTest without hardware'}
$serial=[IO.Ports.SerialPort]::new($Port,115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$serial.Handshake=[IO.Ports.Handshake]::None
$serial.DtrEnable=$false;$serial.RtsEnable=$false
$serial.ReadTimeout=50;$serial.WriteTimeout=200
try {
    $serial.Open();$serial.DiscardInBuffer()
    [uint32]$token=Get-Random -Minimum 1 -Maximum 2147483647
    [uint16]$seq=1
    foreach($cmd in @(1,0x0106,4,2,3,4,3)) {
        $f=Invoke-ProbeRequest $serial $token $seq ([uint16]$cmd)
        if($cmd -eq 2) {
            if($f.Payload.Length -ne 52){throw 'Unexpected GET_INFO size'}
            Write-Host "UID=$([BitConverter]::ToString($f.Payload,12,12)) capabilities=$('{0:X8}' -f [BitConverter]::ToUInt32($f.Payload,24))"
            Write-Host 'GET_INFO cannot prove firmware HEX identity or physical Flash capacity.'
        }
        if($cmd -in @(3,0x0106)) {
            if($f.Payload.Length -ne 50){throw 'Unexpected state response size'}
            $state=$f.Payload[42];$mode=$f.Payload[43]
            Write-Host "state=$state mode=$mode calibrated=$($f.Payload[44]) encoderValid=$($f.Payload[45]) faults=$('{0:X8}' -f [BitConverter]::ToUInt32($f.Payload,38))"
            if($state -in @(1,2,3) -or $mode -ne 3){throw 'Device not disabled; stop manual validation and check hardware'}
        }
        $seq++
    }
    'PASS: matched CRC-valid replies and disabled STOP feedback; no enable/calibration sent. Not physical stop acceptance.'
} finally {if($serial.IsOpen){$serial.Close()};$serial.Dispose()}
