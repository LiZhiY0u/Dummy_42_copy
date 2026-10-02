param([string]$Port='COM8')
$ErrorActionPreference='Stop'
function FrameBytes([uint32]$Token,[uint16]$Sequence,[uint16]$Command) {
    $data=[System.Collections.Generic.List[byte]]::new()
    $data.AddRange([byte[]](0xAA,0x55,1,1))
    $isHello=$Command -eq 1
    $data.AddRange([BitConverter]::GetBytes([uint16]$(if($isHello){4}else{0})))
    $data.AddRange([BitConverter]::GetBytes([uint32]$(if($isHello){0}else{$Token})))
    $data.AddRange([BitConverter]::GetBytes($Sequence))
    $data.AddRange([BitConverter]::GetBytes($Command))
    if($isHello){$data.AddRange([BitConverter]::GetBytes($Token))}
    [uint32]$crc=0xFFFF
    for($i=2;$i -lt $data.Count;$i++) {
        $crc=$crc -bxor ([uint32]$data[$i] -shl 8)
        for($bit=0;$bit -lt 8;$bit++) {
            if($crc -band 0x8000){$crc=(($crc -shl 1) -bxor 0x1021) -band 0xFFFF}
            else {$crc=($crc -shl 1) -band 0xFFFF}
        }
    }
    $data.AddRange([BitConverter]::GetBytes([uint16]$crc))
    return ,$data.ToArray()
}
$serial=[System.IO.Ports.SerialPort]::new($Port,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$serial.Handshake=[System.IO.Ports.Handshake]::None
$serial.ReadTimeout=150
$serial.WriteTimeout=1000
try {
    $serial.Open()
    $serial.DiscardInBuffer()
    [uint32]$token=0x12345678
    [uint16]$sequence=1
    # Read-only state probes and HELLO/heartbeat only. Never enable/move/calibrate.
    foreach($command in [uint16[]](1,4,2,3,4)) {
        [byte[]]$tx=FrameBytes $token $sequence $command
        "TX cmd=$command seq=$sequence $([BitConverter]::ToString($tx))"
        $serial.Write($tx,0,$tx.Length)
        $rx=[System.Collections.Generic.List[byte]]::new()
        $watch=[Diagnostics.Stopwatch]::StartNew()
        while($watch.ElapsedMilliseconds -lt 100) {
            $available=$serial.BytesToRead
            if($available -gt 0) {
                $buffer=[byte[]]::new($available)
                $count=$serial.Read($buffer,0,$available)
                for($i=0;$i -lt $count;$i++) {$rx.Add($buffer[$i])}
            }
            Start-Sleep -Milliseconds 2
        }
        "RX [$($rx.Count)B] $([BitConverter]::ToString($rx.ToArray()))"
        $sequence++
    }
} finally { if($serial.IsOpen){$serial.Close()};$serial.Dispose() }
