$ErrorActionPreference='Stop'
$shellExe=(Get-Process -Id $PID).Path
$gate=Join-Path $PSScriptRoot 'check-stack.ps1'
$unknown=Join-Path $PSScriptRoot 'fixtures/stack-unknown.htm'
$output=& $shellExe -NoProfile -File $gate -Mode normal -Report $unknown 2>&1
if($LASTEXITCODE -eq 0) {throw 'Stack gate accepted unknown UART/contract frames'}
if(($output -join "`n") -notmatch 'Unknown UART stack frame') {throw 'Unexpected negative-fixture failure'}
'PASS: unknown UART/contract frame rejected before numeric depth acceptance'
& $shellExe -NoProfile -File $gate -Mode normal -Report (Join-Path $PSScriptRoot 'fixtures/stack-known.htm')
if($LASTEXITCODE -ne 0) {throw 'Known-stack fixture failed'}
'PASS: fully described bounded stack accepted'
$known=Join-Path $PSScriptRoot 'fixtures/stack-known.htm'
& $shellExe -NoProfile -File $gate -Mode normal -Report $known -MinimumHeadroom 436
if($LASTEXITCODE -ne 0){throw 'Exact minimum headroom must pass'}
$output=& $shellExe -NoProfile -File $gate -Mode normal -Report $known -MinimumHeadroom 437 2>&1
if($LASTEXITCODE -eq 0 -or ($output -join "`n") -notmatch 'Stack headroom below requested'){throw 'Headroom threshold did not reject insufficient reserve'}
'PASS: requested headroom exact boundary accepted, one-byte deficit rejected'
