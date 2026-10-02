param([Parameter(Mandatory=$true)][string]$P9Project)
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$checkScript=Join-Path $projectRoot 'scripts/check-firmware-core.ps1'
$shellExe=(Get-Process -Id $PID).Path
$wrongCopy=Join-Path $PSScriptRoot 'fixtures/P9'
$missingCopy=Join-Path $PSScriptRoot 'fixtures'
$missingProject=Join-Path $PSScriptRoot 'fixtures/no-such-project'
foreach($path in @($wrongCopy,$missingCopy,$missingProject)) {
    $output=& $shellExe -NoProfile -File $checkScript -P9Project $path 2>&1
    if($LASTEXITCODE -eq 0) {throw "Copy gate accepted invalid project: $path"}
    Write-Output "PASS: rejected $path"
}
& $shellExe -NoProfile -File $checkScript -P9Project $P9Project
if($LASTEXITCODE -ne 0) {throw 'Copy gate rejected real matching project'}
'PASS: copy gate rejects drift/missing header/missing project and accepts real matching P9'
