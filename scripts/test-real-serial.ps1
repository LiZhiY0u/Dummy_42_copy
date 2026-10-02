param([string]$Port='COM8',[string]$BuildName='host-p4')
$ErrorActionPreference='Stop'
$savedPhysicalPort=$env:STEPPER_TEST_PORT
try {
    $env:STEPPER_TEST_PORT=$Port
    & (Join-Path $PSScriptRoot 'build-host.ps1') -Test -BuildName $BuildName
} finally {$env:STEPPER_TEST_PORT=$savedPhysicalPort}
