param([switch]$ShowWindow, [ValidatePattern('^[A-Za-z0-9_-]+$')][string]$BuildName='host')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$executable = Join-Path $projectRoot "build\$BuildName\app\debug\StepperHost.exe"
if (!(Test-Path -LiteralPath $executable)) { throw 'Build first: .\scripts\build-host.ps1 -Test' }
$originalTaskPath = $env:Path
try {
    $env:Path = 'E:\qt\Tools\mingw730_64\bin;E:\qt\5.14.2\mingw73_64\bin;' + $originalTaskPath
    $windowStyle = if ($ShowWindow) { 'Normal' } else { 'Hidden' }
    Start-Process -FilePath $executable -WorkingDirectory $projectRoot -WindowStyle $windowStyle
} finally { $env:Path = $originalTaskPath }
