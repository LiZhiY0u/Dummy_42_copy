param([switch]$Test, [ValidatePattern('^[A-Za-z0-9_-]+$')][string]$BuildName='host')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$qtBin = 'E:\qt\5.14.2\mingw73_64\bin'
$compilerBin = 'E:\qt\Tools\mingw730_64\bin'
$originalTaskPath = $env:Path
$driveLetter = $null
$mappedDrive = $false
try {
    $env:Path = "$compilerBin;$qtBin;$originalTaskPath"
    $buildPath = Join-Path $projectRoot "build\$BuildName"
    New-Item -ItemType Directory -Force -Path $buildPath | Out-Null
    foreach ($candidate in @('R','S','T','U','V','W','X','Y','Z')) {
        if (!(Test-Path "${candidate}:\") -and !(Get-PSDrive -Name $candidate -ErrorAction SilentlyContinue)) {
            $driveLetter = $candidate
            break
        }
    }
    if (!$driveLetter) { throw 'No free drive letter for the Qt 5 ASCII build path' }
    & subst "${driveLetter}:" $projectRoot
    if ($LASTEXITCODE -ne 0) { throw 'subst failed' }
    $mappedDrive = $true
    $aliasRoot = "${driveLetter}:\"
    Push-Location "${aliasRoot}build\$BuildName"
    try {
        & "$qtBin\qmake.exe" -r "${aliasRoot}host\host.pro" 'CONFIG+=debug' 'CONFIG-=release'
        if ($LASTEXITCODE -ne 0) { throw "qmake failed: $LASTEXITCODE" }
        & "$compilerBin\mingw32-make.exe" -j4
        if ($LASTEXITCODE -ne 0) { throw "build failed: $LASTEXITCODE" }
        if ($Test) {
            $testProcess = Start-Process -FilePath (Join-Path $buildPath 'tests\debug\host_tests.exe') `
                -ArgumentList '-txt' -WindowStyle Hidden -Wait -PassThru `
                -RedirectStandardOutput (Join-Path $buildPath 'tests.log') `
                -RedirectStandardError (Join-Path $buildPath 'tests.err')
            Get-Content -LiteralPath (Join-Path $buildPath 'tests.log')
            Get-Content -LiteralPath (Join-Path $buildPath 'tests.err')
            if ($testProcess.ExitCode -ne 0) { throw "tests failed: $($testProcess.ExitCode)" }
        }
    } finally { Pop-Location }
} finally {
    if ($mappedDrive) { & subst "${driveLetter}:" /D }
    $env:Path = $originalTaskPath
}
