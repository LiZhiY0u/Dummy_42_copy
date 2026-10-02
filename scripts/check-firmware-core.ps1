param()
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$armCompiler = 'E:\keil5_2_4\ARM\ARMCLANG\bin\armclang.exe'
if (!(Test-Path -LiteralPath $armCompiler)) { throw 'Existing ARM Compiler 6.7 not found; no tools will be installed.' }
$outputPath = Join-Path $projectRoot 'build\p4-arm'
New-Item -ItemType Directory -Force -Path $outputPath | Out-Null
& $armCompiler --version
foreach ($sourceName in @('protocol_v1','control_gate','command_dispatcher','control_service')) {
    & $armCompiler --target=arm-arm-none-eabi -mcpu=cortex-m3 -std=c++11 -Wall -Wextra -Werror `
        -fno-exceptions -fno-rtti -c (Join-Path $projectRoot "firmware\core\$sourceName.cpp") `
        -o (Join-Path $outputPath "$sourceName.o")
    if ($LASTEXITCODE -ne 0) { throw "ARM compile failed: $sourceName" }
}
@'
#include "protocol_v1.h"
#include "control_gate.h"
#include "command_dispatcher.h"
#include "control_service.h"
static_assert(sizeof(stepper::Parser)==156,"ARM parser layout changed");
static_assert(sizeof(stepper::Frame)==144,"ARM frame layout changed");
static_assert(sizeof(stepper::ControlGate)==16,"ARM control gate layout changed");
static_assert(sizeof(stepper::CommandDispatcher)==1196,"ARM dispatcher layout changed");
static_assert(sizeof(stepper::ControlSnapshot)==52,"ARM control snapshot layout changed");
static_assert(sizeof(stepper::ControlMailbox)==232,"ARM mailbox layout changed");
static_assert(sizeof(stepper::ControlService)==92,"ARM control service layout changed");
'@ | Set-Content -LiteralPath (Join-Path $outputPath 'layout.cpp') -Encoding utf8
& $armCompiler --target=arm-arm-none-eabi -mcpu=cortex-m3 -std=c++11 -Wall -Wextra -Werror `
    -I (Join-Path $projectRoot 'firmware\core') -c (Join-Path $outputPath 'layout.cpp') `
    -o (Join-Path $outputPath 'layout.o')
if ($LASTEXITCODE -ne 0) { throw 'ARM layout verification failed' }
Write-Output 'ARM object compile passed. sizeof Parser=156B, Frame=144B, ControlGate=16B, CommandDispatcher=1196B, ControlMailbox=232B, ControlService=92B, ControlSnapshot=52B. Full firmware link not performed.'
