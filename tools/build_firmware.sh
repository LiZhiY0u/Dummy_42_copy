#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
repo="$PWD"
build="${1:-$repo/build/firmware-v1}"
if ! command -v arm-none-eabi-gcc >/dev/null; then
    if [ -f /workspace/.setup/activate.sh ]; then
        source /workspace/.setup/activate.sh
    else
        echo 'Install ARM GCC/newlib, CMake and Make, then put arm-none-eabi-gcc on PATH.' >&2
        exit 1
    fi
fi
cmake -S "$repo/原版/Ctrl-Step-Driver-STM32F1-fw" -B "$build" \
    -DCMAKE_BUILD_TYPE=MinSizeRel -DCMAKE_EXE_LINKER_FLAGS=--specs=nosys.specs
cmake --build "$build" -j 2
arm-none-eabi-size "$build/Ctrl-Step-STM32-fw.elf"
test -s "$build/Ctrl-Step-STM32-fw.hex"
test -s "$build/Ctrl-Step-STM32-fw.bin"
