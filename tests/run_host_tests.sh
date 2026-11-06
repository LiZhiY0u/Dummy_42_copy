#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
root="$PWD/原版/Ctrl-Step-Driver-STM32F1-fw"
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
g++ -std=c++17 -Wall -Wextra -I"$root/Ctrl" tests/encoder_test.cpp "$root/Ctrl/Sensor/Encoder/mt6816_base.cpp" -o "$build/encoder"
"$build/encoder"
for name in protocol storage; do
    g++ -std=c++17 -Wall -Wextra -I"$root/UserApp" tests/${name}_test.cpp -o "$build/$name"
    "$build/$name"
done
g++ -std=c++17 -Wall -Wextra -I"$root/UserApp" tests/uart_test.cpp -o "$build/uart"
"$build/uart"
g++ -std=c++17 -Wall -Wextra -Itests/stubs -I"$root/Ctrl" -I"$root/UserApp" tests/motor_test.cpp "$root/Ctrl/Motor/motor.cpp" "$root/Ctrl/Motor/motion_planner.cpp" -o "$build/motor"
"$build/motor"
g++ -std=c++17 -Wall -Wextra -Itests/uart_stubs -I"$root/UserApp" tests/uart_pipeline_test.cpp "$root/UserApp/protocols/interface_uart.cpp" -o "$build/uart_pipeline"
"$build/uart_pipeline"
