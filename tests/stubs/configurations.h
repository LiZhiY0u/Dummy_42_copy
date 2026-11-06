#pragma once
#include <cstdint>
struct BoardConfig_t { int32_t velocityLimit = 30*51200; bool enableMotorOnBoot = false; uint32_t defaultMode = 1; };
extern BoardConfig_t boardConfig;
