#ifndef P9_CALIBRATION_LIFECYCLE_H
#define P9_CALIBRATION_LIFECYCLE_H
#include <stdint.h>
namespace stepper { namespace calibration {
enum class Outcome : uint8_t { Unknown, Pending, Verified, Cancelled, Failed };
// Control boundary starts a generation before setting the output trigger.
// Flash work remains exclusively in the main-loop calibrator.
bool beginTask(uint32_t task);
// Caller separately clears the motion trigger and sleeps the driver.
void requestCancel(uint32_t task);
Outcome outcome(uint32_t task);
uint16_t phasePercent(uint32_t task); // phase in high byte, percent in low
bool storageBusy();
} }
#endif
