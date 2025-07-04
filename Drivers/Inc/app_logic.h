#ifndef __APP_LOGIC_H
#define __APP_LOGIC_H

#include "main.h"

/* Evaluate watering logic, called periodically or when triggers occur.
 * This may set manual_flag in auto mode if threshold condition is met (auto-start watering). */
void evaluate_watering_logic(void);

#endif /* __APP_LOGIC_H */
