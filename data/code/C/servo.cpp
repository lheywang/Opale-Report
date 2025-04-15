/** ================================================================
 * @file    application/src/devices/servo.cpp
 *
 * @brief   Servo.cpp implement some low level interactions with the
 *          servo engines.
 *
 * @date    19-02-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */

// Zephyr
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

// nrfx
#include <nrfx_pwm.h>

// Local libs
#include "devices/servo.h"
#include "config.h"
#include "init/init.h"

// STD
#include <math.h>

/* -----------------------------------------------------------------
 * LOGGER CONFIG
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Servo, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * FUNCTIONS TO COMMAND A SERVO
 * -----------------------------------------------------------------
 */

int SERVO_SetPosition(const struct pwm_dt_spec Target[PWM_SERVO_LEN],
                      ServoAngles const *Command)
{

    // Checking the parameters
    // MIN =< Pos <= MAX
    if ((Command->north < PWM_SERVO_MIN_ANGLE) ||
        (Command->south < PWM_SERVO_MIN_ANGLE) ||
        (Command->east < PWM_SERVO_MIN_ANGLE) ||
        (Command->west < PWM_SERVO_MIN_ANGLE))
    {
        LOG_ERR("Incorrect command passed on the servo. At least one command was under the threshold");
        return -1;
    }
    if ((Command->north > PWM_SERVO_MAX_ANGLE) ||
        (Command->south > PWM_SERVO_MAX_ANGLE) ||
        (Command->east > PWM_SERVO_MAX_ANGLE) ||
        (Command->west > PWM_SERVO_MAX_ANGLE))
    {
        LOG_ERR("Incorrect command passed on the servo. At least one command was under the threshold");
        return -1;
    }

    // Computing the pulses length
    // Due to negative values that are offseted, we need to handle that case too.
    float pulses[PWM_SERVO_LEN] = {0};
    for (uint8_t k = 0; k < PWM_SERVO_LEN; k++)
    {
        switch (k)
        {
        case 0:
            pulses[k] = Command->north;
            break;
        case 1:
            pulses[k] = Command->south;
            break;
        case 2:
            pulses[k] = Command->east;
            break;
        case 3:
            pulses[k] = Command->west;
            break;
        }

        pulses[k] += PWM_SERVO_MAX_ANGLE;
        pulses[k] *= PWM_SERVO_MAX_PULSE_WIDTH;
        pulses[k] /= 2 * PWM_SERVO_MAX_RANGE;
        pulses[k] += PWM_SERVO_MIN_PULSE_WIDTH;
    }

    // Configure the PWM engines
    int err = 0;
    for (uint8_t k = 0; k < NRF_PWM_CHANNEL_COUNT; k++)
        err -= pwm_set(Target[k].dev, k, Target[k].period, (int)round(pulses[k]), Target->flags);

    // End log, and return
    if (err != 0)
    {
        LOG_ERR("Failed to configure the PWM duty cycle for the servo : %d", err);
        return -2;
    }
    return 0;
}