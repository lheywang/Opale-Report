/** ================================================================
 * @file    application/src/peripherals/saadc/saadc.cpp
 *
 * @brief   saadc.c implement some low level functions for the
 *          control of the SAADC (Successive approximation
 *          analog to digital converter).
 *
 * @date    25-02-2025
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

// nRFX
#include <nrfx.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_dppi.h> // This line must be replaced with nrfx_ppi for nRF52 and less series.

// Libs
#include "init/init.h"
#include "config.h"
#include "peripherals/saadc.h"

// Zephyr
#include <zephyr/logging/log.h>

/* -----------------------------------------------------------------
 * LOGGER CONFIG
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(SAADC, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * FUNCTIONS TO COMMAND AN SAADC
 * -----------------------------------------------------------------
 */

int SAADC_Configure()
{
    // Fetch the timer
    nrfx_timer_t *Target_Timer = INIT_GetATimer(TIMERS::SAADC_TIMER);

    // Initialize variables (and remove warnings)
    saadc_buffer_index = 0;
    memset((void *)saadc_buffer, 0, 2 * SAADC_BUFFER_SIZE);
    int err = 0;

    // First, configure the timer to default settings
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000);
    err = nrfx_timer_init(Target_Timer, &timer_config, NULL);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to initialize the timer for the SAADC: %08x", err);
        return -1;
    }

    // Then, configure the timer for OUR needs :
    uint32_t timer_ticks = nrfx_timer_us_to_ticks(Target_Timer,
                                                  SAADC_SAMPLE_INTERVAL_US);
    nrfx_timer_extended_compare(Target_Timer,
                                NRF_TIMER_CC_CHANNEL0,
                                timer_ticks,
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                false);

    // Initializing the interrupt handler
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
                DT_IRQ(DT_NODELABEL(adc), priority),
                nrfx_isr, nrfx_saadc_irq_handler, 0);

    // Ensure that the ADC is working fine
    err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to initialize the SAADC module: %08x", err);
        return -2;
    }

    // Configure the ADC gain
    for (uint8_t k = 0; k < 8; k++)
        channels[k].channel_config.gain = NRF_SAADC_GAIN1_6;

    // Configuring the channels used
    err = nrfx_saadc_channels_config((const nrfx_saadc_channel_t *)channels, 8);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to configure the SAADC gain: %08x", err);
        return -3;
    }

    // Configure the SAADC operation mode (NO OVERSAMPLING, NO BURST, no autostart...)
    nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    err = nrfx_saadc_advanced_mode_set(255UL,
                                       NRF_SAADC_RESOLUTION_12BIT,
                                       &saadc_adv_config,
                                       saadc_event_handler);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to configure advanced mode of the SAADC: %08x", err);
        return -4;
    }

    // Pass the double buffers to the ADC
    err = nrfx_saadc_buffer_set((void *)saadc_buffer[0], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to set the buffer 0: %08x", err);
        return -5;
    }
    err = nrfx_saadc_buffer_set((void *)saadc_buffer[1], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to set the buffer 1: %08x", err);
        return -6;
    }

    // Prepare the ADC to be able to sample
    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("nrfx_saadc_mode_trigger error: %08x", err);
        return -7;
    }

    // Configuring PPI
    uint8_t m_saadc_sample_ppi_channel;
    uint8_t m_saadc_start_ppi_channel;

    // PPI configure
    err = nrfx_gppi_channel_alloc(&m_saadc_sample_ppi_channel);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to allocate GPPI : %08x", err);
        return -8;
    }

    err = nrfx_gppi_channel_alloc(&m_saadc_start_ppi_channel);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to allocate a GPPI channels : %08x", err);
        return -9;
    }

    // Configure GPPI channels endpoints
    nrfx_gppi_channel_endpoints_setup(m_saadc_sample_ppi_channel,
                                      nrfx_timer_compare_event_address_get(Target_Timer, NRF_TIMER_CC_CHANNEL0),
                                      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

    nrfx_gppi_channel_endpoints_setup(m_saadc_start_ppi_channel,
                                      nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
                                      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

    nrfx_gppi_channels_enable(BIT(m_saadc_sample_ppi_channel));
    nrfx_gppi_channels_enable(BIT(m_saadc_start_ppi_channel));

    // Launch the timer
    nrfx_timer_enable(Target_Timer);
    return 0;
}

void saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
    nrfx_err_t err;
    switch (p_event->type)
    {
    // The SAADC need a new buffer
    case nrfx_saadc_evt_type_t::NRFX_SAADC_EVT_BUF_REQ:
    {
        err = nrfx_saadc_buffer_set((void *)saadc_buffer[(saadc_buffer_index++) % 2], SAADC_BUFFER_SIZE);
        if (err != NRFX_SUCCESS)
        {
            LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
            return;
        }
        break;
    }

    // The SAADC has filled a buffer.
    case nrfx_saadc_evt_type_t::NRFX_SAADC_EVT_DONE:
    {
        double averages[SAADC_INPUT_COUNT];
        double minimals[SAADC_INPUT_COUNT];
        double maximals[SAADC_INPUT_COUNT];

        for (uint8_t k = 0; k < SAADC_INPUT_COUNT; k++)
        {
            averages[k] = 0.0;

            minimals[k] = (double)INT16_MAX;
            maximals[k] = (double)INT16_MIN;
        }

        int16_t current = 0;

        for (int i = 0; i < p_event->data.done.size; i++)
        {
            current = ((int16_t *)(p_event->data.done.p_buffer))[i];

            int8_t chan = (i % SAADC_INPUT_COUNT);
            averages[chan] += ((double)current * K);

            if (current > maximals[chan])
            {
                maximals[chan] = ((double)current * K);
            }
            if (current < minimals[chan])
            {
                minimals[chan] = ((double)current * K);
            }
        }

        for (uint8_t k = 0; k < SAADC_INPUT_COUNT; k++)
        {
            averages[k] = averages[k] / (p_event->data.done.size / SAADC_INPUT_COUNT);
            averages[k] = averages[k] * (3.3 / 4096.0);

            minimals[k] = minimals[k] * (3.3 / 4096.0);
            maximals[k] = maximals[k] * (3.3 / 4096.0);
        }

        // LOG_INF("SAADC buffer at 0x%x filled with %d samples", (uint32_t)p_event->data.done.p_buffer, p_event->data.done.size);
        // for (uint8_t k = 0; k < SAADC_INPUT_COUNT; k++)
        // {
        //     LOG_INF("Channel %d (Pin %d) has : average %.3f, min %.3f, max %.3f",
        //             k,
        //             channels[k].pin_p,
        //             averages[k],
        //             minimals[k],
        //             maximals[k]);
        // }
        // TODO Push on a FIFO here !

        break;
    }

    // Unknown event...
    default:
    {
        LOG_WRN("Unhandled SAADC evt %d", p_event->type);
        break;
    }
    }
}
