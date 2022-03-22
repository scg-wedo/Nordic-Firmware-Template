#include "FreeRTOS.h"
#include "task.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "logger_thread.h"

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;    /**< Definition of Logger thread. */

/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

void start_logger_thread( void )
{
#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 512, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif //NRF_LOG_ENABLED
}


extern "C" void vApplicationIdleHook(void);

void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}