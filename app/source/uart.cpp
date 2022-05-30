
#include "nrf.h"
#include "nrf_drv_power.h"
#include "nrf_serial.h"
#include "app_timer.h"

#include "app_error.h"
#include "app_util.h"
#include "boards.h"

static void serial0_event_handler(struct nrf_serial_s const * p_serial, nrf_serial_event_t event)
{
    UNUSED_PARAMETER(p_serial);
    UNUSED_PARAMETER(event);
}

static void serial1_event_handler(struct nrf_serial_s const * p_serial, nrf_serial_event_t event)
{
    UNUSED_PARAMETER(p_serial);
    UNUSED_PARAMETER(event);
}

const nrf_drv_uart_config_t m_uarte0_drv_config {
    pseltxd : TX_PIN_NUMBER,
    pselrxd : RX_PIN_NUMBER,
    pselcts : NRF_UARTE_PSEL_DISCONNECTED,
    pselrts : NRF_UARTE_PSEL_DISCONNECTED,
    p_context : nullptr,
    hwfc : NRF_UART_HWFC_DISABLED,
    parity : NRF_UART_PARITY_EXCLUDED,
    baudrate : NRF_UART_BAUDRATE_115200,
    interrupt_priority : UART_DEFAULT_CONFIG_IRQ_PRIORITY
};

const nrf_drv_uart_config_t m_uarte1_drv_config {
    pseltxd : ARDUINO_SCL_PIN,
    pselrxd : ARDUINO_SDA_PIN,
    pselcts : NRF_UARTE_PSEL_DISCONNECTED,
    pselrts : NRF_UARTE_PSEL_DISCONNECTED,
    p_context : nullptr,
    hwfc : NRF_UART_HWFC_DISABLED,
    parity : NRF_UART_PARITY_EXCLUDED,
    baudrate : NRF_UART_BAUDRATE_115200,
    interrupt_priority : UART_DEFAULT_CONFIG_IRQ_PRIORITY
};

#define SERIAL_FIFO_TX_SIZE 32
#define SERIAL_FIFO_RX_SIZE 32

NRF_SERIAL_QUEUES_DEF(serial0_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_QUEUES_DEF(serial1_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);


#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1

NRF_SERIAL_BUFFERS_DEF(serial0_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial1_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);


NRF_SERIAL_CONFIG_DEF(serial0_config, NRF_SERIAL_MODE_DMA,
                      &serial0_queues, &serial0_buffs, serial0_event_handler, NULL);
NRF_SERIAL_CONFIG_DEF(serial1_config, NRF_SERIAL_MODE_DMA,
                      &serial1_queues, &serial1_buffs, serial1_event_handler, NULL);


NRF_SERIAL_UART_DEF(serial0_uarte, 0);
NRF_SERIAL_UART_DEF(serial1_uarte, 1);


void uart_initialise()
{
    ret_code_t ret;

    ret = nrf_serial_init(&serial0_uarte, &m_uarte0_drv_config, &serial0_config);
    APP_ERROR_CHECK(ret);

    ret = nrf_serial_init(&serial1_uarte, &m_uarte1_drv_config, &serial1_config);
    APP_ERROR_CHECK(ret);

    static char tx_message[] = "Hello nrf_serial!\n\r";

    ret = nrf_serial_write(&serial1_uarte,
                           tx_message,
                           strlen(tx_message),
                           NULL,
                           NRF_SERIAL_MAX_TIMEOUT);
    (void)nrf_serial_flush(&serial1_uarte, 0);

    while (true)
    {
        char c;
        ret = nrf_serial_read(&serial0_uarte, &c, sizeof(c), NULL, 1000);
        if (ret != NRF_SUCCESS)
        {
            continue;
        }
        (void)nrf_serial_write(&serial0_uarte, &c, sizeof(c), NULL, 0);
        (void)nrf_serial_flush(&serial0_uarte, 0);

        ret = nrf_serial_read(&serial1_uarte, &c, sizeof(c), NULL, 1000);
        if (ret != NRF_SUCCESS)
        {
            continue;
        }
        (void)nrf_serial_write(&serial1_uarte, &c, sizeof(c), NULL, 0);
        (void)nrf_serial_flush(&serial1_uarte, 0);
    }
}