/*! ----------------------------------------------------------------------------
 * @file        serial_uart.c
 * @author      Alberto Facheris
 */


#include <zephyr/drivers/uart.h>
#include "port.h"


#define UART_DEV_NODE               DT_NODELABEL(uart0)


static const struct device *uart_dev;
static struct uart_config uart_cfg;
static volatile bool uart_tx_done = false;
static volatile bool uart_tx_err = false;


static void serial_uart_cb (const struct device *dev, struct uart_event *evt, void *user_data);


int serial_uart_init (void)
{
    int ret;

    uart_dev = DEVICE_DT_GET(UART_DEV_NODE);

    // Check if UART device is ready
    ret = device_is_ready(uart_dev);
    if (!ret)
    {
        return PORT_INIT_ERROR;
    }

    // Set UART parameters
    uart_cfg.baudrate = 115200;
    uart_cfg.parity = UART_CFG_PARITY_NONE;
    uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
    uart_cfg.data_bits = UART_CFG_DATA_BITS_8;
    uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    ret = uart_configure(uart_dev, &uart_cfg);
    if (ret == -ENOSYS)
    {
        return PORT_INIT_ERROR;
    }

    // Set UART callback
    ret = uart_callback_set(uart_dev, serial_uart_cb, NULL);
    if (ret)
    {
        return PORT_INIT_ERROR;
    }

    return PORT_SUCCESS;
}


static void serial_uart_cb (const struct device *dev, struct uart_event *evt, void *user_data)
{
    switch (evt->type)
    {
        case UART_TX_DONE:
            uart_tx_done = true;
        break;

        case UART_TX_ABORTED:
            uart_tx_err = true;
        break;

        default:
        break;
    }
}


int serial_uart_write (uint8_t tx_buf[], uint16_t tx_buf_len)
{
    int ret;

    // Start transmission
    ret = uart_tx(uart_dev, tx_buf, tx_buf_len, SYS_FOREVER_US);
    if (ret)
    {
        uart_tx_done = false;
        uart_tx_err = false;
        return PORT_RUN_ERROR;
    }

    // Wait until transmission is completed or aborted
    bool pending = true;
    while (pending)
    {
        if (uart_tx_done)
        {
            pending = false;
            ret = PORT_SUCCESS;
        }
        else if (uart_tx_err)
        {
            pending = false;
            ret = PORT_RUN_ERROR;
        }      
    }

    // Clear flags
    uart_tx_done = false;
    uart_tx_err = false;

    return ret;
}