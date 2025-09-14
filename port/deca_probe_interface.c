/*! ----------------------------------------------------------------------------
 * @file        deca_probe_interface.c
 * @author      Alberto Facheris
 */


#include "deca_probe_interface.h"
#include "../deca_driver/deca_interface.h"
#include "port.h"


extern const struct dwt_driver_s dw3000_driver;
const struct dwt_driver_s* tmp_ptr[] = {&dw3000_driver};

static const struct dwt_spi_s dw3000_spi_fct =
{
    .readfromspi = deca_read_from_spi,
    .writetospi = deca_write_to_spi,
    .writetospiwithcrc = deca_write_to_spi_with_crc,
    .setslowrate = deca_set_spi_slow_rate,
    .setfastrate = deca_set_spi_fast_rate,
};

const struct dwt_probe_s dw3000_probe_interf = 
{
    .dw = NULL,
    .spi = (void*)&dw3000_spi_fct,
    .wakeup_device_with_io = deca_wakeup_device_with_io,
    .driver_list = (struct dwt_driver_s **)tmp_ptr,
    .dw_driver_num = 1,
};