/*! ----------------------------------------------------------------------------
 * @file        main.c
 * @author      Alberto Facheris
 */


#include <stdint.h>
#include <stdbool.h>
#include "../../app/app.h"
#include "../../mac/mac.h"
#include "../../phy/phy.h"
#include "../../port/port.h"   
#include <stdio.h>


#define PRECAL_TX_POWER                 0
#define PRECAL_ANT_DELAY                0

#define NUM_ANCHORS                     2                   // Number of anchors

#define BLUE_LED                        3
#define RED_LED                         1

#define GUARD_TIME                      5                   // Guard time (ms)
#define SLEEP_TIME                      100                 // Sleeping time (ms)
#define BLINK_TIME                      100                 // LED blinking time (ms)

#define LIGHT_SPEED                     2.99702547e8f       // Speed of light (m/s)

#define ALERT_DIST                      0.5f                // Alert distance (m)


static uint16_t my_mac_addr = 0x00;
static uint16_t my_pan_id = 0x00;

static uint16_t tag_mac_addr = PAN_COORDINATOR_MAC_ADDR;
static uint16_t anchor_mac_addr[] = {0x06, 0x07};

static uint8_t led_id[] = {BLUE_LED, RED_LED};
static bool led_state[] = {false, false};

static app_ranging_info_t ranging_info;
static float dist[NUM_ANCHORS];

static uint16_t ant_delay = PRECAL_ANT_DELAY ;
static uint32_t tx_power = PRECAL_TX_POWER;
static dwt_config_t config =
{
    .chan = 5,
    .txPreambLength = DWT_PLEN_1024,
    .rxPAC = DWT_PAC16,
    .txCode = 9,
    .rxCode = 9,
    .sfdType = DWT_SFD_IEEE_4A,
    .dataRate = DWT_BR_850K,
    .phrMode = DWT_PHRMODE_STD,
    .phrRate = DWT_PHRRATE_STD,
    .sfdTO = 1025,
    .stsMode = DWT_STS_MODE_OFF,
    .stsLength = DWT_STS_LEN_64,
    .pdoaMode = DWT_PDOA_M0
};


int main (void)
{
    int ret;

    // Initialize LEDs
    led_gpio_init();

    // Initialize DW3000 IC
    ret = phy_device_init();
    if (ret != PHY_SUCCESS) while(1);

    // Configure DW3000 IC
    ret = phy_set_config(&config);
    if (ret != PHY_SUCCESS) while(1);

    // Set TX power
    ret = phy_set_tx_power(tx_power);
    
    // Set antenna delay
    ret = phy_set_ant_delay(ant_delay);

    // set MAC address
    ret = app_set_mac_addr(my_mac_addr);

    // set PAN ID
    ret = app_set_pan_id(my_pan_id);

    // set tag MAC address
    app_set_tag_mac_addr(tag_mac_addr);

    // Set anchors MAC addresses
    if (NUM_ANCHORS > NUM_LEDS || NUM_ANCHORS != sizeof(anchor_mac_addr)/sizeof(uint16_t)) while(1);
    app_set_anchor_mac_addr(anchor_mac_addr, NUM_ANCHORS);

    // Begin loop
    while (1)
    {
        // Wait a bit before proceeding to make sure anchors have receiver switched on
        if (my_mac_addr == tag_mac_addr)
        {
            app_sleep(GUARD_TIME);
        }

        // Run ranging session
        app_run_ieee_802_15_4_schedule();

        // Read ranging session results
        app_get_ranging_info(&ranging_info);
        
        // Compute distances (m)
        for (int k = 0; k < NUM_ANCHORS; k++)
        {
            dist[k] = LIGHT_SPEED * DWT_TIME_UNITS * ranging_info.dist[k];
        }

        // Set LEDs states if tag is too close to the anchors
        for (int k = 0; k < NUM_ANCHORS; k++)
        {
            if (dist[k] < ALERT_DIST && dist[k] > 0 && my_mac_addr == tag_mac_addr)
            {
                led_state[k] = true;
            }
        }

        // Switch on LEDs
        for (int k = 0; k < NUM_ANCHORS; k++)
        {
            led_gpio_write(led_id[k], led_state[k]);
        }
        
        // Sleep while LEDs are ON
        app_sleep(BLINK_TIME);

        // Reset LEDs states
        for (int k = 0; k < NUM_ANCHORS; k++)
        {
            led_state[k] = false;
        }

        // Switch off LEDs
        for (int k = 0; k < NUM_ANCHORS; k++)
        {
            led_gpio_write(led_id[k], led_state[k]);
        }
        
        // Sleep while LEDs are OFF
        app_sleep(SLEEP_TIME);
    }
}