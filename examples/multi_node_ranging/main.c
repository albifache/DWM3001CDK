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
#include <zephyr/sys/printk.h>


#define PRECAL_TX_POWER                 0
#define PRECAL_ANT_DELAY                0

#define GUARD_TIME                      10                  // Guard time (ms)
#define SLEEP_TIME                      1000                // Sleeping time (ms)

#define LIGHT_SPEED                     2.99702547e8f       // Speed of light (m/s)


static uint16_t my_mac_addr = 0x00;
static uint16_t my_pan_id = 0x00;

static uint16_t node_mac_addr[] = {PAN_COORDINATOR_MAC_ADDR, 0x01, 0x03, 0x06, 0x07, 0x09};
static uint8_t num_nodes = sizeof(node_mac_addr) / sizeof(uint16_t);
static uint8_t node_id = 0;

static app_ranging_info_t ranging_info;
static float dist[MAX_NUM_ANCHORS];

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

    // Initialize DW3000 IC
    ret = phy_device_init();
    if (ret != PHY_SUCCESS)
    {
        printk("\nDW3000 initialization failed.\n");
        while(1);
    }
    else
    {
        printk("\nDW3000 initialization successful.\n");
    }

    // Configure DW3000 IC
    ret = phy_set_config(&config);
    if (ret != PHY_SUCCESS)
    {
        printk("\nDW3000 configuration failed.\n");
        while(1);
    }
    else
    {
        printk("\nDW3000 configuration successful.\n");
    }

    // Set TX power
    phy_set_tx_power(tx_power);
    
    // Set antenna delay
    phy_set_ant_delay(ant_delay);

    // Check MAC errors
    if (num_nodes > MAX_NUM_ANCHORS || my_mac_addr == BROADCAST_MAC_ADDR || my_pan_id == BROADCAST_PAN_ID)
    {
        printk("\nMAC configuration error.\n");
        while(1);
    }
    else
    {
        printk("\nMAC configuration successful.\n");
    }

    // set MAC address
    app_set_mac_addr(my_mac_addr);

    // set PAN ID
    app_set_pan_id(my_pan_id);

    // Set anchors MAC addresses
    app_set_anchor_mac_addr(node_mac_addr, num_nodes);

    // Begin loop
    while (1)
    {
        // Wait a bit before proceeding to make sure anchors have receiver switched on
        if (my_mac_addr == PAN_COORDINATOR_MAC_ADDR)
        {
            app_sleep(GUARD_TIME);
        }

        // Wait a bit before proceeding to make sure anchors have receiver switched on
        if (my_mac_addr == PAN_COORDINATOR_MAC_ADDR)
        {
            node_id++;
            node_id %= num_nodes;
            app_set_tag_mac_addr(node_mac_addr[node_id]);
        }

        // Run ranging session
        app_run_ieee_802_15_4_schedule();

        // Read ranging session results
        app_get_ranging_info(&ranging_info);
        
        // Compute distances (m)
        for (int k = 0; k < num_nodes; k++)
        {
            dist[k] = DWT_TIME_UNITS * LIGHT_SPEED * ranging_info.dist[k];
        }

        // Print log message on console
        printk("\n\n\n\n\n\nRanging session %u completed.\n\n", ranging_info.superframe_id);
        for (int k = 0; k < num_nodes; k++)
        {
            printk("\tDistance from node 0x%02X to node 0x%02X: %.2f m.\n",
                node_mac_addr[node_id], node_mac_addr[k], (double) dist[k]);
        }
        
        // Sleep till next superframe
        app_sleep(SLEEP_TIME);
    }
}