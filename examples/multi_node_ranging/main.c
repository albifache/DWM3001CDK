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


#define RF_CHAN                                 5                           // UWB channel (5 or 9)
#define PREAMBLE_CODE                           9                           // Preamble code
#define BIT_RATE                                DWT_BR_850K                 // UWB bit rate
#define PREAMBLE_LEN                            DWT_PLEN_1024               // Preamble length (number of symbols)
#define STS_LEN                                 DWT_STS_LEN_1024            // STS length (number of symbols)

#define MAC_ADDR                                0x00u                       // MAC address of current node
#define PAN_ID                                  0x00u                       // PAN ID of current network

#define STS_KEY_0                               0ul                         // STS key (bits 0-31)
#define STS_KEY_1                               0ul                         // STS key (bits 32-63)
#define STS_KEY_2                               0ul                         // STS key (bits 64-95)
#define STS_KEY_3                               0ul                         // STS key (bits 96-127)

#define AES_KEY_0                               0ul                         // AES key (bits 0-31)
#define AES_KEY_1                               0ul                         // AES key (bits 32-63)
#define AES_KEY_2                               0ul                         // AES key (bits 64-95)
#define AES_KEY_3                               0ul                         // AES key (bits 96-127)

#define NUM_NODES                               6                           // Number of nodes (max 8)

#define NODE_MAC_ADDR_0                         0x00                        // MAC address of node 0
#define NODE_MAC_ADDR_1                         0x01                        // MAC address of node 1
#define NODE_MAC_ADDR_2                         0x03                        // MAC address of node 2
#define NODE_MAC_ADDR_3                         0x06                        // MAC address of node 3
#define NODE_MAC_ADDR_4                         0x07                        // MAC address of node 4
#define NODE_MAC_ADDR_5                         0x09                        // MAC address of node 5
#define NODE_MAC_ADDR_6                         0                           // MAC address of node 6 (unused in this case)
#define NODE_MAC_ADDR_7                         0                           // MAC address of node 7 (unused in this case)

#define GUARD_TIME                              10                          // Guard time (ms)
#define SLEEP_TIME                              1000                        // Sleeping time (ms)

#define LIGHT_SPEED                             2.99702547e8f               // Speed of light (m/s)


static phy_init_obj_t phy_init_obj;

static app_init_obj_t app_init_obj;
static app_ctrl_obj_t app_ctrl_obj;
static app_log_info_t app_log_info;

static uint16_t node_mac_addr[MAX_NUM_ANCHORS];
static uint8_t node_id = 0;

static float dist[MAX_NUM_ANCHORS];


int main (void)
{
    int ret;
    
    // Initialize DW3000 IC
    ret = deca_init();
    if (ret != PORT_SUCCESS)
    {
        printk("\n[ERROR] DW3000 initialization failed.\n");
        while(1);
    }

    // Set PHY parameters
    phy_init_obj.rf_chan = RF_CHAN;
    phy_init_obj.preamble_code = PREAMBLE_CODE;
    phy_init_obj.preamble_len = PREAMBLE_LEN;
    phy_init_obj.bit_rate = BIT_RATE;
    phy_init_obj.sts_len = STS_LEN;

    // Initialize PHY
    ret = phy_init(&phy_init_obj);
    if (ret != PHY_SUCCESS)
    {
        printk("\n[ERROR] DW3000 configuration failed.\n");
        while(1);
    }

    // Set MAC address
    app_init_obj.mac_addr = MAC_ADDR;

    // Set PAN ID
    app_init_obj.pan_id = PAN_ID;

    // Set STS key (128 bits)
    app_init_obj.sts_key.key0 = STS_KEY_0;
    app_init_obj.sts_key.key1 = STS_KEY_1;
    app_init_obj.sts_key.key2 = STS_KEY_2;
    app_init_obj.sts_key.key3 = STS_KEY_3;

    // Set AES key (128 bits)
    app_init_obj.aes_key.key0 = AES_KEY_0;
    app_init_obj.aes_key.key1 = AES_KEY_1;
    app_init_obj.aes_key.key2 = AES_KEY_2;
    app_init_obj.aes_key.key3 = AES_KEY_3;
    app_init_obj.aes_key.key4 = 0;
    app_init_obj.aes_key.key5 = 0;
    app_init_obj.aes_key.key6 = 0;
    app_init_obj.aes_key.key7 = 0;

    // Initialize app
    ret = app_init(&app_init_obj);
    if (ret != APP_SUCCESS)
    {
        printk("\n[ERROR] App initialization failed.\n");
        while(1);
    }

    // Set number of anchors
    app_ctrl_obj.num_anchors = NUM_NODES;

    // Set nodes MAC addresses
    node_mac_addr[0] = NODE_MAC_ADDR_0;
    node_mac_addr[1] = NODE_MAC_ADDR_1;
    node_mac_addr[2] = NODE_MAC_ADDR_2;
    node_mac_addr[3] = NODE_MAC_ADDR_3;
    node_mac_addr[4] = NODE_MAC_ADDR_4;
    node_mac_addr[5] = NODE_MAC_ADDR_5;
    node_mac_addr[6] = NODE_MAC_ADDR_6;
    node_mac_addr[7] = NODE_MAC_ADDR_7;
    for (int k = 0; k < NUM_NODES; k++)
    {
        app_ctrl_obj.anchor_mac_addr[k] = node_mac_addr[k];
    }

    // Set app runtime parameters
    ret = app_set_ctrl_params(&app_ctrl_obj);
    if (ret != APP_SUCCESS)
    {
        printk("\n[ERROR] App configuration settings are not valid.\n");
        while(1);
    }

    // Begin loop
    while (1)
    {
        #if (MAC_ADDR == PAN_COORDINATOR_MAC_ADDR)
        
        // Wait a bit before proceeding to make sure anchors have receiver switched on
        app_sleep(GUARD_TIME);
        
        // Select which node will operate as tag
        node_id++;
        node_id %= NUM_NODES;
        app_ctrl_obj.tag_mac_addr = node_mac_addr[node_id];
        ret = app_set_ctrl_params(&app_ctrl_obj);
        if (ret != APP_SUCCESS)
        {
            continue;
        }
        
        #endif

        // Run ranging session
        ret = app_run_ieee_802_15_4z_schedule();
        if (ret != APP_SUCCESS)
        {
            continue;
        }

        // Read ranging session results
        app_read_log_info(&app_log_info);
        
        // Compute distances (m)
        for (int k = 0; k < NUM_NODES; k++)
        {
            dist[k] = DWT_TIME_UNITS * LIGHT_SPEED * app_log_info.dist[k];
        }

        // Log superframe ID on console
        printk("\n\n\n\n\n\nRanging session %llu completed.\n\n", app_log_info.superframe_id);

        // Log distances and MAC addresses of tag and anchor nodes
        for (int k = 0; k < NUM_NODES; k++)
        {
            printk("\tDistance from node 0x%02X to node 0x%02X: %.2f m.\n",
                    node_mac_addr[node_id],
                    node_mac_addr[k],
                    (double) dist[k]);
        }
        
        // Sleep till next superframe
        app_sleep(SLEEP_TIME);
    }
}