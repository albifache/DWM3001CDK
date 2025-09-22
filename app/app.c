/*! ----------------------------------------------------------------------------
 * @file        app.c
 * @author      Alberto Facheris
 */


#include "app.h"
#include "app_utils.h"
#include "../mac/mac.h"
#include "../deca_driver/deca_device_api.h"
#include "../port/port.h"


#define SYS_STATUS_RX_ERROR             (SYS_STATUS_RXPHE_BIT_MASK | \
                                        SYS_STATUS_RXFCE_BIT_MASK | \
                                        SYS_STATUS_RXFSL_BIT_MASK | \
                                        SYS_STATUS_RXFTO_BIT_MASK | \
                                        SYS_STATUS_RXPTO_BIT_MASK | \
                                        SYS_STATUS_RXSTO_BIT_MASK)

#define SYS_STATUS_LO_SYS_ERROR         (SYS_STATUS_SPICRCE_BIT_MASK | \
                                        SYS_STATUS_CIAERR_BIT_MASK | \
                                        SYS_STATUS_RXOVRR_BIT_MASK | \
                                        SYS_STATUS_PLL_HILO_BIT_MASK | \
                                        SYS_STATUS_CPERR_BIT_MASK | \
                                        SYS_STATUS_HPDWARN_BIT_MASK)

#define SYS_STATUS_HI_SYS_ERROR         (SYS_STATUS_HI_AES_ERR_BIT_MASK | \
                                        SYS_STATUS_HI_CMD_ERR_BIT_MASK | \
                                        SYS_STATUS_HI_SPI_OVF_BIT_MASK | \
                                        SYS_STATUS_HI_SPI_UNF_BIT_MASK | \
                                        SYS_STATUS_HI_SPIERR_BIT_MASK)
                                      
#define RX_TIMEOUT_DISABLED             0  
#define DEFAULT_RX_TIMEOUT              ((uint64_t) (3.0f * 0.001f / DWT_TIME_UNITS))       // RX timeout (3 ms)
#define SLOT_TIME                       ((uint64_t) (5.0f * 0.001f / DWT_TIME_UNITS))       // Slot duration (5 ms)

#define RX_GUARD_TIME                   ((uint64_t) (1.5f * 0.001f / DWT_TIME_UNITS))       // Guard time to switch on receiver
                                                                                            // before beginning of slot (1.5 ms)
#define DELAY_BEFORE_RX                 (SLOT_TIME - RX_GUARD_TIME)

#define CLOCK_CYCLE                     0x010000000000ull
#define CLOCK_FINE_MASK                 0xFFFFFFFFull
#define CLOCK_COARSE_MASK               0xFFFFFFFE00ull

#define APP_VERSION                     0
#define APP_HEADER_LEN                  8
#define HEADER_LEN                      (MAC_HEADER_LEN + APP_HEADER_LEN)


typedef enum
{
    APP_MSG_INIT = 0,
    APP_MSG_RQST = 1,
    APP_MSG_RESP = 2,  
    APP_MSG_RPT = 3,
    APP_MSG_FINAL = 4
}
app_msg_type_e;


typedef enum
{
    LISTENER = 0,
    TAG = 1,
    ANCHOR = 2
}
app_node_type_e;


typedef enum
{
    APP_STATE_BEGIN = 0,
    APP_STATE_SEND_INIT_MSG = 1,
    APP_STATE_WAIT_INIT_MSG = 2,
    APP_STATE_SEND_RQST_MSG = 3,
    APP_STATE_WAIT_RQST_MSG = 4,
    APP_STATE_SEND_RESP_MSG = 5,
    APP_STATE_WAIT_RESP_MSG = 6,
    APP_STATE_SEND_RPT_MSG = 7,
    APP_STATE_WAIT_RPT_MSG = 8,
    APP_STATE_SEND_FINAL_MSG = 9,
    APP_STATE_WAIT_FINAL_MSG = 10,
    APP_STATE_END = 11
}
app_state_e;


typedef struct
{
    uint16_t version;
    app_msg_type_e msg_type;
    uint32_t superframe_id;
}
app_header_t;


static uint16_t my_pan_id = 0U;
static uint16_t my_mac_addr = 0U;
static uint8_t frame_seq_num = 0;

static uint16_t tag_mac_addr;
static uint16_t anchor_mac_addr[MAX_NUM_ANCHORS];
static uint32_t superframe_id = 0UL;
static app_node_type_e node_type = LISTENER;
static uint8_t num_anchors = MAX_NUM_ANCHORS;
static uint8_t anchor_id = 0;

static uint8_t slot_id = 0;
static app_state_e app_state;

static uint64_t ts_tx_init;
static uint64_t ts_rx_init;
static uint64_t ts_tx_rqst;
static uint64_t ts_rx_rqst;
static uint64_t ts_tx_resp;
static uint64_t ts_rx_resp[MAX_NUM_ANCHORS];
static uint64_t ts_tx_rpt;
static uint64_t ts_rx_rpt;
static uint64_t ts_tx_final;

static uint64_t dist[MAX_NUM_ANCHORS];


static void app_header_write (app_header_t* app_header, uint8_t tx_buffer[]);
static void app_header_read (app_header_t* app_header, uint8_t rx_buffer[]);
static int app_wait_tx_done (void);
static int app_wait_rx_done (void);
static int app_send_init_msg (void);
static int app_wait_init_msg (void);
static int app_send_rqst_msg (void);
static int app_wait_rqst_msg (void);
static int app_send_resp_msg (void);
static int app_wait_resp_msg (void);
static int app_send_report_msg (void);
static int app_wait_report_msg (void);
static int app_send_final_msg (void);
static int app_wait_final_msg (void);


static void app_header_write (app_header_t* app_header, uint8_t tx_buffer[])
{
    // Write app version in TX buffer
    tx_buffer[MAC_HEADER_LEN] = app_header->version & 0xFF;
    tx_buffer[MAC_HEADER_LEN + 1] = (app_header->version >> 8) & 0xFF;

    // Write message type in TX buffer
    tx_buffer[MAC_HEADER_LEN + 2] = app_header->msg_type & 0xFF;
    tx_buffer[MAC_HEADER_LEN + 3] = (app_header->msg_type >> 8) & 0xFF;

    // Write superframe ID in TX buffer
    tx_buffer[MAC_HEADER_LEN + 4] = app_header->superframe_id & 0xFF;
    tx_buffer[MAC_HEADER_LEN + 5] = (app_header->superframe_id >> 8) & 0xFF;
    tx_buffer[MAC_HEADER_LEN + 6] = (app_header->superframe_id >> 16) & 0xFF;
    tx_buffer[MAC_HEADER_LEN + 7] = (app_header->superframe_id >> 24) & 0xFF;

    return;
}


static void app_header_read (app_header_t* app_header, uint8_t rx_buffer[])
{
    // Read app version from RX buffer
    app_header->version = (uint16_t) rx_buffer[MAC_HEADER_LEN];
    app_header->version |= ((uint16_t)rx_buffer[MAC_HEADER_LEN + 1]) << 8;

    // Read message type from RX buffer
    app_header->msg_type = (uint16_t) rx_buffer[MAC_HEADER_LEN + 2];
    app_header->msg_type |= ((uint16_t) rx_buffer[MAC_HEADER_LEN + 3]) << 8;

    // Read superframe ID from RX buffer
    app_header->superframe_id = (uint32_t) rx_buffer[MAC_HEADER_LEN + 4];
    app_header->superframe_id |= ((uint32_t) rx_buffer[MAC_HEADER_LEN + 5]) << 8;
    app_header->superframe_id |= ((uint32_t) rx_buffer[MAC_HEADER_LEN + 6]) << 16;
    app_header->superframe_id |= ((uint32_t) rx_buffer[MAC_HEADER_LEN + 7]) << 24;

    return;
}


static int app_wait_tx_done (void)
{
    volatile uint32_t sysstatuslo = dwt_readsysstatuslo();
    volatile uint32_t sysstatushi = dwt_readsysstatushi();

    while (1)
    {
        sysstatuslo = dwt_readsysstatuslo();
        sysstatushi = dwt_readsysstatushi();
        
        // Check if transmission is successfully completed
        if (sysstatuslo & SYS_STATUS_TXFRS_BIT_MASK)
        {
            dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);

            return APP_SUCCESS;
        }

        // Check if DW3000 system errors occurred during transmission
        else if ((sysstatuslo & SYS_STATUS_LO_SYS_ERROR) || (sysstatushi & SYS_STATUS_HI_SYS_ERROR))
        {
            dwt_forcetrxoff();
            dwt_writesysstatuslo(SYS_STATUS_LO_SYS_ERROR);
            dwt_writesysstatushi(SYS_STATUS_HI_SYS_ERROR);

            return APP_SYS_ERROR;
        }

        // Wait 50 us before next iteration
        deca_usleep(50);
    }
}


static int app_wait_rx_done (void)
{
    volatile uint32_t sysstatuslo;
    volatile uint32_t sysstatushi;

    while (1)
    {
        sysstatuslo = dwt_readsysstatuslo();
        sysstatushi = dwt_readsysstatushi();

        // Check if reception is successfully completed
        if ((sysstatuslo & SYS_STATUS_RXFCG_BIT_MASK) && (sysstatuslo & SYS_STATUS_CIADONE_BIT_MASK))
        {
            dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_CIADONE_BIT_MASK);

            return APP_SUCCESS;
        }

        // Check if PHY errors occurred or timeouts expired during reception
        else if (sysstatuslo & SYS_STATUS_RX_ERROR)
        {
            dwt_writesysstatuslo(SYS_STATUS_RX_ERROR);

            return APP_RX_ERROR;
        }

        // Check if DW3000 system errors occurred during reception
        else if ((sysstatuslo & SYS_STATUS_LO_SYS_ERROR) || (sysstatushi & SYS_STATUS_HI_SYS_ERROR))
        {
            dwt_forcetrxoff();
            dwt_writesysstatuslo(SYS_STATUS_LO_SYS_ERROR);
            dwt_writesysstatushi(SYS_STATUS_HI_SYS_ERROR);

            return APP_SYS_ERROR;
        }
        
        // Wait 50 us before next iteration
        deca_usleep(50); 
    }
}


static int app_send_init_msg (void)
{
    // Write TX frame length and create TX buffer
    uint16_t tx_frame_len = HEADER_LEN + 3 + 2 * num_anchors + FCS_LEN;
    uint8_t tx_buffer[tx_frame_len - FCS_LEN];
    app_write_tx_frame_len(tx_frame_len);

    // Write MAC header
    mac_header_t mac_header;
    mac_header.frame_ctrl = MAC_FRAME_CTRL;
    mac_header.frame_seq_num = frame_seq_num;
    mac_header.pan_id = my_pan_id;
    mac_header.dest_addr = BROADCAST_MAC_ADDR;
    mac_header.src_addr = my_mac_addr;
    mac_header_write(&mac_header, tx_buffer);

    // Write app header
    app_header_t app_header;
    app_header.version = APP_VERSION;
    app_header.msg_type = APP_MSG_INIT;
    superframe_id++;
    app_header.superframe_id = superframe_id;
    app_header_write(&app_header, tx_buffer);

    // Write number of anchors
    tx_buffer[HEADER_LEN] = num_anchors;

    // Write tag MAC address
    tx_buffer[HEADER_LEN + 1] = tag_mac_addr & 0xFF;
    tx_buffer[HEADER_LEN + 2] = (tag_mac_addr >> 8) & 0xFF;

    // Write anchors MAC adresses
    for (int n = 0; n < num_anchors; n++)
    {
        tx_buffer[HEADER_LEN + 3 + 2 * n] = anchor_mac_addr[n] & 0xFF;
        tx_buffer[HEADER_LEN + 4 + 2 * n] = (anchor_mac_addr[n] >> 8) & 0xFF;
    }

    // Check if this node is an anchor
    for (int n = 0; n < num_anchors; n++)
    {
        if (anchor_mac_addr[n] == my_mac_addr)
        {
            node_type = ANCHOR;
            anchor_id = n;
            break;
        }
    }

    // Check if this node is a tag
    if (tag_mac_addr == my_mac_addr)
    {
        node_type = TAG;
    }

    // Write TX buffer
    app_write_tx_buffer(tx_buffer, tx_frame_len);

    // Start transmission (immediate)
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    // Wait until transmission is completed or interrupted
    int ret = app_wait_tx_done();
    if (ret != APP_SUCCESS)
    {
        return ret;
    }

    // Read TX timestamp of INIT message
    ts_tx_init = app_read_tx_timestamp();
    ts_rx_init = ts_tx_init;

    // Update frame sequence number
    frame_seq_num++;

    return APP_SUCCESS;
}


static int app_wait_init_msg (void)
{
    // Disable RX timeout
    app_set_rx_timeout(RX_TIMEOUT_DISABLED);

    // Start receiving (immediate)
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    // Wait until reception is completed or interrupted
    int ret = app_wait_rx_done();
    if (ret != APP_SUCCESS)
    {
        return ret;
    }

    // Read RX timestamp of INIT message
    ts_rx_init = app_read_rx_timestamp();

    // Read RX buffer
    uint16_t rx_frame_len = app_read_rx_frame_len();
    uint8_t rx_buffer[rx_frame_len];
    app_read_rx_buffer(rx_buffer, rx_frame_len);
    
    // Check MAC header
    mac_header_t mac_header;
    mac_header_read(&mac_header, rx_buffer);
    if (mac_header.frame_ctrl != MAC_FRAME_CTRL ||
        mac_header.pan_id != my_pan_id ||
        mac_header.dest_addr != BROADCAST_MAC_ADDR ||
        mac_header.src_addr != PAN_COORDINATOR_MAC_ADDR)
    {
        return MAC_HEADER_WARNING;
    }

    // Check app header
    app_header_t app_header;
    app_header_read(&app_header, rx_buffer);
    if (app_header.version != APP_VERSION ||
        app_header.msg_type != APP_MSG_INIT)
    {
        return APP_HEADER_WARNING;
    }

    // Set superframe ID
    superframe_id = app_header.superframe_id;

    // Read number of anchors
    num_anchors = rx_buffer[HEADER_LEN];

    // Read tag MAC address
    tag_mac_addr = (uint16_t) rx_buffer[HEADER_LEN + 1];
    tag_mac_addr |= ((uint16_t) rx_buffer[HEADER_LEN + 2]) << 8;
    

    // Read anchors MAC addresses
    for (int n = 0; n < num_anchors; n++)
    {
        anchor_mac_addr[n] = (uint16_t) rx_buffer[HEADER_LEN + 3 + 2 * n];
        anchor_mac_addr[n] |= ((uint16_t) rx_buffer[HEADER_LEN + 4 + 2 * n]) << 8;
    }

    // Check if the node has been selected as tag
    if (tag_mac_addr == my_mac_addr)
    {
        node_type = TAG;

        return APP_SUCCESS;
    }

    // Check if the node has been selected as anchor
    for (int n = 0; n < num_anchors; n++)
    {
        if (anchor_mac_addr[n] == my_mac_addr)
        {
            node_type = ANCHOR;
            anchor_id = n;

            return APP_SUCCESS;
        }
    }

    // Set node as LISTENER if it has not been selected as TAG nor as ANCHOR
    node_type = LISTENER;

    return APP_SUCCESS;
}
 

static int app_send_rqst_msg (void)
{
    // Write TX frame length and create TX buffer
    uint16_t tx_frame_len = HEADER_LEN + FCS_LEN;
    uint8_t tx_buffer[tx_frame_len - FCS_LEN];
    app_write_tx_frame_len(tx_frame_len);

    // Write MAC header
    mac_header_t mac_header;
    mac_header.frame_ctrl = MAC_FRAME_CTRL;
    mac_header.frame_seq_num = frame_seq_num;
    mac_header.pan_id = my_pan_id;
    mac_header.dest_addr = BROADCAST_MAC_ADDR;
    mac_header.src_addr = my_mac_addr;
    mac_header_write(&mac_header, tx_buffer);

    // Write app header
    app_header_t app_header;
    app_header.version = APP_VERSION;
    app_header.msg_type = APP_MSG_RQST;
    app_header.superframe_id = superframe_id;
    app_header_write(&app_header, tx_buffer);

    // Write TX buffer
    app_write_tx_buffer(tx_buffer, tx_frame_len);

    // Set TX timestamp of REQUEST message
    ts_tx_rqst = (ts_rx_init + slot_id * SLOT_TIME) & CLOCK_COARSE_MASK;
    app_set_delayed_trx_time(ts_tx_rqst);

    // Start transmission (delayed)
    dwt_starttx(DWT_START_TX_DELAYED);

    // Wait until transmission is completed or interrupted
    int ret = app_wait_tx_done();
    if (ret != APP_SUCCESS)
    {
        return ret;
    }

    // Update frame sequence number
    frame_seq_num++;
    frame_seq_num &= 0xFF;

    return APP_SUCCESS;
}


static int app_wait_rqst_msg (void)
{
    // Enable RX timeout
    app_set_rx_timeout(DEFAULT_RX_TIMEOUT);

    // Set timestamp to switch on the receiver
    app_set_delayed_trx_time(ts_rx_init + (slot_id - 1) * SLOT_TIME + DELAY_BEFORE_RX);

    // Start receiving (delayed)
    dwt_rxenable(DWT_START_RX_DELAYED);

    // Wait until transmission is completed or interrupted
    int ret = app_wait_rx_done();
    if (ret != APP_SUCCESS)
    {
        return ret;
    }

    // Read RX timestamp of RQST message
    ts_rx_rqst = app_read_rx_timestamp();

    // Read RX buffer
    uint16_t rx_frame_len = app_read_rx_frame_len();
    uint8_t rx_buffer[rx_frame_len];
    app_read_rx_buffer(rx_buffer, rx_frame_len);
    
    // Check MAC header
    mac_header_t mac_header;
    mac_header_read(&mac_header, rx_buffer);
    if (mac_header.frame_ctrl != MAC_FRAME_CTRL ||
        mac_header.pan_id != my_pan_id ||
        mac_header.dest_addr != BROADCAST_MAC_ADDR ||
        mac_header.src_addr != tag_mac_addr)
    {
        return MAC_HEADER_WARNING;
    }

    // Check app header
    app_header_t app_header;
    app_header_read(&app_header, rx_buffer);
    if (app_header.version != APP_VERSION ||
        app_header.msg_type != APP_MSG_RQST ||
        app_header.superframe_id != superframe_id)
    {
        return APP_HEADER_WARNING;
    }

    return APP_SUCCESS;
}

      
static int app_send_resp_msg (void)
{
    // Write TX frame length and create TX buffer
    uint16_t tx_frame_len = HEADER_LEN + FCS_LEN;
    uint8_t tx_buffer[tx_frame_len - FCS_LEN];
    app_write_tx_frame_len(tx_frame_len);

    // Write MAC header
    mac_header_t mac_header;
    mac_header.frame_ctrl = MAC_FRAME_CTRL;
    mac_header.frame_seq_num = frame_seq_num;
    mac_header.pan_id = my_pan_id;
    mac_header.dest_addr = tag_mac_addr;
    mac_header.src_addr = my_mac_addr;
    mac_header_write(&mac_header, tx_buffer);

    // Write app header
    app_header_t app_header;
    app_header.version = APP_VERSION;
    app_header.msg_type = APP_MSG_RESP;
    app_header.superframe_id = superframe_id;
    app_header_write(&app_header, tx_buffer);

    // Write TX buffer
    app_write_tx_buffer(tx_buffer, tx_frame_len);

    // Set TX timestamp of RESPONSE message
    ts_tx_resp = (ts_rx_init + slot_id * SLOT_TIME) & CLOCK_COARSE_MASK;
    app_set_delayed_trx_time(ts_tx_resp);

    // Start transmission (delayed)
    dwt_starttx(DWT_START_TX_DELAYED);

    // Wait until transmission is completed or interrupted
    int ret = app_wait_tx_done();
    if (ret != APP_SUCCESS)
    {
        return ret;
    }

    // Update frame sequence number
    frame_seq_num++;
    frame_seq_num &= 0xFF;

    return APP_SUCCESS;
}


static int app_wait_resp_msg (void)
{
    // Enable RX timeout
    app_set_rx_timeout(DEFAULT_RX_TIMEOUT);

    // Set timestamp to switch on the receiver
    app_set_delayed_trx_time(ts_rx_init + (slot_id - 1) * SLOT_TIME + DELAY_BEFORE_RX);

    // Counter for RESPONSE messages
    uint8_t cnt = slot_id - 2;

    // Start receiving (delayed)
    dwt_rxenable(DWT_START_RX_DELAYED);

    // Wait until reception is completed or interrupted
    int ret = app_wait_rx_done();
    if (ret != APP_SUCCESS)
    {
        return ret;
    }

    // Read RX timestamp of RESPONSE message
    ts_rx_resp[slot_id - 2] = app_read_rx_timestamp();

    // Read RX buffer
    uint16_t rx_frame_len = app_read_rx_frame_len();
    uint8_t rx_buffer[rx_frame_len];
    app_read_rx_buffer(rx_buffer, rx_frame_len);

    // Check MAC header
    mac_header_t mac_header;
    mac_header_read(&mac_header, rx_buffer);
    if (mac_header.frame_ctrl != MAC_FRAME_CTRL ||
        mac_header.pan_id != my_pan_id ||
        mac_header.dest_addr != my_mac_addr ||
        mac_header.src_addr != anchor_mac_addr[cnt])
    {
        return MAC_HEADER_WARNING;
    }

    // Check app header
    app_header_t app_header;
    app_header_read(&app_header, rx_buffer);
    if (app_header.version != APP_VERSION ||
        app_header.msg_type != APP_MSG_RESP ||
        app_header.superframe_id != superframe_id)
    {
        return APP_HEADER_WARNING;
    }

    return APP_SUCCESS;
}


static int app_send_report_msg (void)
{
    // Write TX frame length and create TX buffer
    uint16_t tx_frame_len = HEADER_LEN + 8 + 4 * num_anchors + FCS_LEN;
    uint8_t tx_buffer[tx_frame_len - FCS_LEN];
    app_write_tx_frame_len(tx_frame_len);

    // Write MAC header
    mac_header_t mac_header;
    mac_header.frame_ctrl = MAC_FRAME_CTRL;
    mac_header.frame_seq_num = frame_seq_num;
    mac_header.pan_id = my_pan_id;
    mac_header.dest_addr = BROADCAST_MAC_ADDR;
    mac_header.src_addr = my_mac_addr;
    mac_header_write(&mac_header, tx_buffer);

    // Write app header
    app_header_t app_header;
    app_header.version = APP_VERSION;
    app_header.msg_type = APP_MSG_RPT;
    app_header.superframe_id = superframe_id;
    app_header_write(&app_header, tx_buffer);

    // Write TX timestamp of REQUEST message
    tx_buffer[HEADER_LEN] = ts_tx_rqst & 0xFF;
    tx_buffer[HEADER_LEN + 1] = (ts_tx_rqst >> 8) & 0xFF;
    tx_buffer[HEADER_LEN + 2] = (ts_tx_rqst >> 16) & 0xFF;
    tx_buffer[HEADER_LEN + 3] = (ts_tx_rqst >> 24) & 0xFF;
    
    // Write RX timestamp of RESPONSE messages
    for (int n = 0; n < num_anchors; n++)
    {
        tx_buffer[HEADER_LEN + 4 + 4 * n] = ts_rx_resp[n] & 0xFF;
        tx_buffer[HEADER_LEN + 5 + 4 * n] = (ts_rx_resp[n] >> 8) & 0xFF;
        tx_buffer[HEADER_LEN + 6 + 4 * n] = (ts_rx_resp[n] >> 16) & 0xFF;
        tx_buffer[HEADER_LEN + 7 + 4 * n] = (ts_rx_resp[n] >> 24) & 0xFF;
    }

    // Set TX timestamp of REPORT message
    ts_tx_rpt = (ts_rx_init + slot_id * SLOT_TIME) & CLOCK_COARSE_MASK;

    // Write TX timestamp of REPORT message
    tx_buffer[HEADER_LEN + 4 + 4 * num_anchors] = ts_tx_rpt & 0xFF;
    tx_buffer[HEADER_LEN + 5 + 4 * num_anchors] = (ts_tx_rpt >> 8) & 0xFF;
    tx_buffer[HEADER_LEN + 6 + 4 * num_anchors] = (ts_tx_rpt >> 16) & 0xFF;
    tx_buffer[HEADER_LEN + 7 + 4 * num_anchors] = (ts_tx_rpt >> 24) & 0xFF;

    // Write TX buffer
    app_write_tx_buffer(tx_buffer, tx_frame_len);

    // Set TX timestamp
    app_set_delayed_trx_time(ts_tx_rpt);

    // Start transmission (delayed)
    dwt_starttx(DWT_START_TX_DELAYED);

    // Wait until transmission is completed
    int ret = app_wait_tx_done();
    if (ret != APP_SUCCESS)
    {
        return ret;
    }

    // Update frame sequence number
    frame_seq_num++;
    frame_seq_num &= 0xFF;

    return APP_SUCCESS;
}


static int app_wait_report_msg (void)
{
    // Enable RX timeout
    app_set_rx_timeout(DEFAULT_RX_TIMEOUT);

    // Set timestamp to switch on the receiver
    app_set_delayed_trx_time(ts_rx_init + (slot_id - 1) * SLOT_TIME + DELAY_BEFORE_RX);

    // Start receiving (delayed)
    dwt_rxenable(DWT_START_RX_DELAYED);

    // Wait until reception is completed or interrupted
    int ret = app_wait_rx_done();
    if (ret != APP_SUCCESS)
    {
        return ret;
    }

    // Read RX buffer
    uint16_t rx_frame_len = app_read_rx_frame_len();
    uint8_t rx_buffer[rx_frame_len];
    app_read_rx_buffer(rx_buffer, rx_frame_len);

    // Check MAC header
    mac_header_t mac_header;
    mac_header_read(&mac_header, rx_buffer);
    if (mac_header.frame_ctrl != MAC_FRAME_CTRL ||
        mac_header.pan_id != my_pan_id ||
        mac_header.dest_addr != BROADCAST_MAC_ADDR ||
        mac_header.src_addr != tag_mac_addr)
    {
        return MAC_HEADER_WARNING;
    }

    // Check app header
    app_header_t app_header;
    app_header_read(&app_header, rx_buffer);
    if (app_header.version != APP_VERSION ||
        app_header.msg_type != APP_MSG_RPT ||
        app_header.superframe_id != superframe_id)
    {
        return APP_HEADER_WARNING;
    }

    // Read TX timestamp of REQUEST message
    ts_tx_rqst = (uint64_t)rx_buffer[HEADER_LEN];
    ts_tx_rqst |= ((uint64_t)rx_buffer[HEADER_LEN + 1]) << 8;
    ts_tx_rqst |= ((uint64_t)rx_buffer[HEADER_LEN + 2]) << 16;
    ts_tx_rqst |= ((uint64_t)rx_buffer[HEADER_LEN + 3]) << 24;

    // Read RX timestamp of RESPONSE message
    ts_rx_resp[anchor_id] = (uint64_t)rx_buffer[HEADER_LEN + 4 + 4 * anchor_id];
    ts_rx_resp[anchor_id] |= ((uint64_t)rx_buffer[HEADER_LEN + 5 + 4 * anchor_id]) << 8;
    ts_rx_resp[anchor_id] |= ((uint64_t)rx_buffer[HEADER_LEN + 6 + 4 * anchor_id]) << 16;
    ts_rx_resp[anchor_id] |= ((uint64_t)rx_buffer[HEADER_LEN + 7 + 4 * anchor_id]) << 24;

    // Read TX timestamp of REPORT message
    ts_tx_rpt = (uint64_t)rx_buffer[HEADER_LEN + 4 + 4 * num_anchors];
    ts_tx_rpt |= ((uint64_t)rx_buffer[HEADER_LEN + 5 + 4 * num_anchors]) << 8;
    ts_tx_rpt |= ((uint64_t)rx_buffer[HEADER_LEN + 6 + 4 * num_anchors]) << 16;
    ts_tx_rpt |= ((uint64_t)rx_buffer[HEADER_LEN + 7 + 4 * num_anchors]) << 24;

    // Read RX timestamp of REPORT message
    ts_rx_rpt = app_read_rx_timestamp();

    // Compute distance (cm)
    if (ts_rx_resp[anchor_id] == 0ULL)
    {
        dist[anchor_id] = 0UL;
    }
    else
    {
        uint64_t rtt_a = ((ts_rx_resp[anchor_id] | CLOCK_CYCLE) - ts_tx_rqst) & CLOCK_FINE_MASK;
        uint64_t rtt_b = ((ts_rx_rpt | CLOCK_CYCLE) - ts_tx_resp) & CLOCK_FINE_MASK;
        uint64_t wt_a = ((ts_tx_resp | CLOCK_CYCLE) - ts_rx_rqst) & CLOCK_FINE_MASK;
        uint64_t wt_b = ((ts_tx_rpt | CLOCK_CYCLE) - ts_rx_resp[anchor_id]) & CLOCK_FINE_MASK;
        dist[anchor_id] = (rtt_a * rtt_b - wt_a * wt_b) / (rtt_a + rtt_b + wt_a + wt_b);
    }

    return APP_SUCCESS;
}


static int app_send_final_msg (void)
{
    // Write TX frame length and create TX buffer
    uint16_t tx_frame_len = HEADER_LEN + 4 + FCS_LEN;
    uint8_t tx_buffer[tx_frame_len - FCS_LEN];
    app_write_tx_frame_len(tx_frame_len); // Zero offset in TX buffer, ranging enabled

    // Write MAC header
    mac_header_t mac_header;
    mac_header.frame_ctrl = MAC_FRAME_CTRL;
    mac_header.frame_seq_num = frame_seq_num;
    mac_header.pan_id = my_pan_id;
    mac_header.dest_addr = BROADCAST_MAC_ADDR;
    mac_header.src_addr = my_mac_addr;
    mac_header_write(&mac_header, tx_buffer);

    // Write app header
    app_header_t app_header;
    app_header.version = APP_VERSION;
    app_header.msg_type = APP_MSG_FINAL;
    app_header.superframe_id = superframe_id;
    app_header_write(&app_header, tx_buffer);

    // Write distance
    tx_buffer[HEADER_LEN] = dist[anchor_id] & 0xFF;
    tx_buffer[HEADER_LEN + 1] = (dist[anchor_id] >> 8) & 0xFF;
    tx_buffer[HEADER_LEN + 2] = (dist[anchor_id] >> 16) & 0xFF;
    tx_buffer[HEADER_LEN + 3] = (dist[anchor_id] >> 24) & 0xFF;

    // Write TX buffer
    app_write_tx_buffer(tx_buffer, tx_frame_len); // Zero offset in TX buffer

    // Set TX timestamp for FINAL frame
    ts_tx_final = (ts_rx_init + slot_id * SLOT_TIME) & CLOCK_COARSE_MASK;
    app_set_delayed_trx_time(ts_tx_final);

    // Start transmission (delayed)
    dwt_starttx(DWT_START_TX_DELAYED);

    // Wait until transmission is completed or interrupted
    int ret = app_wait_tx_done();
    if (ret != APP_SUCCESS)
    {
        return ret;
    }

    // Update frame sequence number
    frame_seq_num++;
    frame_seq_num &= 0xFF;

    return APP_SUCCESS;
}


static int app_wait_final_msg (void)
{
    // Enable RX timeout
    app_set_rx_timeout(DEFAULT_RX_TIMEOUT);

    // Set timestamp to switch on receiver
    app_set_delayed_trx_time(ts_rx_init + (slot_id - 1) * SLOT_TIME + DELAY_BEFORE_RX);

    // Counter for FINAL messages
    uint8_t cnt = slot_id - 3 - num_anchors;

    // Start receiving (delayed)
    dwt_rxenable(DWT_START_RX_DELAYED);

    // Wait until reception is completed or interrupted
    int ret = app_wait_rx_done();
    if (ret != APP_SUCCESS)
    {
        return ret;
    }

    // Read RX buffer
    uint16_t rx_frame_len = app_read_rx_frame_len();
    uint8_t rx_buffer[rx_frame_len];
    app_read_rx_buffer(rx_buffer, rx_frame_len);

    // Check MAC header
    mac_header_t mac_header;
    mac_header_read(&mac_header, rx_buffer);
    if (mac_header.frame_ctrl != MAC_FRAME_CTRL ||
        mac_header.pan_id != my_pan_id ||
        mac_header.dest_addr != BROADCAST_MAC_ADDR ||
        mac_header.src_addr != anchor_mac_addr[cnt])
    {
        return MAC_HEADER_WARNING;
    }

    // Check app header
    app_header_t app_header;
    app_header_read(&app_header, rx_buffer);
    if (app_header.version != APP_VERSION ||
        app_header.msg_type != APP_MSG_FINAL ||
        app_header.superframe_id != superframe_id)
    {
        return APP_HEADER_WARNING;
    }

    // Read distance (cm)
    dist[cnt] = (uint64_t)rx_buffer[HEADER_LEN];
    dist[cnt] |= ((uint64_t)rx_buffer[HEADER_LEN + 1]) << 8;
    dist[cnt] |= ((uint64_t)rx_buffer[HEADER_LEN + 2]) << 16;
    dist[cnt] |= ((uint64_t)rx_buffer[HEADER_LEN + 3]) << 24;

    return APP_SUCCESS;
}


int app_run_ieee_802_15_4_schedule (void)
{
    int ret = 0;

    // Reset scheduler state and slot ID
    app_state = APP_STATE_BEGIN;
    slot_id = 0;

    // Zeroing all timestamps
    ts_tx_init = 0ULL;
    ts_rx_init = 0ULL;
    ts_tx_rqst = 0ULL;
    ts_rx_rqst = 0ULL;
    ts_tx_resp = 0ULL;
    for (int k = 0; k < MAX_NUM_ANCHORS; k++)
    {
        ts_rx_resp[k] = 0ULL;
    }
    ts_tx_rpt = 0ULL;
    ts_rx_rpt= 0ULL;
    ts_tx_final = 0ULL;

    // Zeroing all distances
    for (int k = 0; k < MAX_NUM_ANCHORS; k++)
    {
        dist[k] = 0ULL;
    }

    // Start loop
    while (1)
    {
        switch (app_state)
        {
            case APP_STATE_BEGIN:

                // Prepare to send INIT message if the node is the PAN coordinator
                if (my_mac_addr == PAN_COORDINATOR_MAC_ADDR)
                {
                    app_state = APP_STATE_SEND_INIT_MSG;
                }

                // Prepare to receive INIT message if the node is not the PAN coordinator
                else
                {
                    app_state = APP_STATE_WAIT_INIT_MSG;
                }

            break;

            case APP_STATE_SEND_INIT_MSG:

                // Send INIT message
                ret = app_send_init_msg();

                // Retry if transmission errors occurred
                if (ret != APP_SUCCESS)
                {
                    slot_id = 0;
                    app_state =  APP_STATE_SEND_INIT_MSG;
                }

                // Check if transmission was successful
                else
                {
                    switch (node_type)
                    {
                        case LISTENER:

                            slot_id = 3 + num_anchors;
                            app_state = APP_STATE_WAIT_FINAL_MSG;

                        break;
                        
                        case TAG:

                            slot_id = 1;
                            app_state = APP_STATE_SEND_RQST_MSG;

                        break;

                        case ANCHOR:

                            slot_id = 1;
                            app_state = APP_STATE_WAIT_RQST_MSG;

                        break;
                    }
                }

            break;
            
            case APP_STATE_WAIT_INIT_MSG:

                ret = app_wait_init_msg();

                // Retry if reception errors occurred
                if (ret != APP_SUCCESS)
                {
                    slot_id = 0;
                    app_state =  APP_STATE_WAIT_INIT_MSG;
                }

                // Check if transmission was successful
                else
                {
                    switch (node_type)
                    {
                        case LISTENER:

                            slot_id = 3 + num_anchors;
                            app_state = APP_STATE_WAIT_FINAL_MSG;

                        break;
                        
                        case TAG:

                            slot_id = 1;
                            app_state = APP_STATE_SEND_RQST_MSG;

                        break;

                        case ANCHOR:

                            slot_id = 1;
                            app_state = APP_STATE_WAIT_RQST_MSG;

                        break;
                    }
                }

            break;

            case APP_STATE_SEND_RQST_MSG:

                ret = app_send_rqst_msg();

                // Exit if REQUEST message was not sent successfully
                if (ret != APP_SUCCESS)
                {
                    slot_id = 3 + num_anchors;
                    app_state =  APP_STATE_WAIT_FINAL_MSG;
                }

                // Proceed if REQUEST message is successfully sent
                else
                {
                    slot_id = 2;
                    app_state = APP_STATE_WAIT_RESP_MSG;
                }

            break;

            case APP_STATE_WAIT_RQST_MSG:
                
                ret = app_wait_rqst_msg();
                
                // Exit if REQUEST message was not received
                if (ret != APP_SUCCESS)
                {
                    slot_id = 3 + num_anchors;
                    app_state = APP_STATE_WAIT_FINAL_MSG;   
                }

                // Proceed if REQUEST message is received
                else
                {
                    slot_id = 2 + anchor_id;                    
                    app_state = APP_STATE_SEND_RESP_MSG;
                }

            break;

            case APP_STATE_SEND_RESP_MSG:

                ret = app_send_resp_msg();

                // Exit if RESPONSE message was not sent successfully
                if (ret != APP_SUCCESS)
                {
                    slot_id = 3 + num_anchors;
                    app_state = APP_STATE_WAIT_FINAL_MSG;
                }

                // Proceed if RESPONSE message was sent successfully
                else
                {
                    slot_id = 2 + num_anchors;
                    app_state = APP_STATE_WAIT_RPT_MSG;
                }

            break;

            case APP_STATE_WAIT_RESP_MSG:

                ret = app_wait_resp_msg();

                // Update slot ID
                slot_id++;

                // Check if all the anchors have replied
                if (slot_id < 2 + num_anchors)
                {
                    app_state = APP_STATE_WAIT_RESP_MSG;
                }
                
                // Proceed if all the anchors have replied
                else if (slot_id == 2 + num_anchors)
                {
                    app_state = APP_STATE_SEND_RPT_MSG;
                }

            break;

            case APP_STATE_SEND_RPT_MSG:

                ret = app_send_report_msg();

                // Go to FINAL message
                slot_id = 3 + num_anchors;
                app_state = APP_STATE_WAIT_FINAL_MSG;

            break;

            case APP_STATE_WAIT_RPT_MSG:

                ret = app_wait_report_msg();

                // Go to FINAL message
                slot_id = 3 + num_anchors;
                app_state = APP_STATE_WAIT_FINAL_MSG;

            break;

            case APP_STATE_SEND_FINAL_MSG:

                // Check if the node must send the FINAL message
                if (node_type == ANCHOR && slot_id == 3 + num_anchors + anchor_id)
                {
                    ret = app_send_final_msg();
                    slot_id++;
                    app_state =  APP_STATE_WAIT_FINAL_MSG;
                }

                // Listen upcoming FINAL messages otherwise
                else
                {
                    app_state =  APP_STATE_WAIT_FINAL_MSG;
                }

            break;

            case APP_STATE_WAIT_FINAL_MSG:

                // Check if all FINAL messages have been sent
                if (slot_id == 3 + 2 * num_anchors)
                {
                    ret = APP_SUCCESS;
                    app_state = APP_STATE_END;
                }

                // Check if the node must send the FINAL message
                else if (node_type == ANCHOR && slot_id == 3 + num_anchors + anchor_id)
                {
                    app_state =  APP_STATE_SEND_FINAL_MSG;
                }

                // Listen upcoming FINAL message
                else
                {
                    ret = app_wait_final_msg();
                    slot_id++;
                }

            break;

            case APP_STATE_END:
                
                return ret;

            break;

            default:

                return APP_RUN_ERROR;

            break;
        }
    }
}


int app_set_mac_addr (uint16_t mac_addr)
{
    // Check if MAC address is valid
    if (mac_addr == BROADCAST_MAC_ADDR)
    {
        return APP_CONFIG_ERROR;
    }

    // Set MAC address
    my_mac_addr = mac_addr;

    return APP_SUCCESS;
}


uint16_t app_get_mac_addr (void)
{
    return my_mac_addr;
}


int app_set_pan_id (uint16_t pan_id)
{
    // Check if PAN ID is valid
    if (pan_id == BROADCAST_PAN_ID)
    {
        return APP_CONFIG_ERROR;
    }
    
    // Set PAN ID
    my_pan_id = pan_id;
    
    return APP_SUCCESS;
}


uint16_t app_get_pan_id (void)
{
    return my_pan_id;
}


int app_set_tag_mac_addr (uint16_t mac_addr)
{
    // Check if MAC address is valid
    if (mac_addr == BROADCAST_MAC_ADDR)
    {
        return APP_CONFIG_ERROR;
    }

    // Set tag MAC address
    tag_mac_addr = mac_addr;

    return APP_SUCCESS;
}


int app_set_anchor_mac_addr (uint16_t mac_addr[], uint8_t cnt)
{
    // Check if number of anchors exceeds the maximum limit
    if (cnt == 0 || cnt > MAX_NUM_ANCHORS)
    {
        return APP_CONFIG_ERROR;
    }

    // Check if all the MAC addresses are valid
    for (int k = 0; k < cnt; k++)
    {
        if (mac_addr[k] == BROADCAST_MAC_ADDR)
        {
            return APP_CONFIG_ERROR;
        }
    }

    // Set number of anchors
    num_anchors = cnt;

    // Set anchors MAC addresses
    for (int k = 0; k < num_anchors; k++)
    {
        anchor_mac_addr[k] = mac_addr[k];
    }

    return APP_SUCCESS;
}


void app_get_ranging_info (app_ranging_info_t *ranging_info)
{
    // Write superframe ID
    ranging_info->superframe_id = superframe_id;

    // Write timestamp of INIT message
    ranging_info->timestamp = ts_rx_init;

    // Write distances
    for (int k = 0; k < MAX_NUM_ANCHORS; k++)
    {
        ranging_info->dist[k] = dist[k];
    }

    // Write number of anchors
    ranging_info->num_anchors = num_anchors;

    // Write tag MAC address
    ranging_info->tag_mac_addr = tag_mac_addr;

    // Write anchors MAC addresses
    for (int k = 0; k < MAX_NUM_ANCHORS; k++)
    {
        ranging_info->anchor_mac_addr[k] = anchor_mac_addr[k];
    }

    return;
}


void app_sleep (uint16_t time_ms)
{
    deca_sleep(time_ms);

    return;
}