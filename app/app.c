/*! ----------------------------------------------------------------------------
 * @file        app.c
 * @author      Alberto Facheris
 */


#include "app.h"
#include "app_utils.h"
#include "../mac/mac.h"
#include "../deca_driver/deca_device_api.h"
#include "../port/port.h"
//#include <zephyr/sys/printk.h>


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
#define DEFAULT_RX_TIMEOUT              ((uint64_t) (3.0f * 0.001f / DWT_TIME_UNITS))       // RX timeout (ms)
#define SLOT_TIME                       ((uint64_t) (6.0f * 0.001f / DWT_TIME_UNITS))       // Slot duration (ms)

#define RX_GUARD_TIME                   ((uint64_t) (1.25f * 0.001f / DWT_TIME_UNITS))      // Guard time (ms) to switch on RX
                                                                                            // before beginning of slot
#define DELAY_BEFORE_RX                 (SLOT_TIME - RX_GUARD_TIME)
#define TRX_POLL_TIME                   50                                                  // Polling time (us) when waiting
                                                                                            // for TRX operation completion

#define CLOCK_CYCLE                     0x010000000000ull
#define CLOCK_FINE_MASK                 0xFFFFFFFFull
#define CLOCK_COARSE_MASK               0xFFFFFFFE00ull

#define APP_VERSION                     1

#define APP_HEADER_LEN                  13
#define HEADER_LEN                      (MAC_HEADER_LEN + APP_HEADER_LEN)
#define MIC_LEN                         16

#define NULL_RX_BUFFER_OFFSET           0
#define NULL_TX_BUFFER_OFFSET           0
#define RANGING_BIT_ENABLED             1


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
node_type_e;


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
    uint64_t superframe_id;
    uint8_t slot_id;
}
app_header_t;


// Initialization parameters
static uint16_t mac_addr;
static uint16_t pan_id;
static dwt_sts_cp_key_t sts_key;
static dwt_aes_key_t aes_key;

// Public logic control variables (user defined)
static uint16_t tag_mac_addr;
static uint8_t num_anchors = MAX_NUM_ANCHORS;
static uint16_t anchor_mac_addr[MAX_NUM_ANCHORS];

// Internal logic control variables
static node_type_e node_type = LISTENER;
static app_state_e app_state;
static uint8_t frame_seq_num = 0;
static uint64_t superframe_id = 0;
static uint8_t slot_id = 0;
static uint8_t anchor_id = 0;

// Timestamps
static uint64_t ts_rx_init;
static uint64_t ts_tx_rqst;
static uint64_t ts_rx_rqst;
static uint64_t ts_tx_resp;
static uint64_t ts_rx_resp[MAX_NUM_ANCHORS];
static uint64_t ts_tx_rpt;
static uint64_t ts_rx_rpt;
static uint64_t ts_tx_final;

// Distances
static uint64_t dist[MAX_NUM_ANCHORS];


// Private functions
static int app_header_write (app_header_t* app_header, uint8_t tx_buf[], uint16_t tx_buf_len);
static int app_header_read (app_header_t* app_header, uint8_t rx_buf[], uint16_t rx_buf_len);
static int app_wait_tx_done (void);
static int app_wait_rx_done (void);
static int app_aes_encrypt (uint8_t tx_buffer[], uint16_t tx_frame_len);
static int app_aes_decrypt (uint8_t rx_buffer[], uint16_t rx_frame_len);
static void app_sts_generate (void);
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


static int app_header_write (app_header_t* app_header, uint8_t tx_buf[], uint16_t tx_buf_len)
{
    // Validate input
    if (app_header == NULL ||
        tx_buf == NULL ||
        tx_buf_len < APP_HEADER_LEN)
    {
        return APP_RUN_ERROR;
    }

    // Write app version in TX buffer
    tx_buf[MAC_HEADER_LEN] = app_header->version & 0xFF;
    tx_buf[MAC_HEADER_LEN + 1] = (app_header->version >> 8) & 0xFF;

    // Write message type in TX buffer
    tx_buf[MAC_HEADER_LEN + 2] = app_header->msg_type & 0xFF;
    tx_buf[MAC_HEADER_LEN + 3] = (app_header->msg_type >> 8) & 0xFF;

    // Write superframe ID in TX buffer
    tx_buf[MAC_HEADER_LEN + 4] = app_header->superframe_id & 0xFF;
    tx_buf[MAC_HEADER_LEN + 5] = (app_header->superframe_id >> 8) & 0xFF;
    tx_buf[MAC_HEADER_LEN + 6] = (app_header->superframe_id >> 16) & 0xFF;
    tx_buf[MAC_HEADER_LEN + 7] = (app_header->superframe_id >> 24) & 0xFF;
    tx_buf[MAC_HEADER_LEN + 8] = (app_header->superframe_id >> 32) & 0xFF;
    tx_buf[MAC_HEADER_LEN + 9] = (app_header->superframe_id >> 40) & 0xFF;
    tx_buf[MAC_HEADER_LEN + 10] = (app_header->superframe_id >> 48) & 0xFF;
    tx_buf[MAC_HEADER_LEN + 11] = (app_header->superframe_id >> 56) & 0xFF;

    // Write slot ID in TX buffer
    tx_buf[MAC_HEADER_LEN + 12] = app_header->slot_id;

    return APP_SUCCESS;
}


static int app_header_read (app_header_t* app_header, uint8_t rx_buf[], uint16_t rx_buf_len)
{
    // Validate input
    if (app_header == NULL ||
        rx_buf == NULL ||
        rx_buf_len < APP_HEADER_LEN)
    {
        return APP_RUN_ERROR;
    }

    // Read app version from RX buffer
    app_header->version = (uint16_t) rx_buf[MAC_HEADER_LEN];
    app_header->version |= ((uint16_t)rx_buf[MAC_HEADER_LEN + 1]) << 8;

    // Read message type from RX buffer
    app_header->msg_type = (uint16_t) rx_buf[MAC_HEADER_LEN + 2];
    app_header->msg_type |= ((uint16_t) rx_buf[MAC_HEADER_LEN + 3]) << 8;

    // Read superframe ID from RX buffer
    app_header->superframe_id = (uint64_t) rx_buf[MAC_HEADER_LEN + 4];
    app_header->superframe_id |= ((uint64_t) rx_buf[MAC_HEADER_LEN + 5]) << 8;
    app_header->superframe_id |= ((uint64_t) rx_buf[MAC_HEADER_LEN + 6]) << 16;
    app_header->superframe_id |= ((uint64_t) rx_buf[MAC_HEADER_LEN + 7]) << 24;
    app_header->superframe_id |= ((uint64_t) rx_buf[MAC_HEADER_LEN + 8]) << 32;
    app_header->superframe_id |= ((uint64_t) rx_buf[MAC_HEADER_LEN + 9]) << 40;
    app_header->superframe_id |= ((uint64_t) rx_buf[MAC_HEADER_LEN + 10]) << 48;
    app_header->superframe_id |= ((uint64_t) rx_buf[MAC_HEADER_LEN + 11]) << 56;

    // Read slot ID from RX buffer
    app_header->slot_id = rx_buf[MAC_HEADER_LEN + 12];

    return APP_SUCCESS;
}


static int app_aes_encrypt (uint8_t tx_buffer[], uint16_t tx_frame_len)
{
    // Configure AES engine
    dwt_aes_config_t aes_config;
    aes_config.key_load = AES_KEY_Load;
    aes_config.key_size = AES_KEY_128bit;
    aes_config.key_src = AES_KEY_Src_Register;
    aes_config.mic = MIC_16;
    aes_config.mode = AES_Encrypt;
    aes_config.aes_core_type = AES_core_type_GCM;
    aes_config.aes_key_otp_type = AES_key_RAM;
    aes_config.key_addr = 0;
    dwt_configure_aes(&aes_config);

    // Generate nonce
    uint8_t nonce[12];
    nonce[0] = slot_id;
    nonce[1] = 0;
    nonce[2] = 0;
    nonce[3] = 0;
    nonce[4] = superframe_id & 0xFF;
    nonce[5] = (superframe_id >> 8) & 0xFF;
    nonce[6] = (superframe_id >> 16) & 0xFF;
    nonce[7] = (superframe_id >> 24) & 0xFF;
    nonce[8] = (superframe_id >> 32) & 0xFF;
    nonce[9] = (superframe_id >> 40) & 0xFF;
    nonce[10] = (superframe_id >> 48) & 0xFF;
    nonce[11] = (superframe_id >> 56) & 0xFF;

    // Configure AES job
    dwt_aes_job_t aes_job;
    aes_job.nonce = nonce;               
    aes_job.header = &tx_buffer[0];
    aes_job.header_len = HEADER_LEN;
    aes_job.payload = &tx_buffer[HEADER_LEN];
    aes_job.payload_len = tx_frame_len - HEADER_LEN - FCS_LEN - MIC_LEN;
    aes_job.src_port = AES_Src_Tx_buf;
    aes_job.dst_port = AES_Dst_Tx_buf;
    aes_job.mode = AES_Encrypt;
    aes_job.mic_size = MIC_LEN;

    // Encrypt data
    int8_t aes_status = dwt_do_aes(&aes_job, aes_config.aes_core_type);
    if ((aes_status < 0) || (aes_status & DWT_AES_ERRORS))
    {
        return APP_RUN_ERROR;
    }
    
    return APP_SUCCESS;
}


static int app_aes_decrypt (uint8_t rx_buffer[], uint16_t rx_frame_len)
{
    // Configure AES engine
    dwt_aes_config_t aes_config;
    aes_config.key_load = AES_KEY_Load;
    aes_config.key_size = AES_KEY_128bit;
    aes_config.key_src = AES_KEY_Src_Register;
    aes_config.mic = MIC_16;
    aes_config.mode = AES_Decrypt;
    aes_config.aes_core_type = AES_core_type_GCM;
    aes_config.aes_key_otp_type = AES_key_RAM;
    aes_config.key_addr = 0;
    dwt_configure_aes(&aes_config);

    // Generate nonce
    uint8_t nonce[12];
    nonce[0] = slot_id;
    nonce[1] = 0;
    nonce[2] = 0;
    nonce[3] = 0;
    nonce[4] = superframe_id & 0xFF;
    nonce[5] = (superframe_id >> 8) & 0xFF;
    nonce[6] = (superframe_id >> 16) & 0xFF;
    nonce[7] = (superframe_id >> 24) & 0xFF;
    nonce[8] = (superframe_id >> 32) & 0xFF;
    nonce[9] = (superframe_id >> 40) & 0xFF;
    nonce[10] = (superframe_id >> 48) & 0xFF;
    nonce[11] = (superframe_id >> 56) & 0xFF;

    // Configure AES job
    dwt_aes_job_t aes_job;
    aes_job.nonce = nonce;               
    aes_job.header = &rx_buffer[0];
    aes_job.header_len = HEADER_LEN;
    aes_job.payload = &rx_buffer[HEADER_LEN];
    aes_job.payload_len = rx_frame_len - HEADER_LEN - FCS_LEN - MIC_LEN;
    aes_job.src_port = AES_Src_Rx_buf_0;
    aes_job.dst_port = AES_Dst_Rx_buf_0;
    aes_job.mode = AES_Decrypt;
    aes_job.mic_size = MIC_LEN;

    // Decrypt data
    int8_t aes_status = dwt_do_aes(&aes_job, aes_config.aes_core_type);
    if ((aes_status < 0) || (aes_status & DWT_AES_ERRORS))
    {
        return APP_RUN_ERROR;
    }
    
    return APP_SUCCESS;
}


static void app_sts_generate (void)
{
    dwt_sts_cp_iv_t sts_iv;

    // Set default STS init vector for first message of each superframe
    if (slot_id == 0)
    {
        sts_iv.iv0 = 0;
        sts_iv.iv1 = 0;
        sts_iv.iv2 = 0;
        sts_iv.iv3 = 0;
    }

    // Set unique STS init vector otherwise
    else
    {
        sts_iv.iv0 = slot_id;
        sts_iv.iv1 = superframe_id;
        sts_iv.iv2 = superframe_id >> 32;
        sts_iv.iv3 = 0;
    }

    // Write STS init vector
    dwt_configurestsiv(&sts_iv);
    dwt_configurestsloadiv();
}


static int app_wait_tx_done (void)
{
    volatile uint32_t sys_status_lo;
    volatile uint32_t sys_status_hi;

    while (1)
    {
        sys_status_lo = dwt_readsysstatuslo();
        sys_status_hi = dwt_readsysstatushi();
        
        // Check if transmission is successfully completed
        if (sys_status_lo & SYS_STATUS_TXFRS_BIT_MASK)
        {
            dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
            return APP_SUCCESS;
        }

        // Check if DW3000 system errors occurred during transmission
        else if ((sys_status_lo & SYS_STATUS_LO_SYS_ERROR) || (sys_status_hi & SYS_STATUS_HI_SYS_ERROR))
        {
            dwt_forcetrxoff();
            dwt_writesysstatuslo(SYS_STATUS_LO_SYS_ERROR);
            dwt_writesysstatushi(SYS_STATUS_HI_SYS_ERROR);
            return APP_RUN_ERROR;
        }

        // Wait 50 us before next iteration
        deca_usleep(TRX_POLL_TIME);
    }
}


static int app_wait_rx_done (void)
{
    volatile uint32_t sys_status_lo;
    volatile uint32_t sys_status_hi;

    volatile int32_t sts_qual;
    volatile int32_t sts_status;
    int16_t sts_qual_idx;
    uint16_t sts_status_val;

    while (1)
    {
        sys_status_lo = dwt_readsysstatuslo();
        sys_status_hi = dwt_readsysstatushi();

        // Check if reception is successfully completed
        if ((sys_status_lo & SYS_STATUS_RXFCG_BIT_MASK) && (sys_status_lo & SYS_STATUS_CIADONE_BIT_MASK))
        {
            dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_CIADONE_BIT_MASK);

            // Read STS quality and status
            sts_qual = dwt_readstsquality(&sts_qual_idx, 0);
            sts_status = dwt_readstsstatus(&sts_status_val, 0);

            // Check if STS quality is good enough
            if (sts_qual < 0 || sts_status < 0)
            {
                return APP_RUN_ERROR;
            }
            else
            {
                return APP_SUCCESS;
            }
        }

        // Check if PHY errors occurred or timeouts expired during reception
        else if (sys_status_lo & SYS_STATUS_RX_ERROR)
        {
            dwt_writesysstatuslo(SYS_STATUS_RX_ERROR);

            return APP_RUN_ERROR;
        }

        // Check if DW3000 system errors occurred during reception
        else if ((sys_status_lo & SYS_STATUS_LO_SYS_ERROR) || (sys_status_hi & SYS_STATUS_HI_SYS_ERROR))
        {
            dwt_forcetrxoff();
            dwt_writesysstatuslo(SYS_STATUS_LO_SYS_ERROR);
            dwt_writesysstatushi(SYS_STATUS_HI_SYS_ERROR);

            return APP_RUN_ERROR;
        }
        
        // Wait 50 us before next iteration
        deca_usleep(TRX_POLL_TIME); 
    }
}


static int app_send_init_msg (void)
{
    int ret;

    // Set TX frame length
    uint16_t tx_frame_len = HEADER_LEN + 3 + 2 * num_anchors + MIC_LEN + FCS_LEN;
    dwt_writetxfctrl(tx_frame_len, NULL_TX_BUFFER_OFFSET, RANGING_BIT_ENABLED);

    // Create TX buffer
    uint8_t tx_buffer[tx_frame_len - FCS_LEN];

    // Write MAC header
    mac_header_t mac_header;
    mac_header.frame_ctrl = MAC_FRAME_CTRL;
    mac_header.frame_seq_num = frame_seq_num;
    mac_header.pan_id = pan_id;
    mac_header.dest_addr = BROADCAST_MAC_ADDR;
    mac_header.src_addr = mac_addr;
    ret = mac_header_write(&mac_header, tx_buffer, sizeof(tx_buffer));
    if (ret != MAC_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Update superframe ID
    superframe_id++;

    // Write app header
    app_header_t app_header;
    app_header.version = APP_VERSION;
    app_header.msg_type = APP_MSG_INIT;
    app_header.superframe_id = superframe_id;
    app_header.slot_id = slot_id;
    ret = app_header_write(&app_header, tx_buffer, sizeof(tx_buffer));
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Write number of anchors
    tx_buffer[HEADER_LEN] = num_anchors;

    // Write tag MAC address
    tx_buffer[HEADER_LEN + 1] = tag_mac_addr & 0xFF;
    tx_buffer[HEADER_LEN + 2] = (tag_mac_addr >> 8) & 0xFF;

    // Write anchors MAC adresses
    for (int k = 0; k < num_anchors; k++)
    {
        tx_buffer[HEADER_LEN + 3 + 2 * k] = anchor_mac_addr[k] & 0xFF;
        tx_buffer[HEADER_LEN + 4 + 2 * k] = (anchor_mac_addr[k] >> 8) & 0xFF;
    }

    // Check if this node is an anchor
    for (int k = 0; k < num_anchors; k++)
    {
        if (anchor_mac_addr[k] == mac_addr)
        {
            node_type = ANCHOR;
            anchor_id = k;
            break;
        }
    }

    // Check if this node is a tag
    if (tag_mac_addr == mac_addr)
    {
        node_type = TAG;
    }

    // Generate STS
    app_sts_generate();

    // Encrypt TX data
    ret = app_aes_encrypt(tx_buffer, tx_frame_len);
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Start transmission (immediate)
    ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
    if (ret != DWT_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Wait until transmission is completed or interrupted
    ret = app_wait_tx_done();
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Read TX timestamp of INIT message (TX stamp and RX stamp coincide in this case)
    ts_rx_init = app_read_tx_timestamp();

    // Update frame sequence number
    frame_seq_num++;

    return APP_SUCCESS;
}


static int app_wait_init_msg (void)
{
    int ret;

    // Disable RX timeout
    app_set_rx_timeout(RX_TIMEOUT_DISABLED);

    // Generate STS
    app_sts_generate();

    // Start receiving (immediate)
    ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);
    if (ret != DWT_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Wait until reception is completed or interrupted
    ret = app_wait_rx_done();
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Read RX timestamp of INIT message
    ts_rx_init = app_read_rx_timestamp();

    // Read RX frame length
    uint16_t rx_frame_len = dwt_getframelength(NULL);
    if (rx_frame_len < HEADER_LEN)
    {
        return APP_RUN_ERROR;
    }

    // Read RX buffer
    uint8_t rx_buffer[rx_frame_len];
    dwt_readrxdata(rx_buffer, HEADER_LEN, NULL_RX_BUFFER_OFFSET);
    
    // Read MAC header
    mac_header_t mac_header;
    ret = mac_header_read(&mac_header, rx_buffer, sizeof(rx_buffer));
    if (ret != MAC_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Check MAC header
    if (mac_header.frame_ctrl != MAC_FRAME_CTRL ||
        mac_header.pan_id != pan_id ||
        mac_header.dest_addr != BROADCAST_MAC_ADDR ||
        mac_header.src_addr != PAN_COORDINATOR_MAC_ADDR)
    {
        return APP_RUN_WARNING;
    }

    // Read app header
    app_header_t app_header;
    ret = app_header_read(&app_header, rx_buffer, sizeof(rx_buffer));
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Check app header
    if (app_header.version != APP_VERSION ||
        app_header.msg_type != APP_MSG_INIT ||
        app_header.slot_id != slot_id)
    {
        return APP_RUN_WARNING;
    }

    // Set superframe ID
    superframe_id = app_header.superframe_id;

    // Decrypt payload
    ret = app_aes_decrypt(rx_buffer, rx_frame_len);
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Read number of anchors
    num_anchors = rx_buffer[HEADER_LEN];

    // Read tag MAC address
    tag_mac_addr = (uint16_t) rx_buffer[HEADER_LEN + 1];
    tag_mac_addr |= ((uint16_t) rx_buffer[HEADER_LEN + 2]) << 8;
    
    // Read anchors MAC addresses
    for (int k = 0; k < num_anchors; k++)
    {
        anchor_mac_addr[k] = (uint16_t) rx_buffer[HEADER_LEN + 3 + 2 * k];
        anchor_mac_addr[k] |= ((uint16_t) rx_buffer[HEADER_LEN + 4 + 2 * k]) << 8;
    }

    // Check if the node has been selected as tag
    if (tag_mac_addr == mac_addr)
    {
        node_type = TAG;

        return APP_SUCCESS;
    }

    // Check if the node has been selected as anchor
    for (int k = 0; k < num_anchors; k++)
    {
        if (anchor_mac_addr[k] == mac_addr)
        {
            node_type = ANCHOR;
            anchor_id = k;

            return APP_SUCCESS;
        }
    }

    // Set node as LISTENER if it has not been selected as TAG nor as ANCHOR
    node_type = LISTENER;

    return APP_SUCCESS;
}
 

static int app_send_rqst_msg (void)
{
    int ret;

    // Set TX frame length
    uint16_t tx_frame_len = HEADER_LEN + MIC_LEN + FCS_LEN;
    dwt_writetxfctrl(tx_frame_len, NULL_TX_BUFFER_OFFSET, RANGING_BIT_ENABLED);

    // Create TX buffer
    uint8_t tx_buffer[tx_frame_len - FCS_LEN];

    // Write MAC header
    mac_header_t mac_header;
    mac_header.frame_ctrl = MAC_FRAME_CTRL;
    mac_header.frame_seq_num = frame_seq_num;
    mac_header.pan_id = pan_id;
    mac_header.dest_addr = BROADCAST_MAC_ADDR;
    mac_header.src_addr = mac_addr;
    ret = mac_header_write(&mac_header, tx_buffer, sizeof(tx_buffer));
    if (ret != MAC_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Write app header
    app_header_t app_header;
    app_header.version = APP_VERSION;
    app_header.msg_type = APP_MSG_RQST;
    app_header.superframe_id = superframe_id;
    app_header.slot_id = slot_id;
    ret = app_header_write(&app_header, tx_buffer, sizeof(tx_buffer));
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Set TX timestamp of REQUEST message
    ts_tx_rqst = (ts_rx_init + slot_id * SLOT_TIME) & CLOCK_COARSE_MASK;
    app_set_delayed_trx_time(ts_tx_rqst);

    // Generate STS
    app_sts_generate();

    // Encrypt TX data
    ret = app_aes_encrypt(tx_buffer, tx_frame_len);
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;;
    }

    // Start transmission (delayed)
    ret = dwt_starttx(DWT_START_TX_DELAYED);
    if (ret != DWT_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Wait until transmission is completed or interrupted
    ret = app_wait_tx_done();
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Update frame sequence number
    frame_seq_num++;
    frame_seq_num &= 0xFF;

    return APP_SUCCESS;
}


static int app_wait_rqst_msg (void)
{
    int ret;

    // Enable RX timeout
    app_set_rx_timeout(DEFAULT_RX_TIMEOUT);

    // Set timestamp to switch on the receiver
    app_set_delayed_trx_time(ts_rx_init + (slot_id - 1) * SLOT_TIME + DELAY_BEFORE_RX);

    // Generate STS
    app_sts_generate();

    // Start receiving (delayed)
    ret = dwt_rxenable(DWT_START_RX_DELAYED);
    if (ret != DWT_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Wait until transmission is completed or interrupted
    ret = app_wait_rx_done();
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Read RX timestamp of RQST message
    ts_rx_rqst = app_read_rx_timestamp();

    // Read RX frame length
    uint16_t rx_frame_len = dwt_getframelength(NULL);
    if (rx_frame_len < HEADER_LEN)
    {
        return APP_RUN_ERROR;
    }

    // Read RX buffer
    uint8_t rx_buffer[rx_frame_len];
    dwt_readrxdata(rx_buffer, HEADER_LEN, NULL_RX_BUFFER_OFFSET);
    
    // Read MAC header
    mac_header_t mac_header;
    ret = mac_header_read(&mac_header, rx_buffer, sizeof(rx_buffer));
    if (ret != MAC_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Check MAC header
    if (mac_header.frame_ctrl != MAC_FRAME_CTRL ||
        mac_header.pan_id != pan_id ||
        mac_header.dest_addr != BROADCAST_MAC_ADDR ||
        mac_header.src_addr != tag_mac_addr)
    {
        return APP_RUN_WARNING;
    }

    // Read app header
    app_header_t app_header;
    ret = app_header_read(&app_header, rx_buffer, sizeof(rx_buffer));
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Check app header
    if (app_header.version != APP_VERSION ||
        app_header.msg_type != APP_MSG_RQST ||
        app_header.superframe_id != superframe_id ||
        app_header.slot_id != slot_id)
    {
        return APP_RUN_WARNING;
    }

    return APP_SUCCESS;
}

      
static int app_send_resp_msg (void)
{
    int ret;

    // Set TX frame length
    uint16_t tx_frame_len = HEADER_LEN + MIC_LEN + FCS_LEN;
    dwt_writetxfctrl(tx_frame_len, NULL_TX_BUFFER_OFFSET, RANGING_BIT_ENABLED);

    // Create TX buffer
    uint8_t tx_buffer[tx_frame_len - FCS_LEN];
    
    // Write MAC header
    mac_header_t mac_header;
    mac_header.frame_ctrl = MAC_FRAME_CTRL;
    mac_header.frame_seq_num = frame_seq_num;
    mac_header.pan_id = pan_id;
    mac_header.dest_addr = tag_mac_addr;
    mac_header.src_addr = mac_addr;
    ret = mac_header_write(&mac_header, tx_buffer, sizeof(tx_buffer));
    if (ret != MAC_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Write app header
    app_header_t app_header;
    app_header.version = APP_VERSION;
    app_header.msg_type = APP_MSG_RESP;
    app_header.superframe_id = superframe_id;
    app_header.slot_id = slot_id;
    ret = app_header_write(&app_header, tx_buffer, sizeof(tx_buffer));
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }
    // Set TX timestamp of RESPONSE message
    ts_tx_resp = (ts_rx_init + slot_id * SLOT_TIME) & CLOCK_COARSE_MASK;
    app_set_delayed_trx_time(ts_tx_resp);

    // Generate STS
    app_sts_generate();

    // Encrypt TX data
    ret = app_aes_encrypt(tx_buffer, tx_frame_len);
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Start transmission (delayed)
    ret = dwt_starttx(DWT_START_TX_DELAYED);
    if (ret != DWT_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Wait until transmission is completed or interrupted
    ret = app_wait_tx_done();
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Update frame sequence number
    frame_seq_num++;
    frame_seq_num &= 0xFF;

    return APP_SUCCESS;
}


static int app_wait_resp_msg (void)
{
    int ret;

    // Enable RX timeout
    app_set_rx_timeout(DEFAULT_RX_TIMEOUT);

    // Set timestamp to switch on the receiver
    app_set_delayed_trx_time(ts_rx_init + (slot_id - 1) * SLOT_TIME + DELAY_BEFORE_RX);

    // Generate STS
    app_sts_generate();

    // Start receiving (delayed)
    ret = dwt_rxenable(DWT_START_RX_DELAYED);
    if (ret != DWT_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Wait until reception is completed or interrupted
    ret = app_wait_rx_done();
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Counter for RESPONSE messages
    uint8_t cnt = slot_id - 2;

    // Read RX timestamp of RESPONSE message
    ts_rx_resp[cnt] = app_read_rx_timestamp();

    // Read RX frame length
    uint16_t rx_frame_len = dwt_getframelength(NULL);
    if (rx_frame_len < HEADER_LEN)
    {
        return APP_RUN_ERROR;
    }

    // Read RX buffer
    uint8_t rx_buffer[rx_frame_len];
    dwt_readrxdata(rx_buffer, HEADER_LEN, NULL_RX_BUFFER_OFFSET);

    // Read MAC header
    mac_header_t mac_header;
    ret = mac_header_read(&mac_header, rx_buffer, sizeof(rx_buffer));
    if (ret != MAC_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Check MAC header
    if (mac_header.frame_ctrl != MAC_FRAME_CTRL ||
        mac_header.pan_id != pan_id ||
        mac_header.dest_addr != mac_addr ||
        mac_header.src_addr != anchor_mac_addr[cnt])
    {
        return APP_RUN_WARNING;
    }

    // Read app header
    app_header_t app_header;
    ret = app_header_read(&app_header, rx_buffer, sizeof(rx_buffer));
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Check app header
    if (app_header.version != APP_VERSION ||
        app_header.msg_type != APP_MSG_RESP ||
        app_header.superframe_id != superframe_id ||
        app_header.slot_id != slot_id)
    {
        return APP_RUN_WARNING;
    }

    return APP_SUCCESS;
}


static int app_send_report_msg (void)
{
    int ret;

    // Set TX frame length
    uint16_t tx_frame_len = HEADER_LEN + 8 + 4 * num_anchors + MIC_LEN + FCS_LEN;
    dwt_writetxfctrl(tx_frame_len, NULL_TX_BUFFER_OFFSET, RANGING_BIT_ENABLED);

    // Create TX buffer
    uint8_t tx_buffer[tx_frame_len - FCS_LEN];

    // Write MAC header
    mac_header_t mac_header;
    mac_header.frame_ctrl = MAC_FRAME_CTRL;
    mac_header.frame_seq_num = frame_seq_num;
    mac_header.pan_id = pan_id;
    mac_header.dest_addr = BROADCAST_MAC_ADDR;
    mac_header.src_addr = mac_addr;
    ret = mac_header_write(&mac_header, tx_buffer, sizeof(tx_buffer));
    if (ret != MAC_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Write app header
    app_header_t app_header;
    app_header.version = APP_VERSION;
    app_header.msg_type = APP_MSG_RPT;
    app_header.superframe_id = superframe_id;
    app_header.slot_id = slot_id;
    ret = app_header_write(&app_header, tx_buffer, sizeof(tx_buffer));
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Write TX timestamp of REQUEST message
    tx_buffer[HEADER_LEN] = ts_tx_rqst & 0xFF;
    tx_buffer[HEADER_LEN + 1] = (ts_tx_rqst >> 8) & 0xFF;
    tx_buffer[HEADER_LEN + 2] = (ts_tx_rqst >> 16) & 0xFF;
    tx_buffer[HEADER_LEN + 3] = (ts_tx_rqst >> 24) & 0xFF;
    
    // Write RX timestamp of RESPONSE messages
    for (int k = 0; k < num_anchors; k++)
    {
        tx_buffer[HEADER_LEN + 4 + 4 * k] = ts_rx_resp[k] & 0xFF;
        tx_buffer[HEADER_LEN + 5 + 4 * k] = (ts_rx_resp[k] >> 8) & 0xFF;
        tx_buffer[HEADER_LEN + 6 + 4 * k] = (ts_rx_resp[k] >> 16) & 0xFF;
        tx_buffer[HEADER_LEN + 7 + 4 * k] = (ts_rx_resp[k] >> 24) & 0xFF;
    }

    // Set TX timestamp of REPORT message
    ts_tx_rpt = (ts_rx_init + slot_id * SLOT_TIME) & CLOCK_COARSE_MASK;

    // Write TX timestamp of REPORT message
    tx_buffer[HEADER_LEN + 4 + 4 * num_anchors] = ts_tx_rpt & 0xFF;
    tx_buffer[HEADER_LEN + 5 + 4 * num_anchors] = (ts_tx_rpt >> 8) & 0xFF;
    tx_buffer[HEADER_LEN + 6 + 4 * num_anchors] = (ts_tx_rpt >> 16) & 0xFF;
    tx_buffer[HEADER_LEN + 7 + 4 * num_anchors] = (ts_tx_rpt >> 24) & 0xFF;

    // Set TX timestamp
    app_set_delayed_trx_time(ts_tx_rpt);

    // Generate STS
    app_sts_generate();

    // Encrypt TX data
    ret = app_aes_encrypt(tx_buffer, tx_frame_len);
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Start transmission (delayed)
    ret = dwt_starttx(DWT_START_TX_DELAYED);
    if (ret != DWT_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Wait until transmission is completed
    ret = app_wait_tx_done();
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Update frame sequence number
    frame_seq_num++;
    frame_seq_num &= 0xFF;

    return APP_SUCCESS;
}


static int app_wait_report_msg (void)
{
    int ret;

    // Enable RX timeout
    app_set_rx_timeout(DEFAULT_RX_TIMEOUT);

    // Set timestamp to switch on the receiver
    app_set_delayed_trx_time(ts_rx_init + (slot_id - 1) * SLOT_TIME + DELAY_BEFORE_RX);

    // Generate STS
    app_sts_generate();

    // Start receiving (delayed)
    ret = dwt_rxenable(DWT_START_RX_DELAYED);
    if (ret != DWT_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Wait until reception is completed or interrupted
    ret = app_wait_rx_done();
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Read RX frame length
    uint16_t rx_frame_len = dwt_getframelength(NULL);
    if (rx_frame_len < HEADER_LEN)
    {
        return APP_RUN_ERROR;
    }

    // Read RX buffer
    uint8_t rx_buffer[rx_frame_len];
    dwt_readrxdata(rx_buffer, HEADER_LEN, NULL_RX_BUFFER_OFFSET);

    // Read MAC header
    mac_header_t mac_header;
    ret = mac_header_read(&mac_header, rx_buffer, sizeof(rx_buffer));
    if (ret != MAC_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Check MAC header
    if (mac_header.frame_ctrl != MAC_FRAME_CTRL ||
        mac_header.pan_id != pan_id ||
        mac_header.dest_addr != BROADCAST_MAC_ADDR ||
        mac_header.src_addr != tag_mac_addr)
    {
        return APP_RUN_WARNING;
    }

    // Read app header
    app_header_t app_header;
    ret = app_header_read(&app_header, rx_buffer, sizeof(rx_buffer));
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Check app header
    if (app_header.version != APP_VERSION ||
        app_header.msg_type != APP_MSG_RPT ||
        app_header.superframe_id != superframe_id ||
        app_header.slot_id != slot_id)
    {
        return APP_RUN_WARNING;
    }

    // Decrypt payload
    ret = app_aes_decrypt(rx_buffer, rx_frame_len);
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
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

    // Compute distance (DS-TWR formula) if RESP message was successfully received
    if (ts_rx_resp[anchor_id] != 0ULL)
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
    int ret;

    // Set TX frame length
    uint16_t tx_frame_len = HEADER_LEN + 4 + MIC_LEN + FCS_LEN;
    dwt_writetxfctrl(tx_frame_len, NULL_TX_BUFFER_OFFSET, RANGING_BIT_ENABLED);

    // Create TX buffer
    uint8_t tx_buffer[tx_frame_len - FCS_LEN];

    // Write MAC header
    mac_header_t mac_header;
    mac_header.frame_ctrl = MAC_FRAME_CTRL;
    mac_header.frame_seq_num = frame_seq_num;
    mac_header.pan_id = pan_id;
    mac_header.dest_addr = BROADCAST_MAC_ADDR;
    mac_header.src_addr = mac_addr;
    ret = mac_header_write(&mac_header, tx_buffer, sizeof(tx_buffer));
    if (ret != MAC_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Write app header
    app_header_t app_header;
    app_header.version = APP_VERSION;
    app_header.msg_type = APP_MSG_FINAL;
    app_header.superframe_id = superframe_id;
    app_header.slot_id = slot_id;
    ret = app_header_write(&app_header, tx_buffer, sizeof(tx_buffer));
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Write distance
    tx_buffer[HEADER_LEN] = dist[anchor_id] & 0xFF;
    tx_buffer[HEADER_LEN + 1] = (dist[anchor_id] >> 8) & 0xFF;
    tx_buffer[HEADER_LEN + 2] = (dist[anchor_id] >> 16) & 0xFF;
    tx_buffer[HEADER_LEN + 3] = (dist[anchor_id] >> 24) & 0xFF;

    // Set TX timestamp for FINAL frame
    ts_tx_final = (ts_rx_init + slot_id * SLOT_TIME) & CLOCK_COARSE_MASK;
    app_set_delayed_trx_time(ts_tx_final);

    // Generate STS
    app_sts_generate();

    // Encrypt TX data
    ret = app_aes_encrypt(tx_buffer, tx_frame_len);
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    ret = dwt_starttx(DWT_START_TX_DELAYED);
    if (ret != DWT_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Wait until transmission is completed or interrupted
    ret = app_wait_tx_done();
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Update frame sequence number
    frame_seq_num++;
    frame_seq_num &= 0xFF;

    return APP_SUCCESS;
}


static int app_wait_final_msg (void)
{
    int ret;

    // Enable RX timeout
    app_set_rx_timeout(DEFAULT_RX_TIMEOUT);

    // Set timestamp to switch on receiver
    app_set_delayed_trx_time(ts_rx_init + (slot_id - 1) * SLOT_TIME + DELAY_BEFORE_RX);

    // Generate STS
    app_sts_generate();

    // Start receiving (delayed)
    ret = dwt_rxenable(DWT_START_RX_DELAYED);
    if (ret != DWT_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Wait until reception is completed or interrupted
    ret = app_wait_rx_done();
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Read RX frame length
    uint16_t rx_frame_len = dwt_getframelength(NULL);
    if (rx_frame_len < HEADER_LEN)
    {
        return APP_RUN_ERROR;
    }

    // Read RX buffer
    uint8_t rx_buffer[rx_frame_len];
    dwt_readrxdata(rx_buffer, HEADER_LEN, NULL_RX_BUFFER_OFFSET);

    // Counter for FINAL messages
    uint8_t cnt = slot_id - 3 - num_anchors;

    // Read MAC header
    mac_header_t mac_header;
    ret = mac_header_read(&mac_header, rx_buffer, sizeof(rx_buffer));
    if (ret != MAC_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Check MAC header
    if (mac_header.frame_ctrl != MAC_FRAME_CTRL ||
        mac_header.pan_id != pan_id ||
        mac_header.dest_addr != BROADCAST_MAC_ADDR ||
        mac_header.src_addr != anchor_mac_addr[cnt])
    {
        return APP_RUN_WARNING;
    }

    // Read app header
    app_header_t app_header;
    ret = app_header_read(&app_header, rx_buffer, sizeof(rx_buffer));
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Check app header
    if (app_header.version != APP_VERSION ||
        app_header.msg_type != APP_MSG_FINAL ||
        app_header.superframe_id != superframe_id ||
        app_header.slot_id != slot_id)
    {
        return APP_RUN_WARNING;
    }

    // Decrypt payload
    ret = app_aes_decrypt(rx_buffer, rx_frame_len);
    if (ret != APP_SUCCESS)
    {
        return APP_RUN_ERROR;
    }

    // Read distance
    dist[cnt] = (uint64_t) rx_buffer[HEADER_LEN];
    dist[cnt] |= ((uint64_t) rx_buffer[HEADER_LEN + 1]) << 8;
    dist[cnt] |= ((uint64_t) rx_buffer[HEADER_LEN + 2]) << 16;
    dist[cnt] |= ((uint64_t) rx_buffer[HEADER_LEN + 3]) << 24;

    return APP_SUCCESS;
}


int app_run_ieee_802_15_4z_schedule (void)
{
    int ret = 0;

    // Reset scheduler state and slot ID
    app_state = APP_STATE_BEGIN;
    slot_id = 0;

    // Zeroing all timestamps
    ts_rx_init = 0ull;
    ts_tx_rqst = 0ull;
    ts_rx_rqst = 0ull;
    ts_tx_resp = 0ull;
    for (int k = 0; k < MAX_NUM_ANCHORS; k++)
    {
        ts_rx_resp[k] = 0ull;
    }
    ts_tx_rpt = 0ull;
    ts_rx_rpt= 0ull;
    ts_tx_final = 0ull;

    // Zeroing all distances
    for (int k = 0; k < MAX_NUM_ANCHORS; k++)
    {
        dist[k] = 0ull;
    }

    // Start loop
    while (1)
    {
        switch (app_state)
        {
            case APP_STATE_BEGIN:

                // Prepare to send INIT message if the node is the PAN coordinator
                if (mac_addr == PAN_COORDINATOR_MAC_ADDR)
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

                        default:

                            ret = APP_RUN_ERROR;
                            app_state = APP_STATE_END;

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

                        default:

                            ret = APP_RUN_ERROR;
                            app_state = APP_STATE_END;

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


int app_init (app_init_obj_t *obj)
{
    // Check if pointer is valid
    if (obj == NULL)
    {
        return APP_INIT_ERROR;
    }

    // Check if MAC address and PAN ID are valid
    if (obj->mac_addr == BROADCAST_MAC_ADDR || obj->pan_id == BROADCAST_PAN_ID)
    {
        return APP_INIT_ERROR;
    }

    // Set MAC address
    mac_addr = obj->mac_addr;

    // Set PAN ID
    pan_id = obj->pan_id;

    // Set STS key
    sts_key.key0 = obj->sts_key.key0;
    sts_key.key1 = obj->sts_key.key1;
    sts_key.key2 = obj->sts_key.key2;
    sts_key.key3 = obj->sts_key.key3;
    dwt_configurestskey(&sts_key);

    // Set AES key
    aes_key.key0 = obj->aes_key.key0;
    aes_key.key1 = obj->aes_key.key1;
    aes_key.key2 = obj->aes_key.key2;
    aes_key.key3 = obj->aes_key.key3;
    aes_key.key4 = obj->aes_key.key4;
    aes_key.key5 = obj->aes_key.key5;
    aes_key.key6 = obj->aes_key.key6;
    aes_key.key7 = obj->aes_key.key7;
    dwt_set_keyreg_128(&aes_key);
    
    return APP_SUCCESS;
}


int app_set_ctrl_params (app_ctrl_obj_t *obj)
{
    // Check if pointer is valid
    if (obj == NULL)
    {
        return APP_RUN_ERROR;
    }

    // Check if tag MAC address is valid
    if (obj->tag_mac_addr == BROADCAST_MAC_ADDR)
    {
        return APP_RUN_ERROR;
    }

    // Check if number of anchors exceeds the maximum limit
    if (obj->num_anchors == 0 || obj->num_anchors > MAX_NUM_ANCHORS)
    {
        return APP_RUN_ERROR;
    }

    // Check if the MAC addresses of all the anchors are valid
    for (int k = 0; k < obj->num_anchors; k++)
    {
        if (obj->anchor_mac_addr[k] == BROADCAST_MAC_ADDR)
        {
            return APP_RUN_ERROR;
        }
    }

    // Set tag MAC address
    tag_mac_addr = obj->tag_mac_addr;

    // Set number of anchors
    num_anchors = obj->num_anchors;

    // Set anchors MAC addresses
    for (int k = 0; k < num_anchors; k++)
    {
        anchor_mac_addr[k] = obj->anchor_mac_addr[k];
    }

    return APP_SUCCESS;
}


int app_read_log_info (app_log_info_t *info)
{
    // Check if pointer is valid
    if (info == NULL)
    {
        return APP_RUN_ERROR;
    }

    // Write superframe ID
    info->superframe_id = superframe_id;

    // Write timestamp of INIT message
    info->ts_init = ts_rx_init;

    // Write distances
    for (int k = 0; k < MAX_NUM_ANCHORS; k++)
    {
        info->dist[k] = dist[k];
    }

    // Write number of anchors
    info->num_anchors = num_anchors;

    // Write tag MAC address
    info->tag_mac_addr = tag_mac_addr;

    // Write anchors MAC addresses
    for (int k = 0; k < MAX_NUM_ANCHORS; k++)
    {
        info->anchor_mac_addr[k] = anchor_mac_addr[k];
    }

    return APP_SUCCESS;
}


void app_sleep (uint16_t ms)
{
    deca_sleep(ms);

    return;
}