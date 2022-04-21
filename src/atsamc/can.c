// Serial over CAN emulation for STM32 boards.
//
// Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
// Copyright (C) 2020 Pontus Borg <glpontus@gmail.com>
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include <stdint.h> // standard types
#include <samc21.h> // LITTLE_ENDIAN warning
#include "autoconf.h" // CONFIG_MACH_STM32F1
#include "board/armcm_timer.h" // udelay
#include "fasthash.h" // fasthash64
#include "board/armcm_boot.h" // armcm_enable_irq
#include "generic/canbus.h" // canbus_notify_tx
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT
#include "mcan.h" // SAMC21 CANBUS hal

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/* size of our custom Rx and Tx Buffer Elements, in words */
#define RAM_BUF_SIZE                  (MCAN_RAM_BUF_HDR_SIZE + 64u / 4)

#define RAM_ARRAY_SIZE_FILT_STD       (8u)
#define RAM_ARRAY_SIZE_FILT_EXT       (8u)
#define RAM_FIFO_SIZE_RX0             (12u)
/* no Rx FIFO 1 in our Message RAM */
#define RAM_ARRAY_SIZE_RX             (4u)
/* no Tx Event FIFO in our Message RAM */
#define RAM_ARRAY_SIZE_TX             (4u)
#define RAM_FIFO_SIZE_TX              (4u)
#define MSG_ID_ALLOW_ALL_MASK     0x000ul     /* bits 0 & 1 are don't care */

/* size of our custom Message RAM, in words */
#define MSG_RAM_SIZE      ( \
      RAM_ARRAY_SIZE_FILT_STD * MCAN_RAM_FILT_STD_SIZE \
    + RAM_ARRAY_SIZE_FILT_EXT * MCAN_RAM_FILT_EXT_SIZE \
    + RAM_FIFO_SIZE_RX0 * RAM_BUF_SIZE \
    + RAM_ARRAY_SIZE_RX * RAM_BUF_SIZE \
    + RAM_ARRAY_SIZE_TX * RAM_BUF_SIZE \
    + RAM_FIFO_SIZE_TX * RAM_BUF_SIZE )

#define MSG_LEN_1_CAN     8
#define MSG_LEN_1_CAN_FD  64
#define MSG_LEN_2_CAN     7
#define MSG_LEN_2_CAN_FD  48
#define MSG_ID_2          (CAN_STD_MSG_ID | 0x444)
#define MSG_ID_2_MASK     0x7FCul     /* bits 0 & 1 are don't care */
#define UID_BASE          ((uint32_t)0x0080A00C)       /*!< Unique device ID register base address */

#define RX_BUFFER_0       0
#define RX_BUFFER_1       1
#define RX_FILTER_0       0
#define RX_FILTER_1       1
#define MAX_FILTERS  16
#define MAX_RX_BUFFERS 8

typedef struct frame_desc
{
	uint32_t id;
	uint8_t data[8];
	uint8_t len;
	uint8_t buf_idx;
} frame_desc;

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/* the Message RAM is allocated from within the SAMC21's RAM
 */
static uint32_t mcan_msg_ram[MSG_RAM_SIZE] __attribute__ ((aligned (4)));
static struct mcan_set mcan;
static volatile bool rx_ded_buffer_data = false;
static volatile bool tx_clear_to_send = true;
//static uint8_t      * txMailbox0;
static frame_desc rx_buffers[MAX_RX_BUFFERS];
static volatile uint8_t rxb_start;
static volatile uint8_t rxb_end;

// Read the next CAN packet
int
canbus_read(uint32_t *id, uint8_t *data)
{
  int len;

  // Process data packets and return the first admin packet
  while (rxb_start != rxb_end) {
    frame_desc *rx_buf = rx_buffers + rxb_start;

    // Process the data packets here. They are ignored in the calling function.
    if (*id != CANBUS_ID_ADMIN) {
      canbus_process_data(rx_buf->id, rx_buf->len, rx_buf->data);
    }

    // Copy the info from the packet into the return datastructures
    memcpy(data, rx_buf->data, rx_buf->len);
    *id = rx_buf->id;
    len = rx_buf->len;
    rxb_start = (rxb_start + 1) % MAX_RX_BUFFERS;

    // Notify the rx task just in case there are more packets available
    canbus_notify_rx();

    return len;
  }

  return -1;
}

// Transmit a packet
int
canbus_send(uint32_t id, uint32_t len, uint8_t *data)
{
    if (!mcan_is_enabled(&mcan)) {
        return -1;
    }

    // We should be able to send immediately, but just in case...
    while (!tx_clear_to_send);

    // Send the packet
    if (mcan_enqueue_outgoing_msg(&mcan, id, len, data) != 0xFF) {
        tx_clear_to_send = false;
        return len;
    }

    return -1;
}

// Setup the receive packet filter
void
canbus_set_filter(uint32_t id)
{
    mcan_filter_single_id(&mcan, RX_BUFFER_0, RX_FILTER_1, id);
}

// This function handles CAN global interrupts
void CAN0_Handler(void)
{

  // Read any packets that are available into the rx buffers
  if (mcan_rx_array_data(&mcan)) {
    struct mcan_msg_info msg;
    mcan_clear_rx_array_flag(&mcan);

    if (mcan_rx_buffer_data(&mcan, RX_BUFFER_0)) {
      uint8_t next_rxb = (rxb_end + 1) % MAX_RX_BUFFERS;
      if (next_rxb != rxb_start) {
        frame_desc *rx_buf = rx_buffers + rxb_end;
        msg.data = rx_buf->data;
        msg.data_len = sizeof(rx_buf->data);
        mcan_read_rx_buffer(&mcan, RX_BUFFER_0, &msg);
        if (msg.data_len > 0) {
          rx_buf->id = msg.id;
          rx_buf->len = msg.data_len;
          canbus_notify_rx();
          rxb_end = next_rxb;
        }
      }
    }

    if (mcan_rx_buffer_data(&mcan, RX_BUFFER_1)) {
      uint8_t next_rxb = (rxb_end + 1) % MAX_RX_BUFFERS;
      if (next_rxb != rxb_start) {
        frame_desc *rx_buf = rx_buffers + rxb_end;
        msg.data = rx_buf->data;
        msg.data_len = sizeof(rx_buf->data);
        mcan_read_rx_buffer(&mcan, RX_BUFFER_1, &msg);
        if (msg.data_len > 0) {
          rx_buf->id = msg.id;
          rx_buf->len = msg.data_len;
          canbus_notify_rx();
          rxb_end = next_rxb;
        }
      }
    }
  }

  // See if the last tx transfer is complete.
  if (mcan_is_tx_complete(&mcan)) {
    mcan_clear_tx_flag(&mcan);
    tx_clear_to_send = true;
  }

  // Wake up the tx task if we're ready to transmit
  if (tx_clear_to_send) {
    canbus_notify_tx();
  }
}

void
can_init(void)
{
  const struct mcan_config mcan_cfg = {
    .id = ID_CAN0,
    .regs = CAN0,
    .msg_ram = mcan_msg_ram,

    .array_size_filt_std = RAM_ARRAY_SIZE_FILT_STD,
    .array_size_filt_ext = RAM_ARRAY_SIZE_FILT_EXT,
    .fifo_size_rx0 = RAM_FIFO_SIZE_RX0,
    .fifo_size_rx1 = 0,
    .array_size_rx = RAM_ARRAY_SIZE_RX,
    .fifo_size_tx_evt = 0,
    .array_size_tx = RAM_ARRAY_SIZE_TX,
    .fifo_size_tx = RAM_FIFO_SIZE_TX,

    .buf_size_rx_fifo0 = 64,
    .buf_size_rx_fifo1 = 0,
    .buf_size_rx = 64,
    .buf_size_tx = 64,

    /*
      using values from AT6493 (SAMC21 app note); the plus values are to add on what the MCAN driver subtracts back off
    */
    .bit_rate = CONFIG_CANBUS_FREQUENCY,
    .quanta_before_sp = 10 + 2,
    .quanta_after_sp = 3 + 1,
    .quanta_sync_jump = 3 + 1,

    /*
      AT6493 (SAMC21 app note) 'fast' values were unhelpfully the same as normal speed; these are for double (1MBit)
      the maximum peripheral clock of 48MHz on the SAMC21 does restrict us from very high rates
    */
    .bit_rate_fd = CONFIG_CANBUS_FREQUENCY,
    .quanta_before_sp_fd = 10 + 2,
    .quanta_after_sp_fd = 3 + 1,
    .quanta_sync_jump_fd = 3 + 1,
  };
  uint32_t mcan_msg_ram_size = ARRAY_SIZE(mcan_msg_ram);

  /*##-1- Configure the CAN #######################################*/
    
  PORT->Group[0].DIRSET.reg = PORT_PA24;
  PORT->Group[0].DIRCLR.reg = PORT_PA25;
  PORT->Group[0].PINCFG[24].reg = PORT_PINCFG_INEN | PORT_PINCFG_PMUXEN;
  PORT->Group[0].PINCFG[25].reg = PORT_PINCFG_INEN | PORT_PINCFG_PMUXEN;
  PORT->Group[0].PMUX[24 / 2].reg = PORT_PMUX_PMUXE(6 /* CAN0 G */) | PORT_PMUX_PMUXO(6 /* CAN0 G */); /* have to write odd and even at once */

  // Enable the clock
  GCLK->PCHCTRL[CAN0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
  MCLK->AHBMASK.reg |= MCLK_AHBMASK_CAN0;

  if (!mcan_configure_msg_ram(&mcan_cfg, &mcan_msg_ram_size))
    return;
  if (mcan_msg_ram_size > ARRAY_SIZE(mcan_msg_ram)) {
    return;
  } else if (mcan_msg_ram_size != ARRAY_SIZE(mcan_msg_ram)) {
    return;
  }          

  if (!mcan_initialize(&mcan, &mcan_cfg)) {
    return;
  }
  mcan_set_tx_queue_mode(&mcan);
  mcan_loopback_off(&mcan);
  //mcan_loopback_on(&mcan);
  mcan_set_mode(&mcan, MCAN_MODE_CAN);
  mcan_enable(&mcan);
  mcan_enable_rx_array_flag(&mcan, 0);
  mcan.cfg.regs->IE.reg |= CAN_IE_RF0NE;
  mcan.cfg.regs->IE.reg |= CAN_IE_TCFE;
  mcan.cfg.regs->IE.reg |= CAN_IE_TCE;

  while (!mcan_is_enabled(&mcan)) { }

  /*##-2- Configure the CAN Filter #######################################*/
  mcan_filter_single_id(&mcan, RX_BUFFER_0, 0, CANBUS_ID_ADMIN);

  /*##-3- Configure Interrupts #################################*/
  armcm_enable_irq(CAN0_Handler, CAN0_IRQn, 0);
  __enable_irq();

  // Convert unique 128-bit chip id into 48 bit representation
  uint64_t hash = fasthash64((uint8_t*)UID_BASE, 16, 0xA16231A7);
  canbus_set_uuid(&hash);

  return;
}
DECL_INIT(can_init);
