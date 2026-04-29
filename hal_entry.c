/* =============================================================================
 * FILE:    hal_entry.c
 * PROJECT: UVC Sensor PCB Board — Final Firmware
 * VERSION: v1.0.0
 * DATE:    2026-04-25
 *
 * SCOPE REFERENCE: UVC_Firmware_Scope.docx
 *
 * ─── SYSTEM OVERVIEW ────────────────────────────────────────────────────────
 *
 *  4 boards arranged in a cylindrical tube, connected via flex PCB.
 *  Each board has: 1 UV LED + 1 normal LED + 1 photodiode.
 *  All face inward. Opposite faces measure each other.
 *
 *  FACING PAIRS:
 *    Board1 (UV_LED1 + PD1)  ←→  Board3 (UV_LED3 + PD3)
 *    Board2 (UV_LED2 + PD2)  ←→  Board4 (UV_LED4 + PD4)
 *
 * ─── MEASUREMENT SEQUENCE (round-robin, per scope) ──────────────────────────
 *
 *  Each LED cycle:
 *    Step 1: UV_LED1 ON  → wait 10ms (driver+noise settle) →
 *            wait 50ms (ADC settle) → read PD3 (AIN2) →
 *            UV_LED1 OFF → 40ms gap
 *    Step 2: UV_LED2 ON  → ... → read PD4 (AIN3) → OFF → 40ms gap
 *    Step 3: UV_LED3 ON  → ... → read PD1 (AIN0) → OFF → 40ms gap
 *    Step 4: UV_LED4 ON  → ... → read PD2 (AIN1) → OFF → 40ms gap
 *    Step 5: All OFF     → read temperature sensor
 *    Transmit full cycle data via USB CDC
 *    Repeat
 *
 *  Total per LED: 10+50+~5(SPI)+35 = 100ms ON + 300ms OFF = 400ms = 25% duty
 *
 * ─── ADC CONFIGURATION (verified from AD7175-2 datasheet Rev.B) ─────────────
 *
 *  FILTCON0 = 0x0D05
 *    Bit11     ENHFILTEN = 1        Enhanced filter enabled
 *    Bits[10:8] ENHFILT  = 101b     20 SPS, 50ms settling, 86dB 50/60Hz rej
 *    Bits[6:5]  ORDER    = 00       sinc5+sinc1 (required for enhanced filter)
 *    Bits[4:0]  ODR      = 00101b   25000 SPS base (overridden by ENHFILT)
 *    → 20 SPS output, 50ms settling time, 24-bit resolution
 *    Source: Table 23, AD7175-2 datasheet page 36
 *
 *  SETUPCON0 = 0x1300
 *    BI_UNIPOLAR=1, REFBUF+=1, REFBUF-=0, AINBUF+=1, AINBUF-=1,
 *    REF_SEL=10 (internal 2.5V)
 *
 *  ADCMODE = 0x8000
 *    REF_EN=1, SING_CYC=0, MODE=000 continuous, CLOCKSEL=00 internal osc
 *
 *  IFMODE = 0x0000  (no CRC, 24-bit data)
 *
 *  CH0: reconfigured per step (one channel active at a time)
 *    Step1: AIN2(+) vs AIN0(-) → PD3 vs dark PD1
 *    Step2: AIN3(+) vs AIN1(-) → PD4 vs dark PD2
 *    Step3: AIN0(+) vs AIN2(-) → PD1 vs dark PD3
 *    Step4: AIN1(+) vs AIN3(-) → PD2 vs dark PD4
 *    Step5: TEMP+(0x11) vs TEMP-(0x12) → temperature sensor
 *
 * ─── TEMPERATURE FORMULA (datasheet page 48) ────────────────────────────────
 *
 *  Bipolar offset binary: midscale = 2^23 = 8388608 = 0V
 *  V = (raw - 8388608) × 298.023e-9  [V per LSB with VREF=2.5V]
 *  T = (V / 477e-6) - 273.15         [°C]
 *  Verified: raw=8882903 → T≈35.6°C ✓
 *
 * ─── PIN MAP (verified from schematic Sheet 3) ───────────────────────────────
 *
 *  SPI:
 *    P103 = RSPI0_RSPCK   ADC clock
 *    P104 = RSPI0_MISO    ADC DOUT/RDY
 *    P105 = RSPI0_MOSI    ADC DIN
 *    P102 = GPIO Out High ADC CS (active LOW)
 *  UV LEDs (active HIGH → TPS923611 ADIM → UV LED):
 *    P400 = PWM_DIM_1 → UV LED1
 *    P401 = PWM_DIM_2 → UV LED2
 *    P409 = PWM_DIM_3 → UV LED3
 *    P408 = PWM_DIM_4 → UV LED4
 *  Indicator LEDs (active HIGH, 470R series):
 *    P101 = IND LED1
 *    P100 = IND LED2
 *    P014 = IND LED3
 *    P013 = IND LED4
 *  USB:
 *    P914 = USB_DP
 *    P915 = USB_DM
 *    P407 = USB_VBUS (sense)
 *
 * ─── FSP CONFIGURATION ───────────────────────────────────────────────────────
 *
 *  Stacks: g_spi0 (r_spi, ch0, CPOL=1, CPHA=1, 1MHz, 8-bit, cb=spi0_callback)
 *          g_basic0 (r_usb_basic, Peri, FullSpeed, cb=NULL, VBUS=P407)
 *          g_pcdc0  (r_usb_pcdc)
 *  Pins:   P103=RSPI0_RSPCK, P104=RSPI0_MISO, P105=RSPI0_MOSI
 *          P102=GPIO Out High, P407=USB_VBUS
 *          P400,P401,P408,P409=GPIO Out Low (UV LEDs — NO PWM needed)
 *          P101,P100,P014,P013=GPIO Out Low (indicator LEDs)
 *
 * ─── USB OUTPUT FORMAT ───────────────────────────────────────────────────────
 *
 *  === UVC Sensor Board v1.0.0 ===
 *  ADC ID: 0x0CDE  OK
 *  --------------------------------------------------
 *  UV_LED1_ON  PD3_RAW: 12456789
 *  UV_LED2_ON  PD4_RAW: 12398234
 *  UV_LED3_ON  PD1_RAW: 12501123
 *  UV_LED4_ON  PD2_RAW: 12478901
 *  TEMP: 35.68 C
 *  --- Cycle: 1 ---
 *  --------------------------------------------------
 *
 * ─── LIVE EXPRESSIONS ────────────────────────────────────────────────────────
 *
 *  g_adc_id      : should be 0x0CDE (AD7175-2 ID)
 *  g_adc_ok      : 1 = ADC init passed
 *  g_pd1_raw     : last PD1 24-bit reading
 *  g_pd2_raw     : last PD2 24-bit reading
 *  g_pd3_raw     : last PD3 24-bit reading
 *  g_pd4_raw     : last PD4 24-bit reading
 *  g_temp_celsius: last temperature in °C
 *  g_cycle_count : number of complete cycles
 *  g_usb_ready   : 1 = USB enumerated
 * =============================================================================
 */

#include "hal_data.h"
#include <string.h>
#include <stdio.h>

/* ============================================================
 * FIRMWARE VERSION
 * ============================================================ */
#define FW_VERSION  "v1.0.0"

/* ============================================================
 * PIN DEFINITIONS
 * ============================================================ */
/* UV LED control — active HIGH (TPS923611 ADIM HIGH = LED driver ON) */
#define UV_LED1_PIN     BSP_IO_PORT_04_PIN_00   /* P400 → PWM_DIM_1 */
#define UV_LED2_PIN     BSP_IO_PORT_04_PIN_01   /* P401 → PWM_DIM_2 */
#define UV_LED3_PIN     BSP_IO_PORT_04_PIN_09   /* P409 → PWM_DIM_3 */
#define UV_LED4_PIN     BSP_IO_PORT_04_PIN_08   /* P408 → PWM_DIM_4 */

/* Indicator LEDs — active HIGH */
#define IND_LED1_PIN    BSP_IO_PORT_01_PIN_01   /* P101 */
#define IND_LED2_PIN    BSP_IO_PORT_01_PIN_00   /* P100 */
#define IND_LED3_PIN    BSP_IO_PORT_00_PIN_14   /* P014 */
#define IND_LED4_PIN    BSP_IO_PORT_00_PIN_13   /* P013 */

/* ADC chip select — active LOW */
#define ADC_CS_PIN      BSP_IO_PORT_01_PIN_02   /* P102 */

/* ============================================================
 * AD7175-2 REGISTER ADDRESSES
 * ============================================================ */
#define ADC_REG_STATUS      0x00u
#define ADC_REG_ADCMODE     0x01u
#define ADC_REG_IFMODE      0x02u
#define ADC_REG_DATA        0x04u
#define ADC_REG_ID          0x07u
#define ADC_REG_CH0         0x10u
#define ADC_REG_CH1         0x11u
#define ADC_REG_CH2         0x12u
#define ADC_REG_CH3         0x13u
#define ADC_REG_SETUPCON0   0x20u
#define ADC_REG_FILTCON0    0x28u

/* SPI command byte */
#define ADC_CMD_WRITE(a)    ((uint8_t)(0x00u | ((a) & 0x3Fu)))
#define ADC_CMD_READ(a)     ((uint8_t)(0x40u | ((a) & 0x3Fu)))

/* Device ID */
#define ADC_ID_EXPECTED     0x0CD0u
#define ADC_ID_MASK         0xFFF0u

/* STATUS register */
#define ADC_STATUS_RDY      (1u << 7)   /* 0 = new data ready */

/* CH register — build value:
 * Bit15=CH_EN, Bits[13:12]=Setup0, Bits[9:5]=AINPOS, Bits[4:0]=AINNEG */
#define ADC_CH_ENABLE       (1u << 15)
#define ADC_CH_DISABLE      0x0001u
#define ADC_CH_CFG(pos,neg) ((uint16_t)(ADC_CH_ENABLE \
                             | (((uint16_t)(pos) & 0x1Fu) << 5u) \
                             |  ((uint16_t)(neg) & 0x1Fu)))

/* AIN input codes */
#define AIN0    0x00u   /* PD1 → V_OUTB1 */
#define AIN1    0x01u   /* PD2 → V_OUTB2 */
#define AIN2    0x02u   /* PD3 → V_OUTB3 */
#define AIN3    0x03u   /* PD4 → V_OUTB4 */
#define TEMP_P  0x11u   /* Internal temperature sensor + */
#define TEMP_N  0x12u   /* Internal temperature sensor - */

/* ============================================================
 * TIMING CONSTANTS (all in milliseconds, per scope document)
 * ============================================================ */
#define LED_DRIVER_SETTLE_MS    10u     /* TPS923611 + inductor noise settle */
#define ADC_FILTER_SETTLE_MS    50u     /* 20 SPS enhanced filter settle time */
#define LED_OFF_GAP_MS          40u     /* gap after LED OFF before next step */
#define LED_OFF_TOTAL_MS        300u    /* total OFF time per LED per cycle    */
#define RDY_TIMEOUT_MS          500u    /* ADC ready poll timeout              */
#define USB_ENUM_WAIT_MS        5000u   /* max wait for USB enumeration        */
#define ADC_INIT_SETTLE_MS      200u    /* after ADCMODE write on init         */

/* ============================================================
 * CIRCULAR BUFFER FOR USB TRANSMIT
 * Each slot holds one complete formatted output line (max 64 chars)
 * Producer: measurement loop — Consumer: USB write
 * ============================================================ */
#define CIRC_BUF_SLOTS      8u          /* power of 2 */
#define CIRC_BUF_LINE_LEN   64u

typedef struct
{
    char     lines[CIRC_BUF_SLOTS][CIRC_BUF_LINE_LEN];
    uint32_t head;                      /* write index */
    uint32_t tail;                      /* read index  */
} circ_buf_t;

static circ_buf_t s_circ;

static bool circ_full(void)
{
    return (((s_circ.head + 1u) & (CIRC_BUF_SLOTS - 1u)) == s_circ.tail);
}

static bool circ_empty(void)
{
    return (s_circ.head == s_circ.tail);
}

/* Push one line — silently drops if buffer full (measurement never blocked) */
static void circ_push(const char *line)
{
    if (circ_full()) return;
    strncpy(s_circ.lines[s_circ.head], line, CIRC_BUF_LINE_LEN - 1u);
    s_circ.lines[s_circ.head][CIRC_BUF_LINE_LEN - 1u] = '\0';
    s_circ.head = (s_circ.head + 1u) & (CIRC_BUF_SLOTS - 1u);
}

/* Pop one line — returns NULL if empty */
static const char *circ_pop(void)
{
    if (circ_empty()) return NULL;
    const char *line = s_circ.lines[s_circ.tail];
    s_circ.tail = (s_circ.tail + 1u) & (CIRC_BUF_SLOTS - 1u);
    return line;
}

/* ============================================================
 * GLOBAL VARIABLES — watch in e2studio Live Expressions
 * ============================================================ */
volatile uint16_t g_adc_id      = 0u;
volatile uint8_t  g_adc_ok      = 0u;
volatile uint32_t g_pd1_raw     = 0u;
volatile uint32_t g_pd2_raw     = 0u;
volatile uint32_t g_pd3_raw     = 0u;
volatile uint32_t g_pd4_raw     = 0u;
volatile float    g_temp_celsius = 0.0f;
volatile uint32_t g_cycle_count = 0u;
volatile uint8_t  g_usb_ready   = 0u;

/* USB CDC line coding (stored, returned to host on request) */
static usb_pcdc_linecoding_t s_line_coding =
{
    .dw_dte_rate   = 115200,
    .b_char_format = 0,
    .b_parity_type = 0,
    .b_data_bits   = 8,
};

/* SPI transfer buffers */
static uint8_t          s_spi_tx[5];
static uint8_t          s_spi_rx[5];
static volatile bool    s_spi_done = false;

/* USB TX staging buffer */
#define USB_TX_LEN  64u
static uint8_t       s_usb_tx[USB_TX_LEN];
static volatile bool s_usb_write_done = false;

/* ============================================================
 * SPI CALLBACK (registered as spi0_callback in FSP)
 * ============================================================ */
void spi1_callback(spi_callback_args_t *p_args)
{
    if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
    {
        s_spi_done = true;
    }
}

/* ============================================================
 * USB EVENT POLLING
 * Called regularly in the main loop — no RTOS, no callback.
 * Uses uint16_t for request type comparison to avoid
 * -Wtype-limits warnings with uint8_t.
 * ============================================================ */
static void usb_poll(void)
{
    usb_event_info_t info;
    usb_status_t     event;

    if (FSP_SUCCESS != R_USB_EventGet(&info, &event))
    {
        return;
    }

    switch (event)
    {
        case USB_STATUS_CONFIGURED:
            g_usb_ready = 1u;
            break;

        case USB_STATUS_DETACH:
        case USB_STATUS_SUSPEND:
            g_usb_ready = 0u;
            break;

        case USB_STATUS_RESUME:
            g_usb_ready = 1u;
            break;

        case USB_STATUS_WRITE_COMPLETE:
            s_usb_write_done = true;
            break;

        case USB_STATUS_REQUEST:
        {
            /* Use uint16_t — avoids -Wtype-limits with uint8_t */
            uint16_t req = (uint16_t)(info.setup.request_type & USB_BREQUEST);

            if (USB_PCDC_SET_LINE_CODING == req)
            {
                R_USB_PeriControlDataGet(&g_basic0_ctrl,
                    (uint8_t *)&s_line_coding,
                    (uint32_t)sizeof(s_line_coding));
            }
            else if (USB_PCDC_GET_LINE_CODING == req)
            {
                R_USB_PeriControlDataSet(&g_basic0_ctrl,
                    (uint8_t *)&s_line_coding,
                    (uint32_t)sizeof(s_line_coding));
            }
            else if (USB_PCDC_SET_CONTROL_LINE_STATE == req)
            {
                R_USB_PeriControlStatusSet(&g_basic0_ctrl,
                    USB_SETUP_STATUS_ACK);
            }
            else
            {
                R_USB_PeriControlStatusSet(&g_basic0_ctrl,
                    USB_SETUP_STATUS_STALL);
            }
            break;
        }

        default:
            break;
    }
}

/* ============================================================
 * USB CDC TRANSMIT — non-blocking
 *
 * Uses a simple state machine:
 *   IDLE       → pop from buffer, call R_USB_Write, go to WRITING
 *   WRITING    → wait for WRITE_COMPLETE event (set by usb_poll)
 *   WRITE_COMPLETE → go back to IDLE
 *
 * Call usb_drain() regularly in the main loop.
 * The measurement loop is NEVER blocked by USB.
 * If USB is not connected or slow, data just stays in the buffer.
 * ============================================================ */
typedef enum { USB_TX_IDLE = 0, USB_TX_WRITING } usb_tx_state_t;
static usb_tx_state_t s_usb_tx_state = USB_TX_IDLE;

static void usb_drain(void)
{
    if (0u == g_usb_ready)
    {
        s_usb_tx_state = USB_TX_IDLE;
        return;
    }

    /* If previous write completed, go back to idle */
    if (USB_TX_WRITING == s_usb_tx_state)
    {
        if (s_usb_write_done)
        {
            s_usb_tx_state = USB_TX_IDLE;
        }
        else
        {
            return; /* still waiting for write complete */
        }
    }

    /* IDLE — try to send next line from buffer */
    const char *line = circ_pop();
    if (NULL == line)
    {
        return;   /* nothing to send */
    }

    uint32_t len = (uint32_t)strlen(line);
    if (0u == len || len >= USB_TX_LEN)
    {
        return;
    }

    memcpy(s_usb_tx, line, len);
    s_usb_write_done = false;

    fsp_err_t err = R_USB_Write(&g_basic0_ctrl,
                                 s_usb_tx, len, USB_CLASS_PCDC);
    if (FSP_SUCCESS == err)
    {
        s_usb_tx_state = USB_TX_WRITING;
    }
    else
    {
        /* Write failed — USB not ready, discard this line */
        s_usb_tx_state = USB_TX_IDLE;
    }
}

/* ============================================================
 * PIN HELPERS
 * ============================================================ */
static void pin_set(bsp_io_port_pin_t pin, bool on)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(pin, on ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();
}

static void all_off(void)
{
    /* UV LEDs off */
    pin_set(UV_LED1_PIN,  false);
    pin_set(UV_LED2_PIN,  false);
    pin_set(UV_LED3_PIN,  false);
    pin_set(UV_LED4_PIN,  false);
    /* Indicator LEDs off */
    pin_set(IND_LED1_PIN, false);
    pin_set(IND_LED2_PIN, false);
    pin_set(IND_LED3_PIN, false);
    pin_set(IND_LED4_PIN, false);
}

static void delay_ms(uint32_t ms)
{
    R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
}

/* ============================================================
 * ADC CS CONTROL
 * ============================================================ */
static void adc_cs_assert(void)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(ADC_CS_PIN, BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();
}

static void adc_cs_deassert(void)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(ADC_CS_PIN, BSP_IO_LEVEL_HIGH);
    R_BSP_PinAccessDisable();
}

/* ============================================================
 * SPI TRANSFER (blocking via s_spi_done flag)
 * ============================================================ */
static fsp_err_t spi_xfer(uint8_t *tx, uint8_t *rx, uint32_t len)
{
    s_spi_done = false;

    fsp_err_t e = R_SPI_WriteRead(&g_spi0_ctrl, tx, rx, len,
                                   SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != e)
    {
        return e;
    }

    uint32_t timeout = 200000u;
    while (!s_spi_done && timeout > 0u)
    {
        timeout--;
    }

    return (timeout > 0u) ? FSP_SUCCESS : FSP_ERR_TIMEOUT;
}

/* ============================================================
 * ADC LOW-LEVEL REGISTER ACCESS
 * ============================================================ */
static fsp_err_t adc_write16(uint8_t addr, uint16_t val)
{
    s_spi_tx[0] = ADC_CMD_WRITE(addr);
    s_spi_tx[1] = (uint8_t)(val >> 8u);
    s_spi_tx[2] = (uint8_t)(val & 0xFFu);
    adc_cs_assert();
    fsp_err_t e = spi_xfer(s_spi_tx, s_spi_rx, 3u);
    adc_cs_deassert();
    return e;
}

static fsp_err_t adc_read16(uint8_t addr, uint16_t *val)
{
    s_spi_tx[0] = ADC_CMD_READ(addr);
    s_spi_tx[1] = 0u;
    s_spi_tx[2] = 0u;
    adc_cs_assert();
    fsp_err_t e = spi_xfer(s_spi_tx, s_spi_rx, 3u);
    adc_cs_deassert();
    if (FSP_SUCCESS == e)
    {
        *val = (uint16_t)(((uint16_t)s_spi_rx[1] << 8u) | s_spi_rx[2]);
    }
    return e;
}

static fsp_err_t adc_reset(void)
{
    uint8_t tx[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    uint8_t rx[8];
    adc_cs_assert();
    fsp_err_t e = spi_xfer(tx, rx, 8u);
    adc_cs_deassert();
    delay_ms(5u);
    return e;
}

static fsp_err_t adc_read_status(uint8_t *status)
{
    s_spi_tx[0] = ADC_CMD_READ(ADC_REG_STATUS);
    s_spi_tx[1] = 0u;
    adc_cs_assert();
    fsp_err_t e = spi_xfer(s_spi_tx, s_spi_rx, 2u);
    adc_cs_deassert();
    if (FSP_SUCCESS == e)
    {
        *status = s_spi_rx[1];
    }
    return e;
}

static fsp_err_t adc_read_data24(uint32_t *data)
{
    s_spi_tx[0] = ADC_CMD_READ(ADC_REG_DATA);
    s_spi_tx[1] = 0u;
    s_spi_tx[2] = 0u;
    s_spi_tx[3] = 0u;
    adc_cs_assert();
    fsp_err_t e = spi_xfer(s_spi_tx, s_spi_rx, 4u);
    adc_cs_deassert();
    if (FSP_SUCCESS == e)
    {
        *data = ((uint32_t)s_spi_rx[1] << 16u)
              | ((uint32_t)s_spi_rx[2] <<  8u)
              |  (uint32_t)s_spi_rx[3];
    }
    return e;
}

/* ============================================================
 * ADC WAIT FOR RDY
 * Polls STATUS register until RDY bit = 0 (new data ready)
 * Also calls usb_poll to keep USB alive during wait
 * ============================================================ */
static fsp_err_t adc_wait_rdy(void)
{
    uint8_t  status  = 0u;
    uint32_t timeout = RDY_TIMEOUT_MS;

    while (timeout > 0u)
    {
        usb_poll();

        if (FSP_SUCCESS != adc_read_status(&status))
        {
            return FSP_ERR_ABORTED;
        }

        if (0u == (status & ADC_STATUS_RDY))
        {
            return FSP_SUCCESS;     /* data ready */
        }

        delay_ms(1u);
        timeout--;
    }

    return FSP_ERR_TIMEOUT;
}

/* ============================================================
 * ADC INIT
 * Configures common registers. Channel config done per measurement.
 * ============================================================ */
static fsp_err_t adc_init(void)
{
    fsp_err_t e;

    /* Hardware reset via SPI (64 clocks, DIN=1) */
    e = adc_reset();
    if (FSP_SUCCESS != e) return e;

    /* Verify device ID — expected 0x0CDx */
    uint16_t id = 0u;
    e = adc_read16(ADC_REG_ID, &id);
    if (FSP_SUCCESS != e) return e;
    g_adc_id = id;
    if ((id & ADC_ID_MASK) != ADC_ID_EXPECTED)
    {
        return FSP_ERR_INVALID_HW_CONDITION;
    }

    /* IFMODE = 0x0000
     * No CRC, no DATA_STAT, 24-bit data, simplest mode */
    e = adc_write16(ADC_REG_IFMODE, 0x0000u);
    if (FSP_SUCCESS != e) return e;

    /* SETUPCON0 = 0x1000
     * Bit12 BI_UNIPOLAR=1  bipolar offset binary coding
     * Bit11 REFBUF+=0      REF+ buffer DISABLED
     * Bit10 REFBUF-=0      REF- buffer DISABLED
     * Bit9  AINBUF+=0      AIN+ input buffer DISABLED
     * Bit8  AINBUF-=0      AIN- input buffer DISABLED
     * Bits[5:4] REF_SEL=10 internal 2.5V reference
     *
     * WHY BUFFERS DISABLED:
     * With buffers enabled, input range = AVSS+100mV to AVDD1-1.5V = 0.1V to 3.5V
     * With buffers disabled, input range = AVSS to AVDD1 = 0V to 5V
     * V_OUTB op-amp outputs can exceed 3.5V with 47MΩ TIA gain at UV LED on,
     * causing ADC saturation (0xFFFFFF). Disabling buffers extends range to 5V.
     * Note: temperature sensor reading still works with buffers disabled.
     * Source: AD7175-2 datasheet Table 2.2, AINBUF description. */
    e = adc_write16(ADC_REG_SETUPCON0, 0x1000u);
    if (FSP_SUCCESS != e) return e;

    /* FILTCON0 = 0x0D05
     * Bit11     ENHFILTEN=1   enhanced filter enabled
     * Bits[10:8] ENHFILT=101  20 SPS, 50ms settling, 86dB 50/60Hz rejection
     * Bits[6:5]  ORDER=00     sinc5+sinc1 (required for enhanced filter)
     * Bits[4:0]  ODR=00101    (base rate, overridden by enhanced filter)
     * Source: AD7175-2 datasheet Table 23, page 36 */
    e = adc_write16(ADC_REG_FILTCON0, 0x0D05u);
    if (FSP_SUCCESS != e) return e;

    /* Disable all channels — will be enabled one at a time per step */
    e = adc_write16(ADC_REG_CH0, ADC_CH_DISABLE);
    if (FSP_SUCCESS != e) return e;
    e = adc_write16(ADC_REG_CH1, ADC_CH_DISABLE);
    if (FSP_SUCCESS != e) return e;
    e = adc_write16(ADC_REG_CH2, ADC_CH_DISABLE);
    if (FSP_SUCCESS != e) return e;
    e = adc_write16(ADC_REG_CH3, ADC_CH_DISABLE);
    if (FSP_SUCCESS != e) return e;

    /* ADCMODE = 0x8000
     * Bit15 REF_EN=1    enable internal 2.5V reference
     * Bit13 SING_CYC=0  disabled (single channel, no need for sing_cyc)
     * Bits[6:4] MODE=000 continuous conversion
     * Bits[3:2] CLOCKSEL=00 internal oscillator (16 MHz)
     * Write LAST — this starts conversions */
    e = adc_write16(ADC_REG_ADCMODE, 0x8000u);
    if (FSP_SUCCESS != e) return e;

    delay_ms(ADC_INIT_SETTLE_MS);

    g_adc_ok = 1u;
    return FSP_SUCCESS;
}

/* ============================================================
 * ADC SET CHANNEL
 * Disables CH0, waits briefly, then re-enables with new
 * AINPOS/AINNEG pair. Call delay ADC_FILTER_SETTLE_MS after.
 * ============================================================ */
static fsp_err_t adc_set_channel(uint8_t ainpos, uint8_t ainneg)
{
    /* Disable CH0 to reset the filter state */
    fsp_err_t e = adc_write16(ADC_REG_CH0, ADC_CH_DISABLE);
    if (FSP_SUCCESS != e) return e;

    delay_ms(5u);   /* short gap for filter reset to take effect */

    /* Re-enable CH0 with new input pair */
    e = adc_write16(ADC_REG_CH0, ADC_CH_CFG(ainpos, ainneg));
    return e;
}

/* ============================================================
 * ADC READ ONE SAMPLE
 * Waits for RDY then reads 24-bit raw result
 * ============================================================ */
static fsp_err_t adc_read_one(uint32_t *raw)
{
    fsp_err_t e = adc_wait_rdy();
    if (FSP_SUCCESS != e) return e;
    return adc_read_data24(raw);
}

/* ============================================================
 * TEMPERATURE READ AND CONVERT
 * Formula verified from AD7175-2 datasheet page 48 and
 * confirmed working in hal_entry_spi_adc_test_v1.0.1.c
 *
 * IMPORTANT: AD7175-2 datasheet page 48 states:
 * "The temperature sensor requires the analog input buffers
 *  to be enabled on both analog inputs."
 * So we must temporarily set SETUPCON0=0x1300 (buffers ON)
 * for temperature, then restore 0x1000 (buffers OFF) for PD.
 * ============================================================ */
static fsp_err_t adc_read_temperature(float *temp_c)
{
    fsp_err_t e;

    /* Enable input buffers for temperature sensor (required per datasheet) */
    e = adc_write16(ADC_REG_SETUPCON0, 0x1300u);
    if (FSP_SUCCESS != e) return e;

    /* Configure CH0 for internal temperature sensor */
    e = adc_set_channel(TEMP_P, TEMP_N);
    if (FSP_SUCCESS != e) return e;

    /* Wait for filter to settle (50ms at 20 SPS) */
    delay_ms(ADC_FILTER_SETTLE_MS);

    /* Read raw 24-bit result */
    uint32_t raw = 0u;
    e = adc_read_one(&raw);

    /* Restore input buffers disabled for photodiode measurements
     * (PD op-amp outputs can exceed 3.5V so buffers must be off) */
    adc_write16(ADC_REG_SETUPCON0, 0x1000u);

    if (FSP_SUCCESS != e) return e;

    /* Temperature conversion (from working v1.0.1):
     * Bipolar offset binary: midscale = 2^23 = 8388608 = 0V
     * V = (raw - 8388608) × 298.023e-9   [V per LSB, VREF=2.5V]
     * T = (V / 477e-6) - 273.15          [°C]
     *
     * Verified: raw=8882903 → T≈35.6°C ✓ */
    int32_t signed_code = (int32_t)raw - 8388608;
    float   voltage     = (float)signed_code * 298.023e-9f;
    *temp_c = (voltage / 477.0e-6f) - 273.15f;

    return FSP_SUCCESS;
}

/* ============================================================
 * MEASURE ONE UV LED STEP
 * Sequence per scope document:
 *   1. UV LED + indicator ON
 *   2. Wait LED_DRIVER_SETTLE_MS (driver + switching noise settle)
 *   3. Configure ADC channel
 *   4. Wait ADC_FILTER_SETTLE_MS (filter settle at 20 SPS)
 *   5. Read ADC (wait RDY + SPI read)
 *   6. UV LED + indicator OFF
 *   7. Store result, push to circular buffer
 *   8. Wait LED_OFF_GAP_MS (dead time before next step)
 *
 * Parameters:
 *   uv_pin   : UV LED GPIO pin
 *   ind_pin  : indicator LED GPIO pin
 *   ainpos   : ADC AINPOS input (active photodiode)
 *   ainneg   : ADC AINNEG input (dark photodiode as reference)
 *   pd_name  : string label e.g. "PD3"
 *   led_name : string label e.g. "UV_LED1"
 *   p_raw    : output raw 24-bit ADC value
 * ============================================================ */
static void measure_step(bsp_io_port_pin_t uv_pin,
                         bsp_io_port_pin_t ind_pin,
                         uint8_t           ainpos,
                         uint8_t           ainneg,
                         const char       *led_name,
                         const char       *pd_name,
                         volatile uint32_t *p_raw)
{
    char line[CIRC_BUF_LINE_LEN];

    /* Step 1 — UV LED ON + indicator ON */
    pin_set(uv_pin,  true);
    pin_set(ind_pin, true);

    /* Step 2 — wait for TPS923611 driver settling
     * and inductor switching noise dissipation */
    delay_ms(LED_DRIVER_SETTLE_MS);

    /* Step 3+4 — configure ADC channel and wait for filter to settle */
    if (FSP_SUCCESS != adc_set_channel(ainpos, ainneg))
    {
        pin_set(uv_pin,  false);
        pin_set(ind_pin, false);
        snprintf(line, sizeof(line),
            "%s_ON  %s_RAW: CFG_ERR\r\n", led_name, pd_name);
        circ_push(line);
        delay_ms(LED_OFF_GAP_MS);
        return;
    }

    /* Wait ADC filter settle: 50ms for 20 SPS enhanced filter */
    delay_ms(ADC_FILTER_SETTLE_MS);

    /* Step 5 — read ADC conversion result */
    uint32_t raw = 0u;
    if (FSP_SUCCESS != adc_read_one(&raw))
    {
        pin_set(uv_pin,  false);
        pin_set(ind_pin, false);
        snprintf(line, sizeof(line),
            "%s_ON  %s_RAW: RDY_ERR\r\n", led_name, pd_name);
        circ_push(line);
        delay_ms(LED_OFF_GAP_MS);
        return;
    }

    /* Step 6 — UV LED OFF + indicator OFF */
    pin_set(uv_pin,  false);
    pin_set(ind_pin, false);

    /* Step 7 — store result and push to circular buffer */
    *p_raw = raw;
    snprintf(line, sizeof(line),
        "%s_ON  %s_RAW: %8lu\r\n",
        led_name, pd_name, (unsigned long)raw);
    circ_push(line);

    /* Drain USB during the OFF gap — gives USB time to transmit */
    uint32_t gap = LED_OFF_GAP_MS;
    while (gap > 0u)
    {
        usb_poll();
        usb_drain();
        delay_ms(1u);
        gap--;
    }
}

/* ============================================================
 * hal_entry — MAIN ENTRY POINT
 * ============================================================ */
void hal_entry(void)
{
    /* ---- Startup: all outputs off, CS deasserted ---- */
    all_off();
    adc_cs_deassert();
    delay_ms(100u);

    /* ---- Open SPI ---- */
    if (FSP_SUCCESS != R_SPI_Open(&g_spi0_ctrl, &g_spi0_cfg))
    {
        /* SPI open failed — blink IND_LED1 rapidly forever */
        while (1)
        {
            pin_set(IND_LED1_PIN, true);  delay_ms(100u);
            pin_set(IND_LED1_PIN, false); delay_ms(100u);
        }
    }

    /* ---- Open USB ---- */
    if (FSP_SUCCESS != R_USB_Open(&g_basic0_ctrl, &g_basic0_cfg))
    {
        /* USB open failed — blink IND_LED2 rapidly forever */
        while (1)
        {
            pin_set(IND_LED2_PIN, true);  delay_ms(100u);
            pin_set(IND_LED2_PIN, false); delay_ms(100u);
        }
    }

    /* ---- Init ADC ---- */
    fsp_err_t adc_err = adc_init();
    /* Note: we continue even if ADC init fails —
     * USB will report the error to the PC */

    /* ---- Wait for USB enumeration (blink IND_LED1 while waiting) ---- */
    uint32_t wait_ms = 0u;
    while (0u == g_usb_ready && wait_ms < USB_ENUM_WAIT_MS)
    {
        usb_poll();
        pin_set(IND_LED1_PIN, true);  delay_ms(100u);
        pin_set(IND_LED1_PIN, false); delay_ms(100u);
        wait_ms += 200u;
    }
    delay_ms(200u);

    /* ---- Flash all indicator LEDs — system ready ---- */
    pin_set(IND_LED1_PIN, true); pin_set(IND_LED2_PIN, true);
    pin_set(IND_LED3_PIN, true); pin_set(IND_LED4_PIN, true);
    delay_ms(500u);
    all_off();
    delay_ms(100u);

    /* ---- Send startup banner ---- */
    {
        char buf[CIRC_BUF_LINE_LEN];

        circ_push("\r\n=== UVC Sensor Board " FW_VERSION " ===\r\n");
        usb_drain();

        snprintf(buf, sizeof(buf),
            "ADC ID: 0x%04X  %s\r\n",
            g_adc_id,
            (FSP_SUCCESS == adc_err) ? "OK" : "INIT_FAIL");
        circ_push(buf);
        usb_drain();

        circ_push("LED1->PD3 | LED2->PD4 | LED3->PD1 | LED4->PD2\r\n");
        usb_drain();
        circ_push("--------------------------------------------------\r\n");
        usb_drain();
    }

    /* ---- If ADC failed, report and loop ---- */
    if (FSP_SUCCESS != adc_err)
    {
        circ_push("ERROR: ADC init failed. Check SPI.\r\n");
        while (1)
        {
            usb_drain();
            usb_poll();
            delay_ms(1000u);
        }
    }

    /* ---- DARK READING DIAGNOSTIC (all UV LEDs OFF) ----
     * Reads all 4 PD channels with LEDs off to check baseline.
     * If values are near 8388608 (midscale) = op-amp output near 0V = OK
     * If values are 16777215 (max) = op-amp saturated at +5V rail
     * If values are 0 = op-amp saturated at -5V / GND rail
     * -------------------------------------------------------- */
    {
        char buf[CIRC_BUF_LINE_LEN];
        uint32_t dark = 0u;

        circ_push("\r\n--- DARK READING (all UV LEDs OFF) ---\r\n");
        usb_drain();

        /* AIN0 dark */
        if (FSP_SUCCESS == adc_set_channel(AIN0, AIN2))
        {
            delay_ms(ADC_FILTER_SETTLE_MS);
            if (FSP_SUCCESS == adc_read_one(&dark))
            {
                snprintf(buf, sizeof(buf), "AIN0(PD1) dark: %lu\r\n",
                         (unsigned long)dark);
            }
            else { snprintf(buf, sizeof(buf), "AIN0(PD1) dark: ERR\r\n"); }
        }
        else { snprintf(buf, sizeof(buf), "AIN0(PD1) dark: CFG_ERR\r\n"); }
        circ_push(buf); usb_drain();

        /* AIN2 dark */
        if (FSP_SUCCESS == adc_set_channel(AIN2, AIN0))
        {
            delay_ms(ADC_FILTER_SETTLE_MS);
            if (FSP_SUCCESS == adc_read_one(&dark))
            {
                snprintf(buf, sizeof(buf), "AIN2(PD3) dark: %lu\r\n",
                         (unsigned long)dark);
            }
            else { snprintf(buf, sizeof(buf), "AIN2(PD3) dark: ERR\r\n"); }
        }
        else { snprintf(buf, sizeof(buf), "AIN2(PD3) dark: CFG_ERR\r\n"); }
        circ_push(buf); usb_drain();

        circ_push("--- END DARK READING ---\r\n\r\n");
        usb_drain();
    }

/* ================================================================
 * MAIN MEASUREMENT LOOP
 * ================================================================ */
    while (1)
    {
        /* Drain any pending USB data before starting new cycle */
        for (uint8_t i = 0u; i < 8u; i++)
        {
            usb_poll();
            usb_drain();
        }

        /* ---- Step 1: UV_LED1 ON → read PD3 ----
         * AIN2(PD3) vs AIN0(PD1 dark — UV_LED3 is OFF) */
        measure_step(UV_LED1_PIN, IND_LED1_PIN,
                     AIN2, AIN0,
                     "UV_LED1", "PD3",
                     &g_pd3_raw);
        usb_poll(); usb_drain();
        usb_poll(); usb_drain();

        /* ---- Step 2: UV_LED2 ON → read PD4 ----
         * AIN3(PD4) vs AIN1(PD2 dark — UV_LED4 is OFF) */
        measure_step(UV_LED2_PIN, IND_LED2_PIN,
                     AIN3, AIN1,
                     "UV_LED2", "PD4",
                     &g_pd4_raw);
        usb_poll(); usb_drain();
        usb_poll(); usb_drain();

        /* ---- Step 3: UV_LED3 ON → read PD1 ----
         * AIN0(PD1) vs AIN2(PD3 dark — UV_LED1 is OFF) */
        measure_step(UV_LED3_PIN, IND_LED3_PIN,
                     AIN0, AIN2,
                     "UV_LED3", "PD1",
                     &g_pd1_raw);
        usb_poll(); usb_drain();
        usb_poll(); usb_drain();

        /* ---- Step 4: UV_LED4 ON → read PD2 ----
         * AIN1(PD2) vs AIN3(PD4 dark — UV_LED2 is OFF) */
        measure_step(UV_LED4_PIN, IND_LED4_PIN,
                     AIN1, AIN3,
                     "UV_LED4", "PD2",
                     &g_pd2_raw);
        usb_poll(); usb_drain();
        usb_poll(); usb_drain();

        /* ---- Step 5: All OFF → read temperature ---- */
        all_off();
        {
            float temp = 0.0f;
            char  buf[CIRC_BUF_LINE_LEN];

            if (FSP_SUCCESS == adc_read_temperature(&temp))
            {
                g_temp_celsius = temp;

                /* Integer formatting — no -u_printf_float linker flag needed */
                int32_t t_int  = (int32_t)temp;
                int32_t t_frac = (int32_t)((temp - (float)t_int) * 100.0f);
                if (t_frac < 0) t_frac = -t_frac;

                snprintf(buf, sizeof(buf),
                    "TEMP: %ld.%02ld C\r\n",
                    (long)t_int, (long)t_frac);
            }
            else
            {
                snprintf(buf, sizeof(buf), "TEMP: READ_ERR\r\n");
            }

            circ_push(buf);
            usb_drain();
        }

        /* ---- End of cycle — print cycle count and separator ---- */
        g_cycle_count++;
        {
            char buf[CIRC_BUF_LINE_LEN];
            snprintf(buf, sizeof(buf),
                "--- Cycle: %lu ---\r\n",
                (unsigned long)g_cycle_count);
            circ_push(buf);
            usb_drain();
        }
        /* Separator line printed at start of NEXT cycle data */
        circ_push("\r\n--------------------------------------------------\r\n");
        usb_drain();

        usb_poll();

    } /* end while(1) */
}
