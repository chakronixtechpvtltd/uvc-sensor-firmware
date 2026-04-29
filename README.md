# uvc-sensor-firmware
Description: UVC Optical Sensing System Firmware

 =============================================================================
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
 *  Stacks: g_spi0 (r_spi, ch1, CPOL=1, CPHA=1, 1MHz, 8-bit, cb=spi1_callback)
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
