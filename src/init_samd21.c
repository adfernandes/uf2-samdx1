#include "uf2.h"
#define SYSCTRL_FUSES_OSC32K_CAL_ADDR   (NVMCTRL_OTP4 + 4)
#define SYSCTRL_FUSES_OSC32K_CAL_Pos   6
#define 	SYSCTRL_FUSES_OSC32K_ADDR   SYSCTRL_FUSES_OSC32K_CAL_ADDR
#define 	SYSCTRL_FUSES_OSC32K_Pos   SYSCTRL_FUSES_OSC32K_CAL_Pos
#define 	SYSCTRL_FUSES_OSC32K_Msk   (0x7Fu << SYSCTRL_FUSES_OSC32K_Pos)

volatile bool g_interrupt_enabled = true;

// SAMD21 starts at 1MHz by default.
uint32_t current_cpu_frequency_MHz = 1;

static void gclk_sync(void) {
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
        ;
}

static void dfll_sync(void) {
    while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0)
        ;
}

#define NVM_SW_CALIB_DFLL48M_COARSE_VAL   58
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     64


void system_init(void) {

  NVMCTRL->CTRLB.bit.RWS = 1; // One wait state for 48 MHz @ 3.3 V

  /*
   * At reset:
   * - OSC8M clock source is enabled with a divider by 8 (1 MHz).
   * - Generic Clock Generator 0 (GCLKMAIN) is using OSC8M as source.
   */

  SYSCTRL->OSC8M.bit.PRESC = 0x0; // Set the OSC8M clock source to 8 MHz

#if defined(CRYSTALLESS)

  /* Turn on DFLL with USB correction and sync to the external USB SOF
   *
   * SAM D21 DA1 Family DataSheet DS40001882F
   * SAM D21 Family Silicon Errata and Data Sheet Clarification DS80000760D Revision D 2019-04
   *
   * USB Clock Recovery Mode
   *   The SOF signal from USB device will be used as reference clock (CLK_DFLL_REF),
   *   ignoring the selected generic clock reference. When the USB device is connected,
   *   a SOF will be sent every 1 ms, thus DFLLVAL.MUX bits should be written with
   *   0xBB80 (48000) to obtain a 48 MHz clock.
   */

    // Works around a quirk in the hardware (Errata 1.2.1):
    // the DFLLCTRL register must be manually reset to this value before configuration.
    dfll_sync();
    SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
    dfll_sync();

    /* Write the coarse and fine calibration from NVM. */
    uint32_t coarse = ((*(uint32_t*)FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
    uint32_t fine = ((*(uint32_t*)FUSES_DFLL48M_FINE_CAL_ADDR) & FUSES_DFLL48M_FINE_CAL_Msk) >> FUSES_DFLL48M_FINE_CAL_Pos;
    SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);
    dfll_sync();

    /* Configure the settings to enable USB clock recovery mode. */
    SYSCTRL->DFLLCTRL.reg |=
        /* Enable USB clock recovery mode */
        SYSCTRL_DFLLCTRL_USBCRM |
        /* Disable chill cycle as per datasheet to speed up locking.
        This is specified in Section 17.6.7.2.2, and chill cycles are described in Section 17.6.7.2.1. */
        SYSCTRL_DFLLCTRL_CCDIS;

    /* Configure the DFLL to multiply the 1 kHz clock to 48 MHz. */
    SYSCTRL->DFLLMUL.reg =
        /* This value is output frequency / reference clock frequency, so 48 MHz / 1 kHz */
        SYSCTRL_DFLLMUL_MUL(48000) |
        /* The coarse and fine values can be set to their minimum
           since coarse is fixed in USB clock recovery mode and fine should lock on quickly. */
        SYSCTRL_DFLLMUL_FSTEP(1) |
        SYSCTRL_DFLLMUL_CSTEP(1);

    /* Set the DFLL into closed loop mode and enable it. */
    SYSCTRL->DFLLCTRL.bit.MODE = 1;
    SYSCTRL->DFLLCTRL.bit.ENABLE = 1;
    dfll_sync();

#else

    SYSCTRL->XOSC32K.reg =
        SYSCTRL_XOSC32K_STARTUP(6) | SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K;
    SYSCTRL->XOSC32K.bit.ENABLE = 1;
    while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0)
        ;

    GCLK->GENDIV.reg = GCLK_GENDIV_ID(1);
    gclk_sync();

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN;
    gclk_sync();

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(0) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
    gclk_sync();

    SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0;
    dfll_sync();

    SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP(31) | SYSCTRL_DFLLMUL_FSTEP(511) |
                           SYSCTRL_DFLLMUL_MUL((CPU_FREQUENCY / (32 * 1024)));
    dfll_sync();

    SYSCTRL->DFLLCTRL.reg |=
        SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK | SYSCTRL_DFLLCTRL_QLDIS;
    dfll_sync();

    SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE;

    while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC) == 0 ||
           (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKF) == 0)
        ;
    dfll_sync();

#endif

    // Configure DFLL48M as source for GCLK_GEN 0
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(0);
    gclk_sync();

    // Add GCLK_GENCTRL_OE below to output GCLK0 on the SWCLK pin.
    GCLK->GENCTRL.reg =
        GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;
    gclk_sync();

    SysTick_Config(1000);

    // Uncomment these two lines to output GCLK0 on the SWCLK pin.
    // PORT->Group[0].PINCFG[30].bit.PMUXEN = 1;
    // Set the port mux mask for odd processor pin numbers, PA30 = 30 is even number, PMUXE = PMUX Even
    // PORT->Group[0].PMUX[30 / 2].reg |= PORT_PMUX_PMUXE_H;

    current_cpu_frequency_MHz = 48;

}

void SysTick_Handler(void) { LED_TICK(); }
