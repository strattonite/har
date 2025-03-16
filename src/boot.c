#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"
#include "warp.h"
#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "arm_math.h"
#include "math.h"
#include "inference.h"
#include "devMMA8451Q.h"
#include "devSSD1331.h"

volatile WarpI2CDeviceState deviceMMA8451QState;

volatile i2c_master_state_t i2cMasterState;
volatile spi_master_state_t spiMasterState;
volatile spi_master_user_config_t spiUserConfig;
volatile lpuart_user_config_t lpuartUserConfig;
volatile lpuart_state_t lpuartState;

volatile bool gWarpBooted = false;
volatile uint32_t gWarpI2cBaudRateKbps = kWarpDefaultI2cBaudRateKbps;
volatile uint32_t gWarpUartBaudRateBps = kWarpDefaultUartBaudRateBps;
volatile uint32_t gWarpSpiBaudRateKbps = kWarpDefaultSpiBaudRateKbps;
volatile uint32_t gWarpSleeptimeSeconds = kWarpDefaultSleeptimeSeconds;
volatile WarpModeMask gWarpMode = kWarpModeDisableAdcOnSleep;
volatile uint32_t gWarpI2cTimeoutMilliseconds = kWarpDefaultI2cTimeoutMilliseconds;
volatile uint32_t gWarpSpiTimeoutMicroseconds = kWarpDefaultSpiTimeoutMicroseconds;
volatile uint32_t gWarpUartTimeoutMilliseconds = kWarpDefaultUartTimeoutMilliseconds;
volatile uint32_t gWarpMenuPrintDelayMilliseconds = kWarpDefaultMenuPrintDelayMilliseconds;
volatile uint32_t gWarpSupplySettlingDelayMilliseconds = kWarpDefaultSupplySettlingDelayMilliseconds;
volatile uint16_t gWarpCurrentSupplyVoltage = kWarpDefaultSupplyVoltageMillivolts;
char gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];

#define kWarpConstantStringI2cFailure "\rI2C failed, reg 0x%02x, code %d\n"
#define kWarpConstantStringErrorInvalidVoltage "\rInvalid supply voltage [%d] mV!"
#define kWarpConstantStringErrorSanity "\rSanity check failed!"

/*
 *	Since only one SPI transaction is ongoing at a time in our implementation
 */
uint8_t gWarpSpiCommonSourceBuffer[kWarpMemoryCommonSpiBufferBytes];
uint8_t gWarpSpiCommonSinkBuffer[kWarpMemoryCommonSpiBufferBytes];

static void disableTPS62740(void);
static void enableTPS62740(uint16_t voltageMillivolts);
static void setTPS62740CommonControlLines(uint16_t voltageMillivolts);
/*
 *	Derived from KSDK power_manager_demo.c BEGIN>>>
 */
clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *notify, void *callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t clockManagerCallbackUserlevelStructure =
    {
        .callback = clockManagerCallbackRoutine,
        .callbackType = kClockManagerCallbackBeforeAfter,
        .callbackData = NULL};

static clock_manager_callback_user_config_t *clockCallbackTable[] =
    {
        &clockManagerCallbackUserlevelStructure};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *notify, void *callbackData)
{
    clock_manager_error_code_t result = kClockManagerSuccess;

    switch (notify->notifyType)
    {
    case kClockManagerNotifyBefore:
        break;
    case kClockManagerNotifyRecover:
    case kClockManagerNotifyAfter:
        break;
    default:
        result = kClockManagerError;
        break;
    }

    return result;
}

/*
 *	Override the RTC IRQ handler
 */
void RTC_IRQHandler(void)
{
    if (RTC_DRV_IsAlarmPending(0))
    {
        RTC_DRV_SetAlarmIntCmd(0, false);
    }
}

/*
 *	Override the RTC Second IRQ handler
 */
void RTC_Seconds_IRQHandler(void)
{
    gWarpSleeptimeSeconds++;
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t
callback0(power_manager_notify_struct_t *notify, power_manager_callback_data_t *dataPtr)
{
    WarpPowerManagerCallbackStructure *callbackUserData = (WarpPowerManagerCallbackStructure *)dataPtr;
    power_manager_error_code_t status = kPowerManagerError;

    switch (notify->notifyType)
    {
    case kPowerManagerNotifyBefore:
        status = kPowerManagerSuccess;
        break;
    case kPowerManagerNotifyAfter:
        status = kPowerManagerSuccess;
        break;
    default:
        callbackUserData->errorCount++;
        break;
    }

    return status;
}

void warpEnableI2Cpins(void)
{
    CLOCK_SYS_EnableI2cClock(0);

    /*
     *	Setup:
     *
     *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	(ALT2 == I2C)
     *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	(ALT2 == I2C)
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

    I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
}

void disableTPS62740(void)
{
#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
    GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_REGCTRL);
#endif
}

void enableTPS62740(uint16_t voltageMillivolts)
{
#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
    /*
     *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
     *
     *	Setup:
     *		PTB5/kWarpPinTPS62740_REGCTRL for GPIO
     *		PTB6/kWarpPinTPS62740_VSEL4 for GPIO
     *		PTB7/kWarpPinTPS62740_VSEL3 for GPIO
     *		PTB10/kWarpPinTPS62740_VSEL2 for GPIO
     *		PTB11/kWarpPinTPS62740_VSEL1 for GPIO
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

    setTPS62740CommonControlLines(voltageMillivolts);
    GPIO_DRV_SetPinOutput(kWarpPinTPS62740_REGCTRL);
#endif
}

void setTPS62740CommonControlLines(uint16_t voltageMillivolts)
{
#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
    switch (voltageMillivolts)
    {
    case 1800:
    {
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 1900:
    {
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 2000:
    {
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 2100:
    {
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 2200:
    {
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 2300:
    {
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 2400:
    {
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 2500:
    {
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 2600:
    {
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 2700:
    {
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 2800:
    {
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 2900:
    {
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 3000:
    {
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 3100:
    {
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 3200:
    {
        GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    case 3300:
    {
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
        GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

        break;
    }

    /*
     *	Should never happen, due to previous check in warpScaleSupplyVoltage()
     */
    default:
    {
        warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
    }
    }

    /*
     *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
     */
    OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
#endif
}

void warpScaleSupplyVoltage(uint16_t voltageMillivolts)
{
    if (voltageMillivolts == gWarpCurrentSupplyVoltage)
    {
        return;
    }

#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
    if (voltageMillivolts >= 1800 && voltageMillivolts <= 3300)
    {
        enableTPS62740(voltageMillivolts);
        gWarpCurrentSupplyVoltage = voltageMillivolts;
    }
    else
    {
        warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorInvalidVoltage RTT_CTRL_RESET "\n", voltageMillivolts);
    }
#endif
}

void warpPrint(const char *fmt, ...)
{
    int fmtlen;
    va_list arg;

    /*
     *	We use an ifdef rather than a C if to allow us to compile-out
     *	all references to SEGGER_RTT_*printf if we don't want them.
     *
     *	NOTE: SEGGER_RTT_vprintf takes a va_list* rather than a va_list
     *	like usual vprintf. We modify the SEGGER_RTT_vprintf so that it
     *	also takes our print buffer which we will eventually send over
     *	BLE. Using SEGGER_RTT_vprintf() versus the libc vsnprintf saves
     *	2kB flash and removes the use of malloc so we can keep heap
     *	allocation to zero.
     */

    /*
     *	We can't use SEGGER_RTT_vprintf to format into a buffer
     *	since SEGGER_RTT_vprintf formats directly into the special
     *	RTT memory region to be picked up by the RTT / SWD mechanism...
     */
    va_start(arg, fmt);
    fmtlen = SEGGER_RTT_vprintf(0, fmt, &arg, gWarpPrintBuffer, kWarpDefaultPrintBufferSizeBytes);
    va_end(arg);

    if (fmtlen < 0)
    {
        SEGGER_RTT_WriteString(0, gWarpEfmt);
    }

    return;
}

volatile bool dataReady = false;

void PORTA_IRQHandler(void)
{
    dataReady = true;
    GPIO_DRV_ClearPinIntFlag(kWarpPinMMA8451QInt2);
#if (DEBUG_LED)
    GPIO_DRV_TogglePinOutput(kGpioLED1);
#endif
}

/*
 *	LLW_IRQHandler override. Since FRDM_KL03Z48M is not defined,
 *	according to power_manager_demo.c, what we need is LLW_IRQHandler.
 *	However, elsewhere in the power_manager_demo.c, the code assumes
 *	FRDM_KL03Z48M _is_ defined (e.g., we need to use LLWU_IRQn, not
 *	LLW_IRQn). Looking through the code base, we see in
 *
 *		ksdk1.1.0/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
 *
 *	that the startup initialization assembly requires LLWU_IRQHandler,
 *	not LLW_IRQHandler. See power_manager_demo.c, circa line 216, if
 *	you want to find out more about this dicsussion.
 */
void LLWU_IRQHandler(void)
{
    /*
     *	BOARD_* defines are defined in warp.h
     */
    LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}

int main(void)
{
    power_manager_user_config_t warpPowerModeWaitConfig;
    power_manager_user_config_t warpPowerModeStopConfig;
    power_manager_user_config_t warpPowerModeVlpwConfig;
    power_manager_user_config_t warpPowerModeVlpsConfig;
    power_manager_user_config_t warpPowerModeVlls0Config;
    power_manager_user_config_t warpPowerModeVlls1Config;
    power_manager_user_config_t warpPowerModeVlls3Config;
    power_manager_user_config_t warpPowerModeRunConfig;

    const power_manager_user_config_t warpPowerModeVlprConfig = {
        .mode = kPowerManagerVlpr,
        .sleepOnExitValue = false,
        .sleepOnExitOption = false};

    power_manager_user_config_t const *powerConfigs[] = {
        /*
         *	NOTE: POWER_SYS_SetMode() depends on this order
         *
         *	See KSDK13APIRM.pdf Section 55.5.3
         */
        &warpPowerModeWaitConfig,
        &warpPowerModeStopConfig,
        &warpPowerModeVlprConfig,
        &warpPowerModeVlpwConfig,
        &warpPowerModeVlpsConfig,
        &warpPowerModeVlls0Config,
        &warpPowerModeVlls1Config,
        &warpPowerModeVlls3Config,
        &warpPowerModeRunConfig,
    };

    WarpPowerManagerCallbackStructure powerManagerCallbackStructure;

    /*
     *	Callback configuration structure for power manager
     */
    const power_manager_callback_user_config_t callbackCfg0 = {
        callback0,
        kPowerManagerCallbackBeforeAfter,
        (power_manager_callback_data_t *)&powerManagerCallbackStructure};

    /*
     *	Pointers to power manager callbacks.
     */
    power_manager_callback_user_config_t const *callbacks[] = {
        &callbackCfg0};

    CLOCK_SYS_EnablePortClock(0);
    CLOCK_SYS_EnablePortClock(1);

    OSA_Init();

    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

    CLOCK_SYS_Init(g_defaultClockConfigurations,
                   CLOCK_CONFIG_NUM, /* The default value of this is defined in fsl_clock_MKL03Z4.h as 2 */
                   &clockCallbackTable,
                   ARRAY_SIZE(clockCallbackTable));
    CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

    memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));

    warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
    warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

    warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
    warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

    warpPowerModeWaitConfig = warpPowerModeVlprConfig;
    warpPowerModeWaitConfig.mode = kPowerManagerWait;

    warpPowerModeStopConfig = warpPowerModeVlprConfig;
    warpPowerModeStopConfig.mode = kPowerManagerStop;

    warpPowerModeVlls0Config = warpPowerModeVlprConfig;
    warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

    warpPowerModeVlls1Config = warpPowerModeVlprConfig;
    warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

    warpPowerModeVlls3Config = warpPowerModeVlprConfig;
    warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

    warpPowerModeRunConfig.mode = kPowerManagerRun;

    POWER_SYS_Init(&powerConfigs,
                   sizeof(powerConfigs) / sizeof(power_manager_user_config_t *),
                   &callbacks,
                   sizeof(callbacks) / sizeof(power_manager_callback_user_config_t *));

    GPIO_DRV_Init(inputPins, outputPins);

    PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

    PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);

    initMMA8451Q(0x1D /* i2cAddress */, kWarpDefaultSupplyVoltageMillivoltsMMA8451Q);
    configureMMA8451Q();

    uint8_t fifo_buf[96];
    uint32_t i;
    uint32_t idx;
    float x, y, z;
    uint16_t val;
    int16_t valSigned;
    i2c_status_t status;
    q15_t samples[128];
    q15_t mags[64];
    uint16_t predictions[4];
    uint32_t counter = 0;
    uint32_t loopCounter = 0;

#if (DEBUG_LED)
    GPIO_DRV_ClearPinOutput(kGpioLED1);
#endif
    readSensorRegisterMMA8451Q(0x00, 1);
    readSensorRegisterMMA8451Q(0x0c, 1);

    arm_cfft_instance_q15 cfft;
    arm_cfft_init_64_q15(&cfft);

    // enter VLPR
    CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_VLPR, kClockManagerPolicyForcible);
    POWER_SYS_SetMode(kPowerManagerVlpr, kPowerManagerPolicyAgreement);

    while (1)
    {
        if (dataReady)
        {
#if (OUTPUT_POWER)
            warpPrint(
                "{\"type\": \"POW\", \"mode\": %d, \"loops\": %u}\n",
                POWER_SYS_GetCurrentMode(),
                loopCounter)
                OSA_TimeDelay(10);
#endif

            dataReady = false;
            readSensorRegisterMMA8451Q(0x0c, 1);
            readSensorRegisterMMA8451Q(0x00, 1);
            status = readFIFO(&fifo_buf[0]);
            memmove(&samples[0], &samples[32], 96 * sizeof(q15_t));

            for (i = 0; i < 16; i++)
            {
                idx = i * 6;
                val = ((fifo_buf[idx] & 0xFF) << 6) | (fifo_buf[idx + 1] >> 2);
                valSigned = (val ^ (1 << 13)) - (1 << 13);
                x = (float)valSigned / 4096.;

                val = ((fifo_buf[idx + 2] & 0xFF) << 6) | (fifo_buf[idx + 3] >> 2);
                valSigned = (val ^ (1 << 13)) - (1 << 13);
                y = (float)valSigned / 4096.;

                val = ((fifo_buf[idx + 4] & 0xFF) << 6) | (fifo_buf[idx + 5] >> 2);
                valSigned = (val ^ (1 << 13)) - (1 << 13);
                z = (float)valSigned / 4096.;

                /*
                 * Multiply by 4096 * 2 to increase dynamic range
                 * int16 -> float32 shouldn't lose any precision.
                 * Technically we could shift output by two bits since our maximum
                 * accel reading is 2g => 8192, but this seems to overflow.
                 * This also has the convenient property that arm_cmplx_mag_q15
                 * outputs Q2.14, so returns correctly scaled values
                 */
                samples[96 + i * 2] = (q15_t)(hypotf(hypotf(x, y), z) * 8192.);
                samples[97 + i * 2] = 0;
            }

            if (counter % 4 == 3)
            {
#if (OUTPUT_TIME)
                for (i = 0; i < 64; i++)
                {
                    warpPrint(
                        "{\"type\": \"TIME\", \"group\": %d, \"sample_num\": %d, \"Re\": %d, \"Im\": %d, \"Mag\": %d}\n",
                        counter / 4,
                        i,
                        (int16_t)(samples[2 * i]),
                        (int16_t)(samples[1 + 2 * i]),
                        (int16_t)(samples[2 * i]));
                    OSA_TimeDelay(10);
                }
#endif

                arm_cfft_q15(&cfft, &samples[0], 0, 0);
                arm_cmplx_mag_q15(&samples[0], &mags[0], 64);
                predict(&mags[0], &predictions[0]);

#if (OUTPUT_FREQ)
                for (i = 0; i < 64; i++)
                {
                    warpPrint(
                        "{\"type\": \"FREQ\", \"group\": %d, \"sample_num\": %d, \"Re\": %d, \"Im\": %d, \"Mag\": %d}\n",
                        counter / 4,
                        i,
                        (int16_t)(samples[2 * i]),
                        (int16_t)(samples[1 + 2 * i]),
                        (int16_t)(mags[i]));
                    OSA_TimeDelay(10);
                }
                OSA_TimeDelay(10);
#endif

                warpPrint(
                    "{\"type\": \"PRED\", \"group\": %u, \"jogging\": %u, \"sitting\": %u, \"stationary\": %u, \"walking\": %u}\n",
                    counter / 4,
                    predictions[0],
                    predictions[1],
                    predictions[2],
                    predictions[3]);
            }
            counter++;
        }
        loopCounter += 1;
        // go to VLPW until dataReady interrupt
        POWER_SYS_SetMode(kWarpPowerModeVLPW, kPowerManagerPolicyAgreement);
    }
}