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

#include "devMMA8451Q.h"
volatile WarpI2CDeviceState deviceMMA8451QState;

#include "devSSD1331.h"

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

static void sleepUntilReset(void);
static void lowPowerPinStates(void);
static void disableTPS62740(void);
static void enableTPS62740(uint16_t voltageMillivolts);
static void setTPS62740CommonControlLines(uint16_t voltageMillivolts);
static void dumpProcessorState(void);
static int char2int(int character);
static uint8_t readHexByte(void);
static int read4digits(void);

/*
 *	TODO: change the following to take byte arrays
 */
WarpStatus writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus writeBytesToSpi(uint8_t *payloadBytes, int payloadLength);

void warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);

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

/*
 *	IRQ handler for the interrupt from RTC, which we wire up
 *	to PTA0/IRQ0/LLWU_P7 in Glaux. BOARD_SW_LLWU_IRQ_HANDLER
 *	is a synonym for PORTA_IRQHandler.
 */
void BOARD_SW_LLWU_IRQ_HANDLER(void)
{
    /*
     *	BOARD_* defines are defined in warp.h
     */
    PORT_HAL_ClearPortIntFlag(BOARD_SW_LLWU_BASE);
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
/*
 *	Derived from KSDK power_manager_demo.c <<END
 */

void sleepUntilReset(void)
{
    while (1)
    {
        warpLowPowerSecondsSleep(1, false /* forceAllPinsIntoLowPowerState */);

        warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
    }
}

void enableLPUARTpins(void)
{
    /*
     *	Enable UART CLOCK
     */
    CLOCK_SYS_EnableLpuartClock(0);

    /*
     *	Set UART pin association. See, e.g., page 99 in
     *
     *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
     *
     *	Setup:
     *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
     *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX

//TODO: we don't use hw flow control so don't need RTS/CTS
 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt3);

    // TODO: we don't use hw flow control so don't need RTS/CTS
    //	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
    //	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
    //	GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO_UART_RTS);
    //	GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI_UART_CTS);

    /*
     *	Initialize LPUART0. See KSDK13APIRM.pdf section 40.4.3, page 1353
     */
    lpuartUserConfig.baudRate = gWarpUartBaudRateBps;
    lpuartUserConfig.parityMode = kLpuartParityDisabled;
    lpuartUserConfig.stopBitCount = kLpuartOneStopBit;
    lpuartUserConfig.bitCountPerChar = kLpuart8BitsPerChar;
    lpuartUserConfig.clockSource = kClockLpuartSrcMcgIrClk;

    LPUART_DRV_Init(0, (lpuart_state_t *)&lpuartState, (lpuart_user_config_t *)&lpuartUserConfig);
}

void disableLPUARTpins(void)
{
    /*
     *	LPUART deinit
     */
    LPUART_DRV_Deinit(0);

    /*
     *	Set UART pin association. See, e.g., page 99 in
     *
     *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
     *
     *	Setup:
     *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
     *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX

//TODO: we don't use the HW flow control and that messes with the SPI any way
 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

    // TODO: we don't use flow-control
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);

    /*
     *	Disable LPUART CLOCK
     */
    CLOCK_SYS_DisableLpuartClock(0);
}

WarpStatus
sendBytesToUART(uint8_t *bytes, size_t nbytes)
{
    lpuart_status_t status;

    status = LPUART_DRV_SendDataBlocking(0, bytes, nbytes, gWarpUartTimeoutMilliseconds);
    if (status != 0)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

void warpEnableSPIpins(void)
{
    CLOCK_SYS_EnableSpiClock(0);

    /*	kWarpPinSPI_MISO_UART_RTS_UART_RTS --> PTA6 (ALT3)	*/
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

    /*	kWarpPinSPI_MOSI_UART_CTS --> PTA7 (ALT3)	*/
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);

    /*
     *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
     */
    uint32_t calculatedBaudRate;
    spiUserConfig.polarity = kSpiClockPolarity_ActiveHigh;
    spiUserConfig.phase = kSpiClockPhase_FirstEdge;
    spiUserConfig.direction = kSpiMsbFirst;
    spiUserConfig.bitsPerSec = gWarpSpiBaudRateKbps * 1000;
    SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
    SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}

void warpDisableSPIpins(void)
{
    SPI_DRV_MasterDeinit(0);

    /*	kWarpPinSPI_MISO_UART_RTS	--> PTA6	(GPI)		*/
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

    /*	kWarpPinSPI_MOSI_UART_CTS	--> PTA7	(GPIO)		*/
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

    // TODO: we don't use HW flow control so can remove these since we don't use the RTS/CTS
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

    CLOCK_SYS_DisableSpiClock(0);
}

void warpDeasserAllSPIchipSelects(void)
{
    /*
     *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
     *
     *	Drive all chip selects high to disable them. Individual drivers call this routine before
     *	appropriately asserting their respective chip selects.
     *
     *	Setup:
     *		PTA12/kWarpPinISL23415_SPI_nCS	for GPIO
     *		PTA9/kWarpPinAT45DB_SPI_nCS	for GPIO
     *		PTA8/kWarpPinADXL362_SPI_nCS	for GPIO
     *		PTB1/kWarpPinFPGA_nCS		for GPIO
     *
     *		On Glaux
            PTB2/kGlauxPinFlash_SPI_nCS for GPIO
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
}

void debugPrintSPIsinkBuffer(void)
{
    for (int i = 0; i < kWarpMemoryCommonSpiBufferBytes; i++)
    {
        warpPrint("\tgWarpSpiCommonSinkBuffer[%d] = [0x%02X]\n", i, gWarpSpiCommonSinkBuffer[i]);
    }
    warpPrint("\n");
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

void warpDisableI2Cpins(void)
{
    I2C_DRV_MasterDeinit(0 /* I2C instance */);

    /*
     *	Setup:
     *
     *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	disabled
     *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	disabled
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

    CLOCK_SYS_DisableI2cClock(0);
}

void lowPowerPinStates(void)
{
    /*
     *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
     *	we configure all pins as output and set them to a known state. We choose
     *	to set them all to '0' since it happens that the devices we want to keep
     *	deactivated (SI4705) also need '0'.
     */

    /*
     *			PORT A
     */
    /*
     *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

    /*
     *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
     *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
     *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
     *	functionality.
     *
     *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

    /*
     *	Disable PTA5
     *
     *	NOTE: Enabling this significantly increases current draw
     *	(from ~180uA to ~4mA) and we don't need the RTC on revC.
     *
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

    /*
     *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
     *
     *		"Unused pins should be configured in the disabled state, mux(0),
     *		to prevent unwanted leakage (potentially caused by floating inputs)."
     *
     *	However, other documents advice to place pin as GPIO and drive low or high.
     *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

    /*
     *	NOTE: The KL03 has no PTA10 or PTA11
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);

    /*
     *			PORT B
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);
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

void warpDisableSupplyVoltage(void)
{
#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
    disableTPS62740();

    /*
     *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
     */
    OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
#endif
}

void warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
    WarpStatus status = kWarpStatusOK;

    /*
     *	Set all pins into low-power states. We don't just disable all pins,
     *	as the various devices hanging off will be left in higher power draw
     *	state. And manuals say set pins to output to reduce power.
     */
    if (forceAllPinsIntoLowPowerState)
    {
        lowPowerPinStates();
    }

    warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
    if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
    {
        warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* sleep seconds : irrelevant here */)() failed...\n");
    }

    status = warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
    if (status != kWarpStatusOK)
    {
        warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPS, 0 /* sleep seconds : irrelevant here */)() failed...\n");
    }
}

/*
void
printPinDirections(void)
{
    warpPrint("I2C0_SDA:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SDA_UART_RX));
    OSA_TimeDelay(100);
    warpPrint("I2C0_SCL:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SCL_UART_TX));
    OSA_TimeDelay(100);
    warpPrint("SPI_MOSI:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MOSI_UART_CTS));
    OSA_TimeDelay(100);
    warpPrint("SPI_MISO:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MISO_UART_RTS));
    OSA_TimeDelay(100);
    warpPrint("SPI_SCK_I2C_PULLUP_EN:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_SCK_I2C_PULLUP_EN));
    OSA_TimeDelay(100);
    warpPrint("ADXL362_CS:%d\n", GPIO_DRV_GetPinDir(kWarpPinADXL362_CS));
    OSA_TimeDelay(100);
}
*/

void dumpProcessorState(void)
{
    uint32_t cpuClockFrequency;

    CLOCK_SYS_GetFreq(kCoreClock, &cpuClockFrequency);
    warpPrint("\r\n\n\tCPU @ %u KHz\n", (cpuClockFrequency / 1000));
    warpPrint("\r\tCPU power mode: %u\n", POWER_SYS_GetCurrentMode());
    warpPrint("\r\tCPU clock manager configuration: %u\n", CLOCK_SYS_GetCurrentConfiguration());
    warpPrint("\r\tRTC clock: %d\n", CLOCK_SYS_GetRtcGateCmd(0));
    warpPrint("\r\tSPI clock: %d\n", CLOCK_SYS_GetSpiGateCmd(0));
    warpPrint("\r\tI2C clock: %d\n", CLOCK_SYS_GetI2cGateCmd(0));
    warpPrint("\r\tLPUART clock: %d\n", CLOCK_SYS_GetLpuartGateCmd(0));
    warpPrint("\r\tPORT A clock: %d\n", CLOCK_SYS_GetPortGateCmd(0));
    warpPrint("\r\tPORT B clock: %d\n", CLOCK_SYS_GetPortGateCmd(1));
    warpPrint("\r\tFTF clock: %d\n", CLOCK_SYS_GetFtfGateCmd(0));
    warpPrint("\r\tADC clock: %d\n", CLOCK_SYS_GetAdcGateCmd(0));
    warpPrint("\r\tCMP clock: %d\n", CLOCK_SYS_GetCmpGateCmd(0));
    warpPrint("\r\tVREF clock: %d\n", CLOCK_SYS_GetVrefGateCmd(0));
    warpPrint("\r\tTPM clock: %d\n", CLOCK_SYS_GetTpmGateCmd(0));
}

void printBootSplash(uint16_t gWarpCurrentSupplyVoltage, uint8_t menuRegisterAddress, WarpPowerManagerCallbackStructure *powerManagerCallbackStructure)
{
    /*
     *	We break up the prints with small delays to allow us to use small RTT print
     *	buffers without overrunning them when at max CPU speed.
     */
    warpPrint("\r\n\n\n\n[ *\t\t\t\tWarp (HW revision C) / Glaux (HW revision B)\t\t\t* ]\n");
    warpPrint("\r[  \t\t\t\t      Cambridge / Physcomplab   \t\t\t\t  ]\n\n");
    warpPrint("\r\tSupply=%dmV,\tDefault Target Read Register=0x%02x\n",
              gWarpCurrentSupplyVoltage, menuRegisterAddress);
    warpPrint("\r\tI2C=%dkb/s,\tSPI=%dkb/s,\tUART=%db/s,\tI2C Pull-Up=%d\n\n",
              gWarpI2cBaudRateKbps, gWarpSpiBaudRateKbps, gWarpUartBaudRateBps);
    warpPrint("\r\tSIM->SCGC6=0x%02x\t\tRTC->SR=0x%02x\t\tRTC->TSR=0x%02x\n", SIM->SCGC6, RTC->SR, RTC->TSR);
    warpPrint("\r\tMCG_C1=0x%02x\t\t\tMCG_C2=0x%02x\t\tMCG_S=0x%02x\n", MCG_C1, MCG_C2, MCG_S);
    warpPrint("\r\tMCG_SC=0x%02x\t\t\tMCG_MC=0x%02x\t\tOSC_CR=0x%02x\n", MCG_SC, MCG_MC, OSC_CR);
    warpPrint("\r\tSMC_PMPROT=0x%02x\t\t\tSMC_PMCTRL=0x%02x\t\tSCB->SCR=0x%02x\n", SMC_PMPROT, SMC_PMCTRL, SCB->SCR);
    warpPrint("\r\tPMC_REGSC=0x%02x\t\t\tSIM_SCGC4=0x%02x\tRTC->TPR=0x%02x\n\n", PMC_REGSC, SIM_SCGC4, RTC->TPR);
    warpPrint("\r\t%ds in RTC Handler to-date,\t%d Pmgr Errors\n", gWarpSleeptimeSeconds, powerManagerCallbackStructure->errorCount);
}

void blinkLED(int pin)
{
    GPIO_DRV_SetPinOutput(pin);
    OSA_TimeDelay(200);
    GPIO_DRV_ClearPinOutput(pin);
    OSA_TimeDelay(200);

    return;
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

int warpWaitKey(void)
{
    /*
     *	SEGGER'S implementation assumes the result of result of
     *	SEGGER_RTT_GetKey() is an int, so we play along.
     */
    int rttKey, bleChar = kWarpMiscMarkerForAbsentByte;

    do
    {
        rttKey = SEGGER_RTT_GetKey();

        /*
         *	NOTE: We ignore all chars on BLE except '0'-'9', 'a'-'z'/'A'-Z'
         */
        if (!(bleChar > 'a' && bleChar < 'z') && !(bleChar > 'A' && bleChar < 'Z') && !(bleChar > '0' && bleChar < '9'))
        {
            bleChar = kWarpMiscMarkerForAbsentByte;
        }
    } while ((rttKey < 0) && (bleChar == kWarpMiscMarkerForAbsentByte));

    return rttKey;
}

int main(void)
{
    while (1)
    {
    }
}

int char2int(int character)
{
    if (character >= '0' && character <= '9')
    {
        return character - '0';
    }

    if (character >= 'a' && character <= 'f')
    {
        return character - 'a' + 10;
    }

    if (character >= 'A' && character <= 'F')
    {
        return character - 'A' + 10;
    }

    return 0;
}

uint8_t
readHexByte(void)
{
    uint8_t topNybble, bottomNybble;

    topNybble = warpWaitKey();
    bottomNybble = warpWaitKey();

    return (char2int(topNybble) << 4) + char2int(bottomNybble);
}

int read4digits(void)
{
    uint8_t digit1, digit2, digit3, digit4;

    digit1 = warpWaitKey();
    digit2 = warpWaitKey();
    digit3 = warpWaitKey();
    digit4 = warpWaitKey();

    return (digit1 - '0') * 1000 + (digit2 - '0') * 100 + (digit3 - '0') * 10 + (digit4 - '0');
}

WarpStatus
writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte)
{
    i2c_status_t status;
    uint8_t commandBuffer[1];
    uint8_t payloadBuffer[1];
    i2c_device_t i2cSlaveConfig =
        {
            .address = i2cAddress,
            .baudRate_kbps = gWarpI2cBaudRateKbps};

    commandBuffer[0] = commandByte;
    payloadBuffer[0] = payloadByte;

    status = I2C_DRV_MasterSendDataBlocking(
        0 /* instance */,
        &i2cSlaveConfig,
        commandBuffer,
        (sendCommandByte ? 1 : 0),
        payloadBuffer,
        (sendPayloadByte ? 1 : 0),
        gWarpI2cTimeoutMilliseconds);

    return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}

WarpStatus
writeBytesToSpi(uint8_t *payloadBytes, int payloadLength)
{
    uint8_t inBuffer[payloadLength];
    spi_status_t status;

    warpEnableSPIpins();
    status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
                                            NULL /* spi_master_user_config_t */,
                                            payloadBytes,
                                            inBuffer,
                                            payloadLength /* transfer size */,
                                            gWarpSpiTimeoutMicroseconds /* timeout in microseconds (unlike I2C which is ms) */);
    warpDisableSPIpins();

    return (status == kStatus_SPI_Success ? kWarpStatusOK : kWarpStatusCommsError);
}