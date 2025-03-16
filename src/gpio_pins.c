#include <stdint.h>
#include <stdlib.h>
#include "config.h"
#include "gpio_pins.h"
#include "device/fsl_device_registers.h"

/*
 *	Here, we configure all pins that we ever use as general-purpose output.
 *
 *	(A) See Section 12.1.1 "GPIO instantiation information" of KL03 family reference, KL03P24M48SF0RM.pdf
 *	for the default state of pins, pull capability, etc.:
 *
 *		PTA0 : pulled down at reset
 *		PTA2 : pulled up at reset
 *		PTA1 / RESET_b : pulled up at reset
 *		PTB5 : pulled up at reset
 *
 *	(B) See Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf) for additional hints on pin setup for low power.
 *
 *	(C) On Glaux, PTA3, PTA4, PTA5, PTA8, PTA12, PTB5, and PTB13 are
 *	either sacrifical or input so we don't configure them as GPIO.
 *
 *	**NOTE 1**:	The semantics is that pins that are excluded are disabled (TODO: double check).
 *
 *	**NOTE 2**:	Empirically, adding entries for pins which we want to leave disabled on Glaux
 *			(e.g., sacrificial pins) leads to higher power dissipation.
 *
 */

gpio_output_pin_user_config_t outputPins[] = {
	{
		.pinName = kGpioLED1,
		.config.outputLogic = 0,
		.config.slewRate = kPortFastSlewRate,
		.config.driveStrength = kPortHighDriveStrength,
	},
	{
		.pinName = kWarpPinTPS62740_REGCTRL,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS62740_VSEL4,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS62740_VSEL3,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS62740_VSEL2,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kWarpPinTPS62740_VSEL1,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = GPIO_PINS_OUT_OF_RANGE,
	}};

/*
 *	Configuration to be passed to GPIO_DRV_Init() to disable all pins.
 *
 *	NOTE: the type here is
 *
 *			gpio_input_pin_user_config_t
 *	not
 *
 *			gpio_output_pin_user_config_t
 *
 *	like the above.
 *
 *	On Warp (but not Glaux), PTB1 is tied to VBATT. Need to configure it as an input pin.
 *	On Glaux, PTA0 (SWD_CLK) is also used as the RTC interrupt line since it is also the
 *	LLWU_P7 source for the low-leakage wakeup unit.
 *
 *	NOTE: The semantics is that pins that are excluded are disabled (TODO: double check).
 */
gpio_input_pin_user_config_t inputPins[] = {
	{
		.pinName = kWarpPinMMA8451QInt2,
		.config.isPullEnable = true,
		.config.pullSelect = kPortPullUp,
		.config.isPassiveFilterEnabled = false,
		.config.interrupt = kPortIntFallingEdge,
	},
	{
		.pinName = GPIO_PINS_OUT_OF_RANGE,
	}};