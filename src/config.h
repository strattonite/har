/*
	Authored 2021, Phillip Stanley-Marbell.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/

#define WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF 1
#define WARP_BUILD_BOOT_TO_CSVSTREAM 0
#define WARP_BUILD_BOOT_TO_VLPR 1
#define WARP_BUILD_DISABLE_SUPPLIES_BY_DEFAULT 0

#define ARM_MATH_CM0PLUS 1
#define OUTPUT_TIME 0
#define OUTPUT_FREQ 0
#define OUTPUT_POWER 0
#define OUTPUT_AXES 0
#define DEBUG_LED 1

/*
 *	NOTE: The choice of WARP_BUILD_ENABLE_GLAUX_VARIANT is defined via the Makefile build rules.
 *	The commented line below should never be uncommented. It is just here to stress a point that
 *	you shouldn't try to enable the Glaux build in this way.
 */
// #define		WARP_BUILD_ENABLE_GLAUX_VARIANT			0

/*
 *	Define this here to activate FRDMKL03-specific behavior. Unlike the Glaux
 *	build variant above, we currently require users wanting to build for the
 *	KL03 to manually set this here.
 */
#define WARP_BUILD_ENABLE_FRDMKL03 1

typedef enum
{
	/*
	 *	Speeds
	 */
	kWarpDefaultI2cBaudRateKbps = 200,
	kWarpDefaultUartBaudRateBps = 115200,
	kWarpDefaultSpiBaudRateKbps = 10000,

	/*
	 *	Times
	 */
	kWarpDefaultSleeptimeSeconds = 0,
	kWarpDefaultI2cTimeoutMilliseconds = 5,
	kWarpDefaultUartTimeoutMilliseconds = 1000,
	kWarpDefaultSpiTimeoutMicroseconds = 5,
	kWarpDefaultMenuPrintDelayMilliseconds = 10,
	kWarpDefaultSupplySettlingDelayMilliseconds = 1,

	/*
	 *	Sizes
	 */
	kWarpDefaultPrintBufferSizeBytes = 64,
	kWarpMemoryCommonSpiBufferBytes = 64,
	kWarpSizesI2cBufferBytes = 4,
	kWarpSizesSpiBufferBytes = 7,
	kWarpSizesUartBufferBytes = 8,
	kWarpSizesBME680CalibrationValuesCount = 41,

	/*
	 *	Voltages
	 */
	kWarpDefaultSupplyVoltageMillivolts = 1800,
	kWarpDefaultSupplyVoltageMillivoltsBGX = 3300,
	kWarpDefaultSupplyVoltageMillivoltsBMX055accel = 1800,
	kWarpDefaultSupplyVoltageMillivoltsBMX055gyro = 1800,
	kWarpDefaultSupplyVoltageMillivoltsBMX055mag = 1800,
	kWarpDefaultSupplyVoltageMillivoltsMMA8451Q = 1800,
	kWarpDefaultSupplyVoltageMillivoltsLPS25H = 1800,
	kWarpDefaultSupplyVoltageMillivoltsHDC1000 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsMAG3110 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsSI7021 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsL3GD20H = 1800,
	kWarpDefaultSupplyVoltageMillivoltsBME680 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsTCS34725 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsSI4705 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsCCS811 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsAMG8834 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsAS7262 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsAS7263 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsRV8803C7 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsADXL362 = 2400,
	kWarpDefaultSupplyVoltageMillivoltsIS25xP = 1800,
	kWarpDefaultSupplyVoltageMillivoltsISL23415 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsAT45DB = 1800,
	kWarpDefaultSupplyVoltageMillivoltsICE40 = 1800,
	kWarpDefaultSupplyVoltageMillivoltsINA219 = 3300,
} WarpDefaults;
