
# Human Activity Recognition (HAR)
##### Tristan Coetzee, Clare college, tc611
This project implements a HAR algorithm running on the [FRDM-KLO3](https://www.nxp.com/design/design-center/development-boards-and-designs/general-purpose-mcus/freedom-development-platform-for-kinetis-kl03-mcus:FRDM-KL03Z), the board is assumed to be held in the user's hand, although the algorithm can easily be modified for arbitrary placement. An emphasis is placed on power efficiency, the project achieves very low power usage as a result of placing the core in sleep (VLPW) mode between reading MMA8451Q FIFO buffer every 5.12 seconds.

# Layout
- src/ contains c source files
- sdk/ contains the Kinesis SDK V1.1.0 
- extern/ contains git submodules CMSIS_5 and CMSIS-DSP
- data/ contains data collected using algo-dev.ipynb
- build/ contains build files and artifacts
## Note:
- file setup.conf is required with the following format, these variables should also be in the user's environment:

JLINKPATH	=	PATH_TO_JLINK

ARMGCC_DIR	=	PATH_TO_ARMGCC_DIR

# Implementation

This repository is based off [Warp-firmware](https://github.com/physical-computation/Warp-firmware) <cite>A Hardware Platform for Efficient Multi-Modal Sensing with Adaptive Approximation”. ArXiv e-prints (2018). arXiv:1804.09241</cite>

Source files included without modification:
- src/devSSD1331.h
- src/errstrs.h
- src/SEGGER_RTT_Conf.h
- src/SEGGER_RTT.h
- src/errstrsEN.c
- src/SEGGER_RTT_printf.c
- src/SEGGER_RTT.c
- src/startup_MKLO3Z4.S

Source files modified from Warp-firmware:
- src/boot.c (don't use Warp menu, removed unused functions, implement HAR algorithm)
- src/warp.h (remove unused function declarations)
- src/CMakeLists.txt (add CMSIS-DSP and CMSIS dependencies)
- src/config.h (macro defines)
- src/devMMA8451Q.c (define ```configureMMA8451Q```, ```readFIFO```)
- src/devMMA8451Q.h (declare ```configureMMA8451Q```, ```readFIFO```)
- src/gpio_pins.c (modify ```outputPins```, ```inputPins```)
- src/gpio_pins.h (remove ```wakeupPins```)

New files:
- src/inference.c 
- src/inference.h
- algo-dev.ipynb
- warp.jlink.commands renamed -> har.jlink.commands

### Output
The algorithm predicts the user's activity every 10.24 seconds as one of the following classes: 

- stationary (board is completely stationary)
- sitting 
- walking 
- jogging

The output is sent over RTT in json (newline delimited), macros OUTPUT_TIME, OUTPUT_FREQ, and OUTPUT_POWER (config.h) turn on/off logging of power mode, time series data, and fft coefficients. Predictions are always logged with the following fields:

- type -> "PRED"
- group -> series identifier, incremented every prediction
- jogging/sitting/stationary/walking -> class prediction confidence * 2^10 as unsigned 16 bit integer

### Collecting data
algo-dev.ipynb is used to generate the parameters used for prediction. Function ```rrt_to_file(filename, num_seconds)``` captures RTT output of time series and fft coefficients and redirects to a file (see data/ for collected samples). The notebook walks through fitting a naive bayes (gaussian) classifier to the fft coefficients of accelerometer samples, and evaluates the accuracy on the training set data with a confusion matrix. $\mu$ and $\sigma$ are then output for each class and used in src/inference.c to generate predictions. This process can be easily repeated with new data to add classes, or change how the device is located w.r.t. the user. When running to generate data, src/config.h should be modified to turn on OUTPUT_TIME and OUTPUT_FREQ, and turn off OUTPUT_POWER. Turning on DEBUG_LED is useful for debugging. 

### Code structure
The board is initialized and placed in VLPW mode in src/boot.c ```main```, the FIFO buffer on the MMA8451Q accelerometer is used to buffer 32 samples at a 6.25 Hz sampling rate (see src/devMMA8451Q.c ```configureMMA8451Q```). Every 5.12 seconds the FIFO buffer interrupt is asserted on the MMA8451Q, ```PORTA_IRQHandler``` is called and the core is woken from VLPW. In src/boot.c ```main```, data is read from the FIFO buffer, the 3 axes of acceleration are combined into a single magnitude per reading, and the ```samples``` buffer is shifted and updated. Every 2 iterations (64 samples) 
```arm_cfft_q15``` computes the FFT of the sampled magnitudes, ```arm_cmplx_mag_q15``` calculates the magnitude of FFT coefficients, and ```predict``` generates predictions for the 4 classes. 

### inference.c 
The implementation of the naive-bayes classifier in inference.c uses parameters calculated in algo-dev.ipynb (*_MU[7], *_SIGMA[7]), and a lookup table for the standard normal distribution, also output from algo-dev.ipynb. ```predict``` downsamples the FFT coefficients and performs the scaling and lookup of standard normal values. Defining the lookup table and distribution parameters as consts allows these to be stored in flash, which is important since we only have 2kb SRAM. For performance reasons, calculations throughout the algorithm are almost entirely performed on integers, with floats only being used for the final normalization of probabilities, and scaling of acceleration readings. 