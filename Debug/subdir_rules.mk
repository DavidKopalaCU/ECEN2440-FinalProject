################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
ADC14.obj: /Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/inc/ADC14.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/ccsv8/ccs_base/arm/include" --include_path="/Applications/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/FinalMaze" --include_path="MazeDrivers" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="ADC14.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

BumpSwitch.obj: /Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/MazeDrivers/BumpSwitch.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/ccsv8/ccs_base/arm/include" --include_path="/Applications/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/FinalMaze" --include_path="MazeDrivers" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="BumpSwitch.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

DriveSystem.obj: /Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/MazeDrivers/DriveSystem.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/ccsv8/ccs_base/arm/include" --include_path="/Applications/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/FinalMaze" --include_path="MazeDrivers" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="DriveSystem.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

LidarSensors.obj: /Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/MazeDrivers/LidarSensors.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/ccsv8/ccs_base/arm/include" --include_path="/Applications/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/FinalMaze" --include_path="MazeDrivers" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="LidarSensors.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

LineSensors.obj: /Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/MazeDrivers/LineSensors.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/ccsv8/ccs_base/arm/include" --include_path="/Applications/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/FinalMaze" --include_path="MazeDrivers" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="LineSensors.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

MotorEncoder.obj: /Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/MazeDrivers/MotorEncoder.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/ccsv8/ccs_base/arm/include" --include_path="/Applications/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/FinalMaze" --include_path="MazeDrivers" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="MotorEncoder.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

PWM.obj: /Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/inc/PWM.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/ccsv8/ccs_base/arm/include" --include_path="/Applications/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/FinalMaze" --include_path="MazeDrivers" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="PWM.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

SysTick.obj: /Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/inc/SysTick.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/ccsv8/ccs_base/arm/include" --include_path="/Applications/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/FinalMaze" --include_path="MazeDrivers" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="SysTick.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

main.obj: ../main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/ccsv8/ccs_base/arm/include" --include_path="/Applications/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/FinalMaze" --include_path="MazeDrivers" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="main.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_msp432p401r_ccs.obj: ../startup_msp432p401r_ccs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/ccsv8/ccs_base/arm/include" --include_path="/Applications/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/FinalMaze" --include_path="MazeDrivers" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="startup_msp432p401r_ccs.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

system_msp432p401r.obj: ../system_msp432p401r.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/ccsv8/ccs_base/arm/include" --include_path="/Applications/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/Users/davidkopala/Documents/CUBoulder/Fall2018/ECEN2440-EmbeddedLab/FinalMaze" --include_path="MazeDrivers" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="system_msp432p401r.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


