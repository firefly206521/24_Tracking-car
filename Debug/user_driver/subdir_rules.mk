################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
user_driver/%.o: ../user_driver/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/CCS_20.5.0.00028_win/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"E:/control_Practice/05_MOTOR _backup2" -I"E:/control_Practice/05_MOTOR _backup2/Debug" -I"D:/CCS_20.5.0.00028_win/ccs2050/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/CCS_20.5.0.00028_win/ccs2050/mspm0_sdk_2_10_00_04/source" -I"E:/control_Practice/05_MOTOR _backup2/user_driver" -gdwarf-3 -Wall -MMD -MP -MF"user_driver/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


