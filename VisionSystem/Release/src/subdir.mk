################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/PnPObj.cpp \
../src/Threshold.cpp \
../src/VisionSystem.cpp 

OBJS += \
./src/PnPObj.o \
./src/Threshold.o \
./src/VisionSystem.o 

CPP_DEPS += \
./src/PnPObj.d \
./src/Threshold.d \
./src/VisionSystem.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


