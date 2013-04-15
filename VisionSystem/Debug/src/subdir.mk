################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/FPSCounter.cpp \
../src/PnPObj.cpp \
../src/Threshold.cpp \
../src/VisionSystem.cpp 

OBJS += \
./src/FPSCounter.o \
./src/PnPObj.o \
./src/Threshold.o \
./src/VisionSystem.o 

CPP_DEPS += \
./src/FPSCounter.d \
./src/PnPObj.d \
./src/Threshold.d \
./src/VisionSystem.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -m32 -I/opt/local/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


