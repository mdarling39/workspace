################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/CamObj.cpp \
../src/CompleteVision_MAIN.cpp \
../src/FPSCounter.cpp \
../src/KalmanFilter.cpp \
../src/PnPObj.cpp \
../src/Threshold.cpp \
../src/customblobdetector.cpp 

OBJS += \
./src/CamObj.o \
./src/CompleteVision_MAIN.o \
./src/FPSCounter.o \
./src/KalmanFilter.o \
./src/PnPObj.o \
./src/Threshold.o \
./src/customblobdetector.o 

CPP_DEPS += \
./src/CamObj.d \
./src/CompleteVision_MAIN.d \
./src/FPSCounter.d \
./src/KalmanFilter.d \
./src/PnPObj.d \
./src/Threshold.d \
./src/customblobdetector.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -m32 -I/opt/local/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

