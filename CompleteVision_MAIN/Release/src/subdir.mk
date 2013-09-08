################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/CamObj.cpp \
../src/DEBUG__CompleteVision_Draft1__DEBUG.cpp \
../src/FPSCounter.cpp \
../src/KalmanFilter.cpp \
../src/PnPObj.cpp \
../src/Threshold.cpp \
../src/customblobdetector.cpp 

OBJS += \
./src/CamObj.o \
./src/DEBUG__CompleteVision_Draft1__DEBUG.o \
./src/FPSCounter.o \
./src/KalmanFilter.o \
./src/PnPObj.o \
./src/Threshold.o \
./src/customblobdetector.o 

CPP_DEPS += \
./src/CamObj.d \
./src/DEBUG__CompleteVision_Draft1__DEBUG.d \
./src/FPSCounter.d \
./src/KalmanFilter.d \
./src/PnPObj.d \
./src/Threshold.d \
./src/customblobdetector.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


