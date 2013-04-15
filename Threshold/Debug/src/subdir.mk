################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Threshold.cpp 

OBJS += \
./src/Threshold.o 

CPP_DEPS += \
./src/Threshold.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/Applications/Xcode.app/Contents/Developer/usr/llvm-gcc-4.2/lib/gcc/i686-apple-darwin11/4.2.1/include -I/Library/Frameworks -I/opt/local/include -I/System/Library/Frameworks -I/usr/include -I/usr/include/c++/4.2.1 -I/usr/include/c++/4.2.1/backward -I/usr/llvm-gcc-4.2/lib/gcc/i686-apple-darwin11/4.2.1/include -I/usr/local/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


