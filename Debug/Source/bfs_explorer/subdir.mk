################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/bfs_explorer/frontier_search.cpp 

OBJS += \
./Source/bfs_explorer/frontier_search.o 

CPP_DEPS += \
./Source/bfs_explorer/frontier_search.d 


# Each subdirectory must supply rules for building sources it contributes
Source/bfs_explorer/%.o: ../Source/bfs_explorer/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/cybernik/Development/Projects/SeNaviCommon/Source" -O0 -g3 -Wall -c -fmessage-length=0 -std=gnu++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


