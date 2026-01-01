################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
E:/STM32N657X0_Q/ThreadX_Projects/TCP_H7_Test/Ping_Test_h7/Middlewares/ST/netxduo/common/drivers/ethernet/lan8742/nx_stm32_phy_driver.c 

OBJS += \
./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.o 

C_DEPS += \
./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.o: E:/STM32N657X0_Q/ThreadX_Projects/TCP_H7_Test/Ping_Test_h7/Middlewares/ST/netxduo/common/drivers/ethernet/lan8742/nx_stm32_phy_driver.c Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -DTX_INCLUDE_USER_DEFINE_FILE -DNX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I../AZURE_RTOS/App -I../NetXDuo/App -I../NetXDuo/Target -I../../Drivers/BSP/Components/lan8742/ -I../../Middlewares/ST/netxduo/common/drivers/ethernet/ -I../../Middlewares/ST/netxduo/addons/dhcp/ -I../../Middlewares/ST/netxduo/common/inc/ -I../../Middlewares/ST/netxduo/ports/cortex_m7/gnu/inc/ -I../../Middlewares/ST/threadx/common/inc/ -I../../Middlewares/ST/threadx/ports/cortex_m7/gnu/inc/ -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/Interfaces/Network/Ethernet Phy Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Interfaces-2f-Network-2f-Ethernet-20-Phy-20-Interface-2f-LAN8742_Phy_Interface

clean-Middlewares-2f-Interfaces-2f-Network-2f-Ethernet-20-Phy-20-Interface-2f-LAN8742_Phy_Interface:
	-$(RM) ./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.cyclo ./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.d ./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.o ./Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/nx_stm32_phy_driver.su

.PHONY: clean-Middlewares-2f-Interfaces-2f-Network-2f-Ethernet-20-Phy-20-Interface-2f-LAN8742_Phy_Interface

