F405_TARGETS    += $(TARGET)
FEATURES        += VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_spi_icm20689.c \
            drivers/pwm_output_stm32f4xx.c
