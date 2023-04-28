#include "bmi270.h"
#include "stm32f4xx_hal.h"

BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

void bmi2_delay_us(uint32_t period, void *intf_ptr);

int8_t bmi2_interface_init(struct bmi2_dev *bma, uint8_t intf);

void bmi2_error_codes_print_result(int8_t rslt);

void set_active_i2s_bus(I2C_HandleTypeDef* active_i2c);