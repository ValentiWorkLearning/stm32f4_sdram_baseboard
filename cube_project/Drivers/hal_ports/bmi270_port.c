#include "bmi270.h"

#include "stm32f4xx_hal.h"
#include "bmi2_defs.h"
#include <stdio.h>
#include <string.h>

/*! Macro that defines read write length */
#define READ_WRITE_LEN     UINT8_C(32)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;
static I2C_HandleTypeDef* active_i2c_handle;

uint8_t tx_buffer[512], GRXBuffer[2048];

BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t device_addr_temp = dev_addr << 1;

	// send register address
	HAL_I2C_Master_Transmit(active_i2c_handle, device_addr_temp, &reg_addr, 1, 1000);
	HAL_I2C_Master_Receive(active_i2c_handle, device_addr_temp, reg_data, len, 1000);
	return 0;
}

BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t device_addr_temp = dev_addr << 1;

    tx_buffer[0] = reg_addr;
	memcpy(&tx_buffer[1], reg_data, len);
    HAL_I2C_Master_Transmit(active_i2c_handle, device_addr_temp, tx_buffer, len+1, 1000);
    return 0;
}

/*!
 * Delay function map to COINES platform
 */
void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    HAL_Delay(10);
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
int8_t bmi2_interface_init(struct bmi2_dev *bmi, uint8_t intf)
{
    int8_t rslt = BMI2_OK;
    if(bmi== NULL){
        return BMI2_E_NULL_PTR;
    }

        /* Bus configuration : I2C */
        if (intf == BMI2_I2C_INTF)
        {
            printf("I2C Interface \n");

            /* To initialize the user I2C function */
            dev_addr = BMI2_I2C_PRIM_ADDR;
            bmi->intf = BMI2_I2C_INTF;
            bmi->read = bmi2_i2c_read;
            bmi->write = bmi2_i2c_write;
        }
        /* Bus configuration : SPI */

        /* Assign device address to interface pointer */
        bmi->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        bmi->delay_us = bmi2_delay_us;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bmi->read_write_len = READ_WRITE_LEN;

        /* Assign to NULL to load the default config file. */
        bmi->config_file_ptr = NULL;

    return rslt;

}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi2_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
        case BMI2_OK:

            /* Do nothing */
            break;

        case BMI2_W_FIFO_EMPTY:
            printf("Warning [%d] : FIFO empty\r\n", rslt);
            break;
        case BMI2_W_PARTIAL_READ:
            printf("Warning [%d] : FIFO partial read\r\n", rslt);
            break;
        case BMI2_E_NULL_PTR:
            printf(
                "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
                rslt);
            break;

        case BMI2_E_COM_FAIL:
            printf(
                "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
                rslt);
            break;

        case BMI2_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMI2_E_INVALID_SENSOR:
            printf(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
                rslt);
            break;

        case BMI2_E_SELF_TEST_FAIL:
            printf(
                "Error [%d] : Self-test failed error. It occurs when the validation of accel self-test data is " "not satisfied\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_INT_PIN:
            printf(
                "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
                rslt);
            break;

        case BMI2_E_OUT_OF_RANGE:
            printf(
                "Error [%d] : Out of range error. It occurs when the data exceeds from filtered or unfiltered data from " "fifo and also when the range exceeds the maximum range for accel and gyro while performing FOC\r\n",
                rslt);
            break;

        case BMI2_E_ACC_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x40\r\n",
                rslt);
            break;

        case BMI2_E_GYRO_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x42\r\n",
                rslt);
            break;

        case BMI2_E_ACC_GYR_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Accel-Gyro configuration error. It occurs when there is a error in accel and gyro" " configuration registers which could be one among range, BW or filter performance in reg address 0x40 " "and 0x42\r\n",
                rslt);
            break;

        case BMI2_E_CONFIG_LOAD:
            printf(
                "Error [%d] : Configuration load error. It occurs when failure observed while loading the configuration " "into the sensor\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_PAGE:
            printf(
                "Error [%d] : Invalid page error. It occurs due to failure in writing the correct feature configuration " "from selected page\r\n",
                rslt);
            break;

        case BMI2_E_SET_APS_FAIL:
            printf(
                "Error [%d] : APS failure error. It occurs due to failure in write of advance power mode configuration " "register\r\n",
                rslt);
            break;

        case BMI2_E_AUX_INVALID_CFG:
            printf(
                "Error [%d] : Invalid AUX configuration error. It occurs when the auxiliary interface settings are not " "enabled properly\r\n",
                rslt);
            break;

        case BMI2_E_AUX_BUSY:
            printf(
                "Error [%d] : AUX busy error. It occurs when the auxiliary interface buses are engaged while configuring" " the AUX\r\n",
                rslt);
            break;

        case BMI2_E_REMAP_ERROR:
            printf(
                "Error [%d] : Remap error. It occurs due to failure in assigning the remap axes data for all the axes " "after change in axis position\r\n",
                rslt);
            break;

        case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
            printf(
                "Error [%d] : Gyro user gain update fail error. It occurs when the reading of user gain update status " "fails\r\n",
                rslt);
            break;

        case BMI2_E_SELF_TEST_NOT_DONE:
            printf(
                "Error [%d] : Self-test not done error. It occurs when the self-test process is ongoing or not " "completed\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_INPUT:
            printf("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
            break;

        case BMI2_E_INVALID_STATUS:
            printf("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
            break;

        case BMI2_E_CRT_ERROR:
            printf("Error [%d] : CRT error. It occurs when the CRT test has failed\r\n", rslt);
            break;

        case BMI2_E_ST_ALREADY_RUNNING:
            printf(
                "Error [%d] : Self-test already running error. It occurs when the self-test is already running and " "another has been initiated\r\n",
                rslt);
            break;

        case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
            printf(
                "Error [%d] : CRT ready for download fail abort error. It occurs when download in CRT fails due to wrong " "address location\r\n",
                rslt);
            break;

        case BMI2_E_DL_ERROR:
            printf(
                "Error [%d] : Download error. It occurs when write length exceeds that of the maximum burst length\r\n",
                rslt);
            break;

        case BMI2_E_PRECON_ERROR:
            printf(
                "Error [%d] : Pre-conditional error. It occurs when precondition to start the feature was not " "completed\r\n",
                rslt);
            break;

        case BMI2_E_ABORT_ERROR:
            printf("Error [%d] : Abort error. It occurs when the device was shaken during CRT test\r\n", rslt);
            break;

        case BMI2_E_WRITE_CYCLE_ONGOING:
            printf(
                "Error [%d] : Write cycle ongoing error. It occurs when the write cycle is already running and another " "has been initiated\r\n",
                rslt);
            break;

        case BMI2_E_ST_NOT_RUNING:
            printf(
                "Error [%d] : Self-test is not running error. It occurs when self-test running is disabled while it's " "running\r\n",
                rslt);
            break;

        case BMI2_E_DATA_RDY_INT_FAILED:
            printf(
                "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_FOC_POSITION:
            printf(
                "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
                rslt);
            break;

        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

void set_active_i2s_bus(I2C_HandleTypeDef* active_i2c)
{
    active_i2c_handle = active_i2c;
}
