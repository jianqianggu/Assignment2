  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"

#include "stdio.h"

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

static void MX_GPIO_Init(void);
void SystemClock_Config(void);

HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static int time=-500;
	int gap = 0;
	if(GPIO_Pin == BUTTON_EXTI13_Pin)
	{
		gap=-(time-HAL_GetTick());
		time=HAL_GetTick();

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		printf("Single Click. \n");
		printf("Gap:%d,time:%d\n",gap,time);
		if (gap<5){
			printf("\t Double Click\n");
			time=-500;
		}

	}
}

int main(void)
{


	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();

	BSP_TSENSOR_Init();

	BSP_MAGNETO_Init();

	BSP_GYRO_Init();

	BSP_PSENSOR_Init();

	MX_GPIO_Init();
	while (1)
	{
		float accel_data[3];
		int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
		// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
		accel_data[0] = (float)accel_data_i16[0] / 100.0f;
		accel_data[1] = (float)accel_data_i16[1] / 100.0f;
		accel_data[2] = (float)accel_data_i16[2] / 100.0f;

		float mag_data[3];
		int16_t mag_data_i16[3] = { 0 };			// array to store the x, y and z readings.
		BSP_MAGNETO_GetXYZ(mag_data_i16);		// read accelerometer
		// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
		mag_data[0] = (float)mag_data_i16[0] / 1.0f;//remeber to change the divisor to the correct value
		mag_data[1] = (float)mag_data_i16[1] / 1.0f;
		mag_data[2] = (float)mag_data_i16[2] / 1.0f;		

		float temp_data;
		temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor

		float gyro_data[3];
		int16_t gyro_data_i16[3] = { 0 };			// array to store the x, y and z readings.
		BSP_GYRO_GetXYZ(gyro_data_i16);		// read accelerometer
		// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
		gyro_data[0] = (float)gyro_data_i16[0] / 16.0f;
		gyro_data[1] = (float)gyro_data_i16[1] / 16.0f;
		gyro_data[2] = (float)gyro_data_i16[2] / 16.0f;

		float pressure_data;
		pressure_data = BSP_PSENSOR_ReadPressure();			// read temperature sensor

		
		printf("Pressure: %f; Temp: %f\n", pressure_data, temp_data);

		printf("Gyro X: %f, Y: %f, Z: %f\n", gyro_data[0], gyro_data[1], gyro_data[2]);

		printf("Hx : %f, Hy : %f, Hz : %f\n", mag_data[0], mag_data[1], mag_data[2]);

		printf("Accel X : %f; Accel Y : %f; Accel Z : %f\n", accel_data[0], accel_data[1], accel_data[2]);

		HAL_Delay(1000);	// read once a ~second.

	}

}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();	// Enable AHB2 Bus for GPIOB
	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC

	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET); // Reset the LED2_Pin as 0

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Configuration of LED2_Pin (GPIO-B Pin-14) as GPIO output
	GPIO_InitStruct.Pin = LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}
