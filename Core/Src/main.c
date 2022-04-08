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
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"



#include "stdio.h"
#include "stdbool.h"


	// Addresses of AHB2 Registers
	#define RCC_AHB2RSTR	0x4002102C
	#define RCC_AHB2ENR		0x4002104C
	#define RCC_AHB2SMENR	0x4002106C

	// Addresses of GPIO Registers
	#define GPIOB_MODER 	0x48000400
	#define GPIOB_OTYPER	0x48000404
	#define GPIOB_OSPEEDR	0x48000408
	#define GPIOB_PUPDR		0x4800040C
	#define GPIOB_IDR		0x48000410
	#define GPIOB_ODR		0x48000414
	#define GPIOB_BSRR		0x48000418
	#define GPIOB_LCKR		0x4800041C
	#define GPIOB_AFRL		0x48000420
	#define GPIOB_AFRH		0x48000424
	#define GPIOB_BRR		0x48000428
	#define GPIOB_ASCR		0x4800042C

	#define LEDPIN  	(1 << 14)

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

static void MX_GPIO_Init(void);
void SystemClock_Config(void);
static void UART1_Init(void);
UART_HandleTypeDef huart1;


volatile bool singleClick = false;
volatile bool doubleClick = false;

HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // single/doubleclick handler
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
		singleClick = true;
		if (gap<1000){
			printf("\t Double Click\n");
			singleClick = false;
			doubleClick = true;
			time=-500;
		}

	}
}

int main(void)
{
	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	MX_GPIO_Init();

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();

	BSP_TSENSOR_Init();

	BSP_MAGNETO_Init();

	BSP_GYRO_Init();

	BSP_PSENSOR_Init();

	BSP_HSENSOR_Init();

	UART1_Init();

	//HAL_InitTick(2);//not to sure what the pirority to put this.

	//HAL_SetTickFreq(10);//i think....

	static bool warning = false;
	static int mode = 0;//0:stationary, 1:Launch,2:return
	static float speed[3] = {0.0f,0.0f,0.0f};
	static float posit[3] = {0.0f,0.0f,0.0f};
	static float angSpeed[3] = {0.0f,0.0f,0.0f};//mag,hori,vertical
	static float angPosit[3] = {0.0f,0.0f,0.0f};
	static int lastTime;
	lastTime = HAL_GetTick();
	static bool LEDon=true;// 0:off 1:on
	static bool LEDmode=false; //0:25% 1:75%
	static int LEDiter = 0;// for PWM
	const int PWM[4] = {0,1,1,1};
	volatile unsigned int *p = (unsigned int *)GPIOB_BSRR;
	static char trigger[50];
	static int updateCount = 0;
	static int buttonCount = 0;
	static int printCount = 0;
	static int blinkCount = 0;
	static int PWMCount = 0;
	static int warningCount = 0;



	float accel_data[3];
	float mag_data[3];
	float temp_data;
	float gyro_data[3];
	float pressure_data;
	float hum_data;


	//determine true magnetic North
	printf("Finding True magnetic North\n Please Do not Move the Board\n");
	//HAL_Delay(1000);
	int16_t mag_data_i16[3] = { 0 };			// array to store the x, y and z readings.
	BSP_MAGNETO_GetXYZ(mag_data_i16);		// read magnetometer
	// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
	mag_data[0] = (float)mag_data_i16[0] / 1.0f;//remeber to change the divisor to the correct value
	mag_data[1] = (float)mag_data_i16[1] / 1.0f;
	mag_data[2] = (float)mag_data_i16[2] / 1.0f;




	while (1){//change to 1 to activate
		//Data Update at very high hertz to maintain inertial guidance accuracy
		//TODO: find a way to account for changes to speed WRT Direction

		if(HAL_GetTick()-updateCount > 50){

			updateCount = HAL_GetTick();

			int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
			BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
			// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
			accel_data[0] = (float)accel_data_i16[0] / 981.0f;//change to m/s^2
			accel_data[1] = (float)accel_data_i16[1] / 981.0f;
			accel_data[2] = (float)accel_data_i16[2] / 981.0f;
			if(accel_data[0]*accel_data[0] + accel_data[1]*accel_data[1] +accel_data[2]*accel_data[2] > 900){
				//G-Limit = 30
				warning = true;
				strcpy(trigger,"G Limit Exceeded");
			}


			int16_t mag_data_i16[3] = { 0 };			// array to store the x, y and z readings.
			BSP_MAGNETO_GetXYZ(mag_data_i16);		// read magnetometer
			// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
			mag_data[0] = (float)mag_data_i16[0] / 1000.0f;//remeber to change the divisor to the correct value
			mag_data[1] = (float)mag_data_i16[1] / 1000.0f;
			mag_data[2] = (float)mag_data_i16[2] / 1000.0f;
			if(mag_data[0]*mag_data[0] + mag_data[1]*mag_data[1] +mag_data[2]*mag_data[2] > 100){
				//G-Limit = 30
				warning = true;
				strcpy(trigger,"Solar Flare(Magnetometer)");
			}


			temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor
			if(temp_data>100){
				warning = true;
				strcpy(trigger,"Overheat");
			}

			int16_t gyro_data_i16[3] = { 0 };			// array to store the x, y and z readings.
			BSP_GYRO_GetXYZ(gyro_data_i16);		// read accelerometer
			// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
			gyro_data[0] = ((float)gyro_data_i16[0]+630.0f) / 1000.0f;
			gyro_data[1] = ((float)gyro_data_i16[1]+280.0f) / 1000.0f;
			gyro_data[2] = ((float)gyro_data_i16[2]+140.0f) / 1000.0f;

			if(gyro_data[0]*gyro_data[0] + gyro_data[1]*gyro_data[1] +gyro_data[2]*gyro_data[2] > 10000){
				//spin limit??
				warning = true;
				strcpy(trigger,"Spinning...");
			}

			pressure_data = BSP_PSENSOR_ReadPressure()*100.0f;// read temperature sensor
			if(pressure_data<1000){
				warning = true;
				strcpy(trigger,"Air Leak");
			}

			hum_data = BSP_HSENSOR_ReadHumidity();
			if(hum_data>80){
				warning = true;
				strcpy(trigger,"Stop breathing on me, Yuck!");
			}

		}

		//Button Check at 0.5Hz

		if (HAL_GetTick()-buttonCount > 2000){
			buttonCount = HAL_GetTick();
			if (singleClick){
				warning = false;
				printf("reset warning\n");
				singleClick = false;
			}else if(doubleClick){
				//TODO: nextmode CountDown
				mode++;
				mode=mode%3;
				doubleClick = false;
				printf("nextmode\n");
			}
		}
		//LED PWM at 1Mhz
		if(LEDon && HAL_GetTick()-PWMCount >100){
			PWMCount = HAL_GetTick();
			LEDiter++;
			LEDiter= LEDiter%4;
			p = (unsigned int *)GPIOB_BSRR;
			if((mode == 1 && PWM[LEDiter]) || ~PWM[LEDiter]){
				*p |= (1 << 14);	// set BSRR's bit[14], set bit, to turn LD2 on

			}else if (mode == 2){
				*p &= ~((1 << 30) | (1 << 14));	// clear GPIO BSRR's bit[30] and bit[14]
				*p |= (1 << 30);	// set BSRR's bit[30], reset bit, to turn LD2 off
			}



		}
		//Warning blink at 2hz
		if(warning && HAL_GetTick()-warningCount >500){
			warningCount = HAL_GetTick();
			LEDon = ~LEDon;

		}

		//LED blink at 1hz
		if(mode !=0 && HAL_GetTick()-blinkCount >1000){
			blinkCount = HAL_GetTick();
			LEDon = ~LEDon;

		}

		//Main printing at 1Hz
		if (HAL_GetTick() -printCount > 1000){
			printCount = HAL_GetTick();
			if(warning){
				printf("%s\n",trigger);
				switch(mode){
					case 0:
						printf("Stationary mode:WARNING\n");
						break;
					case 1:
						printf("Launch mode:WARNING\n");
						break;
					case 2:
						printf("Return mode: WARNING\n");
						break;
				}


			}else{
				printf("Pressure: %f; Temp: %f;Humidity:%f \n ", pressure_data, temp_data, hum_data);

				printf("Gyro X: %f, Y: %f, Z: %f\n", gyro_data[0], gyro_data[1], gyro_data[2]);

				printf("Hx : %f, Hy : %f, Hz : %f\n", mag_data[0], mag_data[1], mag_data[2]);

				printf("Accel X : %f; Accel Y : %f; Accel Z : %f\n", accel_data[0], accel_data[1], accel_data[2]);

				printf("Temp:%f\n Humidity:%f\n",temp_data,hum_data);
			}
		}





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


static void UART1_Init(void) {
/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
__HAL_RCC_GPIOB_CLK_ENABLE();
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/* Configuring UART1 */
huart1.Instance = USART1;
huart1.Init.BaudRate = 115200;
huart1.Init.WordLength = UART_WORDLENGTH_8B;
huart1.Init.StopBits = UART_STOPBITS_1;
huart1.Init.Parity = UART_PARITY_NONE;
huart1.Init.Mode = UART_MODE_TX_RX;
huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart1.Init.OverSampling = UART_OVERSAMPLING_16;
huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
if(HAL_UART_Init(&huart1) != HAL_OK) {
   while(1);
}
}

