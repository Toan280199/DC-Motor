#include "stm32f4xx.h"
#include "system_timetick.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"

#define DATA_SIZE_T 28
#define DATA_SIZE_R 23

int mm = 0;
float Ts = 0.01;  // Thoi gian lay mau

GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_ICInitTypeDef TIM_ICInitStructure;
USART_InitTypeDef USART_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;

void USART_Config(unsigned int BaudRate);
void NVIC_Config(void);
void Motor_Config(void);
void USART_Send_String(char *String);
void delayms(uint16_t ms); // delay 15  ms
void check_data(void);
void set_PWM(float duty);

char dataR[DATA_SIZE_R];
char dataT[DATA_SIZE_T];
char Calc_CRC_8(const char *DataArray, const uint16_t Length);

char opening;
char mode = 0x31, pre_mode;
float Kp;
float Ki;
float Kd;
float SetPoint, SetSpeed, set_duty;
char ending;

float RealPoint;
float error = 0, pre_error = 0, sum_error = 0;
float duty = 0;

uint32_t encoder_pulse = 0;
int32_t count_temp = -1, count_recent = 0;
float motor_speed = 0, motor_pos = 0, pre_motor_pos;

union ByteToFloat
{
  float myFloat;
  char myByte[4];
}myData;

//! CRC 8 lookup table
static const uint8_t CRC_8_TABLE[256] = 
{
	  0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
	157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
	 35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
	190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
	 70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
	219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
	101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
	248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
	140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
	 17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
	175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
	 50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
	202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
	 87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
	233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
	116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
};

int main(void){
		USART_Config(115200);
		NVIC_Config();
		Motor_Config();
  while (1)
  {
  }
}

void USART_Config(unsigned int BaudRate)   // + DMA
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //khung 8 bit
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //1 stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No; //no parity
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //vo hieu hoa dong dk phan cung
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); //cho phép uart

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	USART_Cmd(USART1, ENABLE);

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); 

/* DMA2 Stream7 Channel4 for UART1 Tx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dataT;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = DATA_SIZE_T;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;// khong tang dia chi ngoai vi
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // tang dia chi bo nho de gui uart
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //gui Normal, nhan Circular
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream7, ENABLE);

/* DMA2 Stream5 Channel4 for UART1 Rx configuration */
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dataR; 
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = DATA_SIZE_R;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;// khong tang dia chi ngoai vi
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // tang dia chi bo nho
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream5, &DMA_InitStructure);

  DMA_Cmd(DMA2_Stream5, ENABLE);
  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);

}

void Motor_Config(void)
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
		
		//Time base config
		TIM_TimeBaseStructure.TIM_Prescaler = 0;
		TIM_TimeBaseStructure.TIM_Period = 8399; // 65535
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_Pulse = 0;
//TIM_OCStructInit(&TIM_OCInitStructure);
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM1, ENABLE);
/* TIM1 enable counter */
		TIM_Cmd(TIM1, ENABLE);
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Prescaler = 0;
		TIM_TimeBaseStructure.TIM_Period = 8399; // 65535
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_Pulse = 0;
//TIM_OCStructInit(&TIM_OCInitStructure);
		TIM_OC1Init(TIM8, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM8, ENABLE);
/* TIM1 enable counter */
		TIM_Cmd(TIM8, ENABLE);
		TIM_CtrlPWMOutputs(TIM8, ENABLE);
	
	//Config for Encoder
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	/*Bat clock TIM3*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); /*Bat clock khoi A*/
	
	//Configure pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	//Configure Timer
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure );
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);	//thiet lap ngat khi tran bo nho  

//Debounce filter
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICPrescaler = 0;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
//Setup quadrature encoder and enable timer
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3,ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	/*Bat clock TIM4*/
	TIM_TimeBaseStructure.TIM_Prescaler=840-1;
	TIM_TimeBaseStructure.TIM_Period = 1000-1;/*so xung trong 1 chu ky*/
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;/* chon mode counter dem tang*/
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);/*thiet lap ngat khi tran bo nho co thong so TIM_IT_Update*/
	TIM_Cmd(TIM4, ENABLE);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_Cmd(TIM4,ENABLE);
}


void set_PWM(float duty)	//ham cap xung
{
	if(duty>0)
		{
			TIM1->CCR1=duty*8399/100;
			TIM8->CCR1=0*8399/100;
		}
	else	
		{
			TIM1->CCR1=0*8399/100;
			TIM8->CCR1=-duty*8399/100;
		}
}

void NVIC_Config(void)
{	
	  /* Configure the Priority Group to 2 bits */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

//  /* Enable the UART1 TX DMA Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
//	  /* Enable the UART1 RX DMA Interrupt */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		/* Enable the TIM3 (ENCODER) Interupt */
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);		
		
				/* Enable the TIM4 Interupt */
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);		
}

void USART_Send_String(char *String) // chuoi 8 byte
{
for(int y = 0 ; y <= 7 ; y++ )
{
		USART_SendData(USART1, String[y]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) != SET);
}
}

void DMA2_Stream5_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5) != RESET)
	{
		//Frame nhan A+mode+Kp+Ki+Kd+SetPoint+duty+B
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
		memcpy(&opening, dataR, 1);
		memcpy(&ending, dataR + 22, 1);
		if((opening == 'A') && (ending == 'B'))
		{
			memcpy(&mode, dataR + 1, 1);
			memcpy(&Kp, dataR + 2, 4);
			memcpy(&Ki, dataR + 6, 4);
			memcpy(&Kd, dataR + 10, 4);
			memcpy(&SetPoint, dataR + 14, 4);
			memcpy(&set_duty, dataR + 18, 4);
		}
		DMA_SetCurrDataCounter(DMA2_Stream5, DATA_SIZE_R);
		DMA_Cmd(DMA2_Stream5, ENABLE);
	}
}

void TIM3_IRQHandler(void)
{
		int32_t timer_temp;
		if (TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) == SET)
			{
				TIM_ClearFlag(TIM3, TIM_FLAG_Update);
				timer_temp = TIM_GetCounter(TIM3);
				if(timer_temp==65535) count_temp--;
				if(timer_temp==0) count_temp++;
			}
}
	
void TIM4_IRQHandler(void)
{
		if (TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) == SET)
			{
				TIM_ClearFlag(TIM4, TIM_FLAG_Update);
				//Cap nhat toc do
				encoder_pulse = TIM_GetCounter(TIM3)+65536*count_temp;
				count_recent=encoder_pulse;
				motor_pos = (float)(count_recent)/1336*360;
				motor_speed = (float)(motor_pos-pre_motor_pos)/360*100*60;
				pre_motor_pos = motor_pos;
				
				//Tinh PID va xuat xung PWM
				if (mode != pre_mode) sum_error = 0;
				if(mode == '1')	//mode PID
				{
					if(SetPoint-SetSpeed>10) SetSpeed += 20;
					else if(SetPoint-SetSpeed<-10) SetSpeed -= 20;
					else SetSpeed = SetPoint;
					error = SetSpeed-motor_speed;
					duty = Kp*error + Ki*Ts*sum_error + Kd*(error-pre_error)/Ts;
					if (duty >99) duty = 99;
					if (duty<-99) duty = -99;
					pre_error = error;
					sum_error += error;
				}
				else if (mode == '2')	//mode duty
				{
					if(set_duty-duty>0.005) duty += 0.2;
					else if(set_duty-duty<-0.005) duty -= 0.2;
					else duty = set_duty;
					error = 0;
				}
				else 
				{
					error = SetPoint-motor_pos;
					duty = Kp*error + Ki*Ts*sum_error + Kd*(error-pre_error)/Ts;
					if (duty >75) duty = 75;
					if (duty<-75) duty = -75;
					pre_error = error;
					sum_error += error;
				}
				set_PWM(duty);
				pre_mode = mode;
				mm ++;
				if (mm == 10)
					{
						DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);// xoa co
						DMA_Cmd(DMA2_Stream7, DISABLE);
						mm = 0;
						//Frame truyen : [Y][Reference][Data][Duty][Kp][Ki][Kd][E]
						dataT[0] = 'Y';
						memcpy(dataT + 1, &SetPoint, 4);
						if (mode == '3') memcpy(dataT + 5, &motor_pos, 4);
						else memcpy(dataT+5,&motor_speed,4);
						memcpy(dataT + 9, &duty, 4);
						memcpy(dataT + 13, &Kp, 4);
						memcpy(dataT + 17, &Ki, 4);
						memcpy(dataT + 21, &Kd, 4);
						dataT[25] = mode;
						dataT[26] = Calc_CRC_8(dataT,25);
						dataT[27] = '\n';
						DMA2_Stream7->NDTR = DATA_SIZE_T;	// so byte can truyen
						DMA_Cmd(DMA2_Stream7, ENABLE);		// cho phep truyen
						TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
					}
		}
}

char Calc_CRC_8(const char *DataArray, const uint16_t Length)
{
	uint8_t my_crc = 0;
 	for (int i=0; i<Length; i++)
		my_crc = CRC_8_TABLE[my_crc ^ DataArray[i]];
 
	return my_crc;
}
