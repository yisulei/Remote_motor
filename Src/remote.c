#include "remote.h"
#include "robo_base.h"
#include <stdlib.h>

extern uint8_t Tx_Data[8];
extern uint8_t RX_Data[8];
extern Speed_System Speed_Motor;
extern Pos_System Pos_Motor;
extern uint8_t count_time;
extern uint8_t count_time_max;
extern 	uint8_t x;
RC_Ctl_t RC_CtrlData={
	{1024,1024,1024,1024,2,2},
	{0},
	{0},
	1024,
	0,
};

void RemoteDataProcess(uint8_t *pData) 
{     	
	if(pData == 0)     
	{         
		return;     
	}          
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;      
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;    
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;     
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;          
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;     
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003); 
 
  RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);     
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);     
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);     
 
  RC_CtrlData.mouse.press_l = pData[12];     
	RC_CtrlData.mouse.press_r = pData[13];       
	RC_CtrlData.key.vh = ((int16_t)pData[14]);
	RC_CtrlData.key.vl = ((int16_t)pData[15]);      
	RC_CtrlData.SW = (uint16_t)((pData[17]<<8)|pData[16]);
	
	RC_CtrlData.update = 1;
}
void Uart_DMA_Process(UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,UART_RX_BUFFER* Uart_Rx,void(*DataProcessFunc)(uint8_t *pData))
{
	uint8_t this_frame_len = 0;
	
	if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))  
		{   
			__HAL_DMA_DISABLE(hdma_usart_rx);
			__HAL_UART_CLEAR_IDLEFLAG(huart);  
			
			this_frame_len = Uart_Rx->Length_Max - __HAL_DMA_GET_COUNTER(hdma_usart_rx);
			if(Uart_Rx->Buffer_Num)
			{
				Uart_Rx->Buffer_Num = 0;
				HAL_UART_Receive_DMA(huart, Uart_Rx->Buffer[0], Uart_Rx->Length_Max);
				if(this_frame_len == Uart_Rx->Length_Max)
					if(DataProcessFunc) DataProcessFunc(Uart_Rx->Buffer[1]);
			}
			else
			{
				Uart_Rx->Buffer_Num = 1;
				HAL_UART_Receive_DMA(huart, Uart_Rx->Buffer[1], Uart_Rx->Length_Max);
				if(this_frame_len == Uart_Rx->Length_Max)
					if(DataProcessFunc) DataProcessFunc(Uart_Rx->Buffer[0]);
			}
		}
}

void Usart_All_Init(void)
{
	
	Uart1_Rx.Buffer[0]=(uint8_t*)malloc(sizeof(uint8_t)*USART1_RX_LEN_MAX);
	Uart1_Rx.Buffer[1]=(uint8_t*)malloc(sizeof(uint8_t)*USART1_RX_LEN_MAX);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	Uart1_Rx.Buffer_Num = 0;
	Uart1_Rx.Length_Max=USART1_RX_LEN_MAX;
	HAL_UART_Receive_DMA(&huart1, Uart1_Rx.Buffer[0], USART1_RX_LEN_MAX);
}


void Remote_Control_Motor(void)
{
		Speed_Motor.Tar_Speed=Target;
}


void Rx_Info_Analysis_State(void)
{
	switch (RC_CtrlData.rc.s1)
	{
		case 1:Speed_Info_Analysis(&Speed_Motor.Info,RX_Data);
		break;
		case 3:Pos_Info_Analysis(&Pos_Motor.Info,RX_Data);
		break;
	}
}
void Tx_Analysis_State(void)
{
	Remote_Control_Motor();
	PID_Speed_Cal(&Speed_Motor,Tx_Data);
}
void Tx_Info_Analysis_State(void)
{
	switch (RC_CtrlData.rc.s1)
	{
		case 1:Tx_Analysis_State();
		break;
		case 3:PID_Pos_Cal(&Pos_Motor,Tx_Data);
		break;
		
	}
}
void Working_Dog()
{
	Tx_Info_Analysis_State();
	Send_To_Motor(&hcan1,Tx_Data);
	if (x==1)
	{
		PID_Init(&Speed_Motor.Speed_PID,0.5,0,0,5000,0,5000,5000);
		PID_Init(&Pos_Motor.Speed_PID,0.5,0,0,5000,0,5000,5000);
	  PID_Init(&Pos_Motor.Pos_PID,0.5,0,0,5000,0,5000,5000);
		x=0;
	}
}

void Missing_Dog()
{
	x=1;
	PID_Init(&Speed_Motor.Speed_PID,1,0,0,0,0,0,0);
	PID_Init(&Pos_Motor.Speed_PID,1,0,0,0,0,0,0);
	PID_Init(&Pos_Motor.Pos_PID,1,0,0,0,0,0,0);
	Speed_Motor.Info.Speed=0;
	Speed_Motor.Info.Electric=0;
	Pos_Motor.Info.Electric=0;
	Pos_Motor.Info.Speed=0;
	Tx_Info_Analysis_State();
//	count_time_max=100;
//enend_To_Motor(&hcan1,Tx_Data);
//	if (Speed_Motor.Info.Electric!=0)
//	{
//		PID_Init(&Speed_Motor.Speed_PID,0.5,0,0,5000,0,5000,5000);
//	}
//	if (Pos_Motor.Info.Electric!=0)
//	{
//		PID_Init(&Pos_Motor.Speed_PID,0.5,0,0,5000,0,5000,5000);
//	  PID_Init(&Pos_Motor.Pos_PID,0.5,0,0,5000,0,5000,5000);
//	}
		
}
void Feed_Dog()
{
	count_time=count_time_max;
}

void WatchDag_State_Judge()
{
	if(count_time==count_time_max-1)
		Working_Dog();
	else if(count_time<count_time_max-1)
	{
		Missing_Dog();
	}
}

