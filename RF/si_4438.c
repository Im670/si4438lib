
/*platform*/
#include "stm8l15x.h"
#include <stdio.h>
#include <string.h>

#include "si_4438.h"
#include "Rfrx.h"
#include "radio_config.h"


/*frame length*/
#include "rf_proto.h"
#define RF_FIX_LEN    sizeof(rf_frame_t)

/*delay*/
#include "delay.h"
/*printf*/
#include "usart1.h"
/*spi*/
#include "spi1.h"


#define SPI_WRITE(x)  spi_write(x)
#define SPI_READ()    spi_read()

#define DELAY_MS(x)    delay_ms(x)
#define DEF_PRINTF     usart1_printf

//----------------------------platform-------------------------------


#define SDN_OUT()        GPIO_Init(GPIOB,GPIO_Pin_1,GPIO_Mode_Out_PP_High_Slow)
	
#define	SET_SDN_LOW()	 GPIO_ResetBits(GPIOB, GPIO_Pin_1)
#define SET_SDN_HIGH()   GPIO_SetBits(GPIOB, GPIO_Pin_1)

#define GET_IRQ_LEVEL()  //

typedef struct
{
	byte chipmaskrevison;
	byte part[2];
	byte build;
	byte id[2];
	byte customerid;
	byte romid;
}chipinfo;


static int si4438_waitCTS(void);
static void si4438_send_command(byte byteCount, byte* pData);
static int read_response(byte len, byte* buffer);
static int si4438_set_property(byte grp, byte num, byte firstaddr, byte *data);


byte gRF_send_completed = 0;
byte gRF_recv_completed = 0;


void si4438_init(void)
{	
	//SDNÅäÖÃ³ÉÊä³ö PC1  	
	SDN_OUT();
	
	SET_SDN_HIGH();
	DELAY_MS(1);
	SET_SDN_LOW();
	DELAY_MS(10);

}

void si4438_NIRQ_irq_enable(void)
{
	//IRQÅäÖÃ³ÉÖÐ¶ÏÊäÈë PC5
	//GPIO_Init(GPIOC,GPIO_Pin_5,GPIO_Mode_In_PU_IT);
	
	//EXTI_SetPinSensitivity(EXTI_Pin_5,EXTI_Trigger_Falling);
}
void si4438_power_off(void)
{
	SET_SDN_HIGH();
	DELAY_MS(1);
}

void si4438_power_on(void)
{
	SET_SDN_LOW();
	DELAY_MS(10);
}

int si4438_waitCTS(void)
{
	byte cts=0;
	unsigned int cnt=0;
	
	while(0xFF != cts)
	{
		SET_CS_LOW();
		SPI_WRITE(CMD_READ_CMD_BUFF);
		cts = SPI_READ();
		//DEF_PRINTF("%02x",cts);
		SET_CS_HIGH();
		if(cnt++>25000)
		{		
			DEF_PRINTF("wait for cts failed.\n");		
			return -1;
		}
		//delay(10);
	}

	return 0;

}


void spi_burst_write(byte len, const byte *data)
{
	byte i=0;
	SET_CS_LOW();
	for(i=0;i<len;i++)
	{
		SPI_WRITE(data[i]);
	}
	SET_CS_HIGH();
}

void si4438_send_command(byte byteCount, byte* pData)
{
	if(NULL == pData)
	{
		return;
	}
	si4438_waitCTS();
	//while(0 > si4438_waitCTS());
	spi_burst_write(byteCount, pData);
}


int read_response(byte len, byte* buffer)
{
  	byte i=0;
	byte ret=0;
	int errcnt=0;

	if(NULL == buffer)
	{
		return -1;
	}
	si4438_waitCTS();
	//while(0 > si4438_waitCTS());
	while(ret != 0xFF)
	{		
		SET_CS_LOW();
		
		SPI_WRITE(READ_CMD_BUFF);
	
		ret = SPI_READ();
		if(ret == 0xFF)
		{
			if (len) 
			{
				for(i = 0; i < len; i++ ) 
				{
					buffer[i] = SPI_READ();
				}
			}
			break;
		}
		else
		{
	
			DEF_PRINTF("read response err.\n");
		
			
			if(errcnt++ > 10)
			{
				break;
			}
		}
	}
	SET_CS_HIGH();
	if(errcnt > 10)
	{
		return -1;
	}
	if(ret != 0xFF)
	{
		return -1;
	}
	
	return 0;
}

		
void si4438_get_partinfo(void)
{
	byte rsp[8]={0};	
		
	rsp[0]=CMD_PART_INFO;
	si4438_send_command(1,rsp);

	read_response(8, rsp);		


#if 1
	DEF_PRINTF("++++++++++++++++++++++++++++++++++++\n");
	DEF_PRINTF("Device inform:\n");	
	
	if(rsp[0]==0x11)DEF_PRINTF("Mask Revision:%02x.\n",(u16)rsp[0]);
	
	
	
	DEF_PRINTF("Part Number:%02x%02x.\n",(u16)rsp[1],(u16)rsp[2]);

	DEF_PRINTF("Build:%02x.\n",(u16)rsp[3]);

	DEF_PRINTF("Chip ID:%02x%02x.\n",(u16)rsp[4],(u16)rsp[5]);

	DEF_PRINTF("Customer ID:%02x.\n",(u16)rsp[6]);
	
	DEF_PRINTF("Rom ID:%02x.\n",(u16)rsp[7]);
	
	DEF_PRINTF("Test:%02x.\n",(u16)0x11);
	DEF_PRINTF("++++++++++++++++++++++++++++++++++++\n");
#endif
}

void si4438_set_config(void)
{
	u16 iCnt;
	byte iData;
	
	const byte RegTbl[] = RADIO_CONFIGURATION_DATA_ARRAY;

	iData = 0xFF;
	iCnt = 0x00;
	
	
	while( iData != 0x00 ) 
	{
		iData = RegTbl[ iCnt ];
		if( iData == 0x00 ) 
		{
			break;
		}
		iCnt++;
		si4438_send_command(iData, (byte*)&RegTbl[iCnt]);		
		iCnt += iData;
	}
#ifdef DEF_PRINTF_ON
	DEF_PRINTF("set config finish.\n");
#endif
}


void si4438_get_int_status(byte PhClrPend, byte ModemClrPend, byte ChipClrPend,status_t *pstatus)
{
	byte buf[4];
	buf[0] = CMD_GET_INT_STATUS;						// Use interrupt status command
	buf[1] = PhClrPend;								// Clear PH_CLR_PEND
	buf[2] = ModemClrPend;							// Clear MODEM_CLR_PEND
	buf[3] = ChipClrPend;							// Clear CHIP_CLR_PEND
	si4438_send_command(4,buf);

	if(NULL !=pstatus)
	{
		read_response( 8, (byte*)pstatus );						// Make sure that CTS is ready then get the response
	}
}

void si4438_start_rx(byte iChannel, byte iCondition, u16 iRx_Len, byte iNext_State1, byte iNext_State2, byte iNext_State3)
{
	byte buf[8]={0};
	si4438_get_int_status(0,0,0,NULL);
#ifdef DEF_PRINTF_ON
	DEF_PRINTF("Rx Channel:%02d.\n",(u16)iChannel);
#endif
	buf[0] = CMD_START_RX;
	buf[1] = iChannel;
	buf[2] = iCondition;
	buf[3] = (byte)(iRx_Len >> 8);
	buf[4] = (byte)(iRx_Len);
	buf[5] = iNext_State1;
	buf[6] = iNext_State2;
	buf[7] = iNext_State3;
	si4438_send_command(0x08, buf);
	si4438_waitCTS();
	
}

void si4438_clear_fifo(byte fifo)
{
	byte buf[2]={0};
	buf[0]=CMD_FIFO_INFO;
	buf[1]=fifo;
	si4438_send_command(2,buf);
	read_response(2,buf);	
}

void write_tx_fifo(byte numBytes, byte* pTxData)
{
	byte buf[RF_FIX_LEN+1]={0};
	memset(buf,0,sizeof(buf));
	buf[0]=CMD_WRITE_TX_FIFO;
	memcpy(&buf[1],pTxData,numBytes);
	spi_burst_write(RF_FIX_LEN+1,buf);
}

int si4438_tx(byte iChannel, byte iCondition, byte iTx_Len, byte *pData, int timeout_ms)
{
	byte buf[5]={0};
	si4438_get_int_status(0,0,0,NULL);//clear pendings
	si4438_clear_fifo(TX_FIFO);//reset tx fifo
	write_tx_fifo(iTx_Len,pData);//write data;
	
	buf[0] = CMD_START_TX;
	buf[1] = iChannel;
	buf[2] = iCondition;
	buf[3] = (byte)(RF_FIX_LEN>> 8);
	buf[4] = (byte)(RF_FIX_LEN);

	si4438_send_command(5 , buf );

	if(0 != timeout_ms)
	{
		while(timeout_ms--)
		{
			if(gRF_send_completed)
			{
				gRF_send_completed = 0;
				return 0;
			}
			DELAY_MS(1);
		}

		return -1;
	}
	/**/
	while(0 == gRF_send_completed);
	gRF_send_completed = 0;
#ifdef DEF_PRINTF_ON
	DEF_PRINTF("tx complete.\n");
#endif
	return 0;
}


byte si4438_rx_data_len(void)
{
	byte buf[2]={0};
	buf[0]=CMD_FIFO_INFO;
	buf[1]=0;
	si4438_send_command(2,buf);
	read_response(2,buf);
	return buf[0];
}

void si4438_change_state(SI_RXTX_STATE_E state)
{
    byte buf[4]={0};
	buf[0] = CMD_CHANGE_STATE;
    buf[1] = state;

    si4438_send_command(2, buf);
}

void si4438_read_rx_fifo(byte numBytes, byte* pRxData)
{
	byte i=0;
	SET_CS_LOW();
	SPI_WRITE(CMD_READ_RX_FIFO);
	for( i = 0; i < numBytes; i++ )
	{
		pRxData[i] = SPI_READ();
	}
	SET_CS_HIGH();
}

int si4438_set_property(byte grp, byte num, byte firstaddr, byte *data)
{
	byte buf[32];
	if(NULL == data || num > 16)
	{
		return -1;
	}
	
	memset(buf, 0, sizeof(buf));
	buf[0] = CMD_SET_PROPERTY; 
	buf[1] = grp;
	buf[2] = num;
	buf[3] = firstaddr;
	memcpy(&buf[4],data, num);
	si4438_send_command(num+4,buf);
    return 0;
}




int si4438_get_gpio_cfg(byte* data)
{
	byte buf[8];
	if(NULL == data)
	{
		return -1;
	}
	memset(buf, 0, sizeof(buf));
	buf[0] = CMD_GPIO_PIN_CFG;
	si4438_send_command(8,buf);
	read_response(7,&buf[1]);
	memcpy(data,buf,8);
	return 0;
}

void si4438_set_gpio_cfg(SI_GPIO_PIN_E gpio,SI_GPIO_MODE_E setmode)
{
	byte buf[8];
	si4438_get_gpio_cfg(buf);
	buf[gpio] = setmode;
	si4438_send_command(8,buf);	
}

int si4438_setWUTLDCTimer(unsigned int  WUT_Timer_ms,unsigned int LDC_Timer_ms)
{
    unsigned int WUT_M;
	unsigned char LDC_M;
	float temp;
	unsigned char temp1;
	unsigned char temp3;
	unsigned char i;
	byte data[2]={0};

	temp = WUT_Timer_ms*32768;
	temp = temp/4000;
	WUT_M = (unsigned int)temp;

	temp = (temp - WUT_M)*10;   
	if(temp >=5)
	{
		WUT_M = WUT_M + 1;
	}

	temp1 = WUT_M;
	temp3 = WUT_M>>8;

	data[0]=temp3;
	data[1]=temp1;
	si4438_set_property(0x00,0x02,0x05,data);
	for(i = 0;i<200;i++){;}	                                  //¶ÌÔÝÑÓ³Ù

	temp = LDC_Timer_ms*32768;
	temp = temp/4000;
	LDC_M = (unsigned int)temp;

	temp = (temp - LDC_M)*10;
	if(temp >=5)
	{
		LDC_M = LDC_M + 1;
	}

	temp1 = LDC_M;
	data[0]=temp1;
	si4438_set_property(0x00,0x01,0x08,data);
	for(i = 0;i<200;i++){;}	                                  //¶ÌÔÝÑÓ³Ù

	return 0;
}

int si4438_enable_WUT_LDC(unsigned char enable_flag)
{
	unsigned char i;
	byte data[2]={0};

	if(enable_flag == 1)
	{
		//DEF_PRINTF("enable WUT LDC.\n");
		data[0]=0x42;
	    si4438_set_property(0x00,0x01,0x04,data);
	    for(i = 0;i<200;i++){;}	                                  //¶ÌÔÝÑÓ³Ù
	}
    else
	{
		data[0]=0x00;
	    si4438_set_property(0x00,0x01,0x04,data);
	    for(i = 0;i<200;i++){;}	                                  //¶ÌÔÝÑÓ³Ù
	}
//**********************************************************************************************
//**********************************************************************************************
	if(enable_flag == 1)
	{
		data[0]=0x41;
	    si4438_set_property(0x00,0x01,0x01,data);	//enable wut ldc
	    for(i = 0;i<200;i++){;}	                    

		data[0]=0x01;
	    si4438_set_property(0x01,0x01,0x03,data);	//enable chip interrupt(WUT interrupt)
	    for(i = 0;i<200;i++){;}	                    

		data[0]=0x04;
		si4438_set_property(0x12,0x01,0x0f,data);	//enable pn

		data[0]=0x20;
		si4438_set_property(0x00,0x01,0x07,data);	//change chip state to sleep after ldc
	}
	else
	{
		data[0]=0x00;
	    si4438_set_property(0x00,0x01,0x01,data);   
	    si4438_set_property(0x12,0x01,0x0f,data);	
		si4438_set_property(0x00,0x01,0x07,data);	
	}
	return 0;
}


/*
void set_gpio_clk_output(void)
{
    byte setdata = (u8)SI_DIVIDED_CLK_EN_ENABLE|(u8)SI_DIV_CLK_SEL_DIV_30|(u8)SI_CLK_32K_SEL_RC;
    //enable clk
    si4438_set_property(0x00,0x01,0x01,&setdata); 

    si4438_set_gpio_cfg(SI_GPIO_PIN_GPIO0,SI_GPIO_MODE_DIV_CLK);
}
*/

void si4438_gpio_set(SI_GPIO_PIN_E gpio,u8 bset)
{
	if(bset)
	{	
		si4438_set_gpio_cfg(gpio,SI_GPIO_MODE_DRIVE1);
	}
	else
	{
		si4438_set_gpio_cfg(gpio,SI_GPIO_MODE_DRIVE0);
	}
}

void si4438_NIRQ_INT_proc(void)
{	  
	status_t status;
	
	//uNIRQ_WakeUp_Flag = 1;
	memset(&status, 0, sizeof(status_t));
	si4438_get_int_status(0,0,0,&status);
	if((status.chip_pend&0x20) != 0)
	{
#ifdef PRINTF_ON	
		printf("\033[31mRX_FIFO over flow:%02x.\033[0m\n",status.chip_pend);
#endif
		si4438_clear_fifo(RX_FIFO); 		
	}

	if((status.pkthdl_pend&0x10) != 0)
	{			
		if(!gRF_recv_completed)
		{
			gRF_recv_completed = 1;
		}
		
	}		

	if((status.pkthdl_pend&0x20))
	{
		gRF_send_completed = 1;
#ifdef PRINTF_ON	
		printf("tx completed\n");
#endif
	}	
}
