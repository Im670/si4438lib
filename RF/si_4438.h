#ifndef __SPI_4438_H
#define __SPI_4438_H

//#include "Prohead.h"

typedef struct
{
	byte irq_pend;	
	byte irq_status; 
	byte pkthdl_pend;
	byte pkthdl_status;
	byte mod_pend;
	byte mod_status;
	byte chip_pend;
	byte chip_status;	
}status_t;

typedef enum
{
	SI_RXTX_MODE_NOCHANGE = 0,
	SI_RXTX_MODE_SLEEP = 1,
	SI_RXTX_MODE_SPI_ACTIVE = 2,
	SI_RXTX_MODE_READY = 3,
	SI_RXTX_MODE_TX_TUNE = 5,
	SI_RXTX_MODE_RX_TUNE = 6,
	SI_RXTX_MODE_TX = 7,
	SI_RXTX_MODE_RX = 8
}SI_RXTX_STATE_E;

#define SI_DIVIDED_CLK_SEL_SBIT   3
#define SI_DIVIDED_CLK_EN_SBIT    6
#define SI_PULL_CTL_SBIT          6

typedef enum
{
	SI_DIV_CLK_SEL_DIV_1	= (0 << SI_DIVIDED_CLK_SEL_SBIT),	//Clock output is system clock divided by 1.	
	SI_DIV_CLK_SEL_DIV_2	= (1 << SI_DIVIDED_CLK_SEL_SBIT),	//Clock output is system clock divided by 2.	
	SI_DIV_CLK_SEL_DIV_3	= (2 << SI_DIVIDED_CLK_SEL_SBIT),	//Clock output is system clock divided by 3.	
	SI_DIV_CLK_SEL_DIV_7_5	= (3 << SI_DIVIDED_CLK_SEL_SBIT),	//Clock output is system clock divided by 7.5.	
	SI_DIV_CLK_SEL_DIV_10	= (4 << SI_DIVIDED_CLK_SEL_SBIT),	//Clock output is system clock divided by 10.	
	SI_DIV_CLK_SEL_DIV_15	= (5 << SI_DIVIDED_CLK_SEL_SBIT),	//Clock output is system clock divided by 15.	
	SI_DIV_CLK_SEL_DIV_30	= (6 << SI_DIVIDED_CLK_SEL_SBIT)		//Clock output is system clock divided by 30.
}
SI_DIV_CLK_SEL_E;

typedef enum
{
    SI_CLK_32K_SEL_OFF	= 0,	//32 kHz clock is disabled.	
    SI_CLK_32K_SEL_RC	= 1,	//32 kHz clock is driven by internal RC oscillator.	
    SI_CLK_32K_SEL_CRYSTAL	= 2	//32 kHz clock is driven by internal crystal oscillator operating with external 32K crystal blank across GPIO0 and GPIO1 pins.
}
SI_CLK_32K_SEL_E;

typedef enum
{
    SI_DIVIDED_CLK_EN_DISABLE	= (0 << SI_DIVIDED_CLK_EN_SBIT),    //Divided system clock output is disabled.	
    SI_DIVIDED_CLK_EN_ENABLE	= (1 << SI_DIVIDED_CLK_EN_SBIT)    //Divided system clock output is enabled.
}
SI_DIVIDED_CLK_EN_E;

typedef enum
{
    SI_PULL_CTL_PULL_DIS    = (0 << SI_PULL_CTL_SBIT),	//Disable pull-up resistor (recommended setting if the pin is driven from an external source, other than an open-drain source).	revB1A
    SI_PULL_CTL_PULL_EN	    = (1 << SI_PULL_CTL_SBIT)   //Enable pull-up resistor.
}
SI_PULL_CTL_E;

typedef enum
{    
    SI_GPIO_MODE_DONOTHING           = 0 ,  //Behavior of this pin is not modified.   revB1A
    SI_GPIO_MODE_TRISTATE            = 1 ,  //Input and output drivers disabled.  revB1A
    SI_GPIO_MODE_DRIVE0              = 2 ,   //Pin is configured as a CMOS output and driven low.  revB1A
    SI_GPIO_MODE_DRIVE1              = 3 ,    //Pin is configured as a CMOS output and driven high. revB1A
    SI_GPIO_MODE_INPUT               = 4 ,  //Pin is configured as a CMOS input. This is used for all GPIO functions that require the pin to be an input (e.g., TXDATA input for TX Direct Mode). However, configuration of this pin as an input does NOT additionally select which internal circuit receives that input; that functionality is controlled by other properties, as appropriate.   revB1A
    SI_GPIO_MODE_32K_CLK             = 5 ,  //Outputs 32 kHz clock selected using GLOBAL_CLK_CFG:CLK_32K_SEL. Output low if the 32 kHz clock is not enabled.  revB1A
    SI_GPIO_MODE_BOOT_CLK            = 6 ,  //Outputs the boot clock signal. This signal will only be present when the chip is in the SPI_ACTIVE state as that is the only state in which the boot clock is active.   revB1A
    SI_GPIO_MODE_DIV_CLK             = 7 ,   //Outputs the divided clock signal (or the divided boot clock signal in SPI ACTIVE state). This output is low while the chip is in SLEEP state as the source (e.g., the Xtal Oscillator) for the divided clock signal is not running, and outputs the divided XtalOsc signal in all other states. The divider is configured using the GLOBAL_CLK_CFG:DIVIDED_CLK_SEL. revB1A
    SI_GPIO_MODE_CTS                 = 8 ,  //Clear To Send signal. This output goes high when the command handler is able to receive a new command, and is low otherwise.    revB1A
    SI_GPIO_MODE_INV_CTS             = 9 ,   //Inverted Clear To Send signal. This output goes low when clear to send a new command, and is high otherwise.    revB1A
    SI_GPIO_MODE_CMD_OVERLAP         = 10, //This output is low unless a command overlap occurs (i.e., another command is sent before the command handler completes processing a previous command). When command overlap occurs, this output goes high until the rising edge of CTS. revB1A
    SI_GPIO_MODE_SDO                 = 11,  //Outputs the Serial Data Out (SDO) signal for the SPI bus.   revB1A
    SI_GPIO_MODE_POR                 = 12,  //This output goes low during Power-On Reset and goes high upon completion of POR.    revB1A
    SI_GPIO_MODE_CAL_WUT             = 13,  //This output is normally low, and pulses high for one cycle of the 32 kHz clock upon expiration of the Calibration Timer. The 32 kHz clock must be enabled in order to use the Calibration Timer. The Calibration Timer period is configured using GLOBAL_WUT_CONFIG:WUT_CAL_PERIOD and enabled by GLOBAL_WUT_CONFIG:CAL_EN. revB1A
    SI_GPIO_MODE_WUT                 = 14,  //This output is normally low, and pulses high for 2(WUT_R+1) cycles of the 32 kHz clock upon expiration of the Wake-Up Timer (WUT). The 32 kHz clock must be enabled in order to use the WUT. The period of the WUT is configured using GLOBAL_WUT_M, and GLOBAL_WUT_R and enabled by GLOBAL_WUT_CONFIG:WUT_EN.  revB1A
    SI_GPIO_MODE_EN_PA               = 15,  //This output goes high when the internal PA is enabled.  revB1A
    SI_GPIO_MODE_TX_DATA_CLK         = 16,  //Outputs the TX Data Clock signal. This signal is a square wave at the selected TX data rate, and is intended for use in TX Direct Synchronous Mode (i.e., in conjunction with a pin configured for TX Data Input).  revB1A
    SI_GPIO_MODE_RX_DATA_CLK         = 17,  //Outputs the RX Data CLK signal. This signal is nominally a square wave that is synchronized to the received data rate, and is typically used to latch the RX Data signal into the host MCU. revB1A
    SI_GPIO_MODE_EN_LNA              = 18,  //This output goes low when the internal LNA is enabled.  revB1A
    SI_GPIO_MODE_TX_DATA             = 19,  //Outputs the TX data bits pulled from the TX FIFO and sent to the TX modulator. This is an output signal (primarily for diagnostic purposes) and is NOT used as an input for TX Direct Sync/Async mode.  revB1A
    SI_GPIO_MODE_RX_DATA             = 20,  //Outputs the demodulated RX Data stream, after synchronization and re-timing by the local RX Data Clock. revB1A
    SI_GPIO_MODE_RX_RAW_DATA         = 21,  //Outputs the demodulated RX Raw Data stream, prior to synchronization and re-timing by the local RX Data Clock.  revB1A
    SI_GPIO_MODE_ANTENNA_1_SW        = 22,  //Antenna-1 Switch signal used for control of an RF switch during Antenna Diversity operation. This signal normally assumes the complementary polarity of the Antenna-2 Switch signal (except during SLEEP state).    revB1A
    SI_GPIO_MODE_ANTENNA_2_SW        = 23,  //Antenna-2 Switch signal used for control of an RF switch during Antenna Diversity operation. This signal normally assumes the complementary polarity of the Antenna-1 Switch signal (except during SLEEP state).    revB1A
    SI_GPIO_MODE_VALID_PREAMBLE      = 24,  //This output goes high when a valid preamble is detected, and returns low after the packet is received or Sync Word timeout occurs.  revB1A
    SI_GPIO_MODE_INVALID_PREAMBLE    = 25,  //Output low normally, pulses output high when the preamble is not detected within a period time (determined by PREAMBLE_CONFIG_STD_2:RX_PREAMBLE_TIMEOUT) after the demodulator is enabled.  revB1A
    SI_GPIO_MODE_SYNC_WORD_DETECT    = 26,  //This output goes high when a Sync Word is detected, and returns low after the packet is received.   revB1A
    SI_GPIO_MODE_CCA                 = 27,  //Clear Channel Assessment. This output goes high when the Current RSSI signal exceeds the threshold value set by the MODEM_RSSI_THRESH property, and is low when the Current RSSI is below threshold. This is a real-time (non-latched) signal.  revB1A
    SI_GPIO_MODE_IN_SLEEP            = 28,  //This output goes high when the chip is NOT in SLEEP state, and goes low when in SLEEP state.    revB1A
    SI_GPIO_MODE_TX_RX_DATA_CLK      = 31,  //Outputs TX or RX data CLK to be used in conjunction with TX or RX Data pin depending on the current power state.    
    SI_GPIO_MODE_TX_STATE            = 32,  //This output is set high while in TX state and is low otherwise. The TX_STATE and RX_STATE signals are typically used for control of peripheral circuits (e.g., a T/R Switch).   revB1A
    SI_GPIO_MODE_RX_STATE            = 33,  //This output is set high while in RX state and is low otherwise. The TX_STATE and RX_STATE signals are typically used for control of peripheral circuits (e.g., a T/R Switch).   revB1A
    SI_GPIO_MODE_RX_FIFO_FULL        = 34,  //This output is high while the number of bytes stored in the RX FIFO exceeds the threshold value set by the PKT_RX_THRESHOLD property, and is low otherwise. revB1A
    SI_GPIO_MODE_TX_FIFO_EMPTY       = 35,  //This output is high while the number of bytes of empty space in the TX FIFO exceeds the threshold value set by the PKT_TX_THRESHOLD property, and is low otherwise. revB1A
    SI_GPIO_MODE_LOW_BATT            = 36,  //This output is high while the battery voltage drops below the threshold value set by the GLOBAL_LOW_BATT_THRESH property, and is low otherwise. 
    SI_GPIO_MODE_CCA_LATCH           = 37,  //This output goes high if the Current RSSI signal exceeds the threshold value set by the MODEM_RSSI_THRESH property and remains high (i.e., is latched) even if the Current RSSI signal subsequently drops below the threshold value. The signal returns low upon detection of the Sync Word or upon exiting RX state.   revB1A
    SI_GPIO_MODE_HOPPED              = 38,  //This output toggles (i.e., switches from low to high, or high to low) whenever an automatic hop within the RX Hop Table occurs. This signal is not affected by a manual hop initiated through the RX_HOP command.   
    SI_GPIO_MODE_HOP_TABLE_WRAP      = 39  //This output toggles (i.e., switches from low to high, or high to low) whenever the automatic hop table wraps. This signal is not affected by a manual hop initiated through the RX_HOP command. 
}
SI_GPIO_MODE_E;


typedef enum
{
    SI_GPIO_PIN_GPIO0       = 0x01,
    SI_GPIO_PIN_GPIO1       = 0x02,     
    SI_GPIO_PIN_GPIO2       = 0x03,
    SI_GPIO_PIN_GPIO3       = 0x04,
    SI_GPIO_PIN_NIRQ        = 0x05,	
    SI_GPIO_PIN_SDO	        = 0x06,	
    SI_GPIO_PIN_GEN_CONFIG	= 0x07
}
SI_GPIO_PIN_E;



#define TX_FIFO 0x01
#define RX_FIFO 0x02


extern byte gRF_send_completed ;
extern byte gRF_recv_completed ;



void si4438_init(void);
void si4438_NIRQ_irq_enable(void);
void si4438_power_off(void);
void si4438_power_on(void);
void si4438_set_config(void);
void si4438_get_int_status(byte PhClrPend, byte ModemClrPend, byte ChipClrPend,status_t *pstatus);
void si4438_change_state(SI_RXTX_STATE_E state);
void si4438_clear_fifo(byte fifo);
void si4438_read_rx_fifo(byte numBytes, byte* pRxData);
byte si4438_rx_data_len(void);
void si4438_start_rx(byte iChannel, byte iCondition, u16 iRx_Len, byte iNext_State1, byte iNext_State2, byte iNext_State3);
int  si4438_tx(byte iChannel, byte iCondition, byte iTx_Len, byte *pData, int timeout_ms);
void si4438_get_partinfo(void);

int  si4438_setWUTLDCTimer(unsigned int  WUT_Timer_ms,unsigned int LDC_Timer_ms);
int  si4438_enable_WUT_LDC(unsigned char enable_flag);
void si4438_set_gpio_cfg(SI_GPIO_PIN_E gpio,SI_GPIO_MODE_E setmode);

void si4438_gpio_set(SI_GPIO_PIN_E gpio,u8 bset);
void si4438_NIRQ_INT_proc(void);

#endif
