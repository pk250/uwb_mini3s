#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "spi.h"

#include "math.h"
#include "port.h"
#include "instance.h"
#include "deca_types.h"
#include "deca_spi.h"
#include "device_info.h" 
#include "stmflash.h"
#include "dma.h"
//#include <string.h>
#include "hw_config.h"
#include "deca_regs.h"


uint8 s1switch = 0;

u8 SendBuff[130];
u8 DMA_USART_Buff[130];
u8 USB_RxBuff[30];



/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[127];//设置缓冲区

static uint32 status_reg = 0;

#define MODULE_ROLE  	  0   //0是标签，1是基站
#define MODULE_CHANNEL	2		//2个频段  2 5


//接收休眠时间
#define SNIFF_ON_TIME 1
#define SNIFF_OFF_TIME 16

static dwt_config_t config = {
	MODULE_CHANNEL,  /* 信道. */
	DWT_PRF_16M,     /* 频率. */
	DWT_PLEN_1024,    /* 帧长. */
	DWT_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
	4,               /* TX preamble code. */
	4,               /* RX preamble code.  */
	1,               /* 0 to use standard SFD, 1 to use non-standard SFD.  */
	DWT_BR_110K,      /* 速率. */
	DWT_PHRMODE_STD, /* PHY header mode. */
	0,							//*DWT_BR_110K=0,DWT_BR_6M8=1*    nSmart Power enable / disable/
	(1025+64-32)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. 
										 64 for 110k (always)   8 for 6.81M*/
};


/* 天线延迟 */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* 发送包信息 */
static uint8 tx_poll_msg[] = {0x00, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x89, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
	
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts=0;
static uint64 resp_tx_ts=0;
static uint64 final_rx_ts=0;

static double tof;
static double distance,dist2;

#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
	

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 祍 and 1 祍 = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536


#define POLL_TX_TO_RESP_RX_DLY_UUS 150 //poll包发送后延迟接收response包时间

#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100//final包发送时间参数

#define RESP_RX_TIMEOUT_UUS 2700//respon包接收超时时间

#define POLL_RX_TO_RESP_TX_DLY_UUS 2600//response包发送时间参数

#define RESP_TX_TO_FINAL_RX_DLY_UUS 500//response包发送后延迟接收final包时间

#define FINAL_RX_TIMEOUT_UUS 3300//final包接收超时时间


#define SPEED_LIGHT 299702547  //光速 m/s
/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts=0;
static uint64 resp_rx_ts=0;
static uint64 final_tx_ts=0;
static uint32 Response_tx_time=0;
/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static double  RX_level = 0; //接收功率
static int  RX_level_C = 0;	 //0x12 CIR_PWR 接收功率参数
static int  RX_level_N = 0;  //0x10 RXPACC  接收功率参数
static float	RX_level_A=0;  //



uint32 inittestapplication(uint8 s1switch)
{
    uint32 devID ;
	  uint32 partID ;
		uint32 lotID ;
	  uint16 otprevision;
		uint32 ldo;

    int result;
		int x=0;
    SPI_ConfigFastRate(SPI_BaudRatePrescaler_32);  //max SPI before PLLs configured is ~4M

    /*读取DW1000芯片ID并判断*/
    devID = instancereaddeviceid() ;
    if(DWT_DEVICE_ID != devID) //如果读取设备ID失败，DW1000可能处于休眠状态
    {
        port_SPIx_clear_chip_select();  //CS拉低
        Sleep(1);   
        port_SPIx_set_chip_select();  //CS 拉高
        Sleep(7);
        devID = instancereaddeviceid() ;
        
        if(DWT_DEVICE_ID != devID)//再次判断ID
            return(-1) ;
        //清除睡眠位-这样在硬复位低于DW不进入睡眠
        dwt_softreset();
    }
		
		/*读取DW1000芯片ID*/
		partID=dwt_getpartid();
		lotID=dwt_getlotid();
		otprevision=dwt_otprevision();
		ldo=dwt_getldotune();
		/*打印ID值*/
		if(DWT_DEVICE_ID == devID)
		{
			x = sprintf((char*)&SendBuff[0], "devID=%8x,partID=%ld,lotID=%ld,vision=%x,ldo=%ld\r\n",DWT_DEVICE_ID,partID,lotID,otprevision,ldo);
			printf("%s",SendBuff);
			USB_TxWrite(SendBuff, x);
		}
		
    return devID;
}


int main(void)
{ 
		int i = 0;
		 
		dwt_txconfig_t txconfig;

		//delay_init();	    	 //延时函数初始化
		//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
		uart_init(115200);	 //串口初始化为115200 	
		GPIO_Configuration();//初始化与LED连接的硬件接口
		SPI_Configuration();//SPI初始化
		peripherals_init();

		USB_Config();//虚拟串口初始化
		MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)DMA_USART_Buff,130);//DMA1通道4,外设为串口1,存储器为SendBuff,长度130.  

		port_DisableEXT_IRQ(); //不使能IRQ

		/*错误指示*/
		if(inittestapplication(s1switch) == (uint32)-1)
		{
			led_on(LED_ALL); //to display error....
			return 0; //error
		}	

		reset_DW1000();//重启DW1000
		spi_set_rate_low();//降低SPI频率
		
		dwt_initialise(DWT_LOADUCODE | DWT_LOADLDOTUNE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM);//初始化DW1000
		
		spi_set_rate_high();//恢复SPI频率
		dwt_configure(&config, DWT_LOADXTALTRIM);//配置DW1000
		dwt_setrxantennadelay(RX_ANT_DLY);		//设置接收天线延迟
		dwt_settxantennadelay(TX_ANT_DLY);		//设置发射天线延迟

		/*控制指示灯闪烁*/
		i=50;
		while(i--)
		{
					if (i & 1) led_off(LED_ALL);
					else    led_on(LED_ALL);

					Sleep(50);
		}
		i = 0;	
		led_off(LED_ALL);
		
    port_EnableEXT_IRQ(); //使能IRQ
		if(config.prf== DWT_PRF_64M)
				RX_level_A = 121.74;
		if(config.prf== DWT_PRF_16M)
				RX_level_A = 113.77;
		
		/*设置发射功率*/	
		txconfig.PGdly=txSpectrumConfig[config.chan].PGdelay;
		txconfig.power = 0x39393939;   
		if(config.dataRate == DWT_BR_110K)
		{
				txconfig.power = (txconfig.power & 0xff) ;
				txconfig.power |= (txconfig.power << 8) + (txconfig.power << 16) + (txconfig.power << 24);
				dwt_setsmarttxpower(0);
		}
		else
		{
				dwt_setsmarttxpower(1);
		}			
		dwt_configuretxrf(&txconfig);
		
		while(1)
		{
			
			if(MODULE_ROLE == 0)
			{
				char count=0;
				dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS );//设置发送完成后开启接收延迟时间
				dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);	//设置接收超时时间
				while(1)
				{
										
					/*将Poll包数据写入DW1000，将在开启发送时传出去*/
					dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
					dwt_writetxfctrl(sizeof(tx_poll_msg), 0);
					
					dwt_setleds(1);//开启接收发送指示灯

					dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);//开启立即发送并等待接收response包
					

					while (!(status_reg=dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG| CLEAR_ALLRXERROR_EVENTS)))//不断查询寄存器直到接收成功
					{ };

					 if (status_reg & SYS_STATUS_RXFCG)//如果成功接收
					 {
							uint32 frame_len_T = 0;
							
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//清除发送接收标志位
							
							frame_len_T = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;	//获得接收到的数据长度

							dwt_readrxdata(rx_buffer, frame_len_T, 0);   //读取接收到的数据
					
							if (rx_buffer[9] == 0x10)//判断response包
							{
									uint32 final_tx_time;	
																	
									poll_tx_ts = get_tx_timestamp_u64();//poll时间戳
									resp_rx_ts = get_rx_timestamp_u64();//resp时间戳

									/*计算设置final包发送时间 */
									final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
									dwt_setdelayedtrxtime(final_tx_time);
								
									final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;//final时间戳是我们编程的传输时间加上TX天线延迟

									/* final包内数据设置 */
									final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
									final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
									final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);				
									
									/*final包发送*/
									dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); 
									dwt_writetxfctrl(sizeof(tx_final_msg), 0); 
									dwt_starttx(DWT_START_TX_DELAYED);		//开启延迟发送					
									
									while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))//不断查询状态直到发送成功
									{ };
							
									dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);//清楚发送标志位	
									tx_final_msg[2] = count;
									count++;

							}	
					}
					else
					{						
						dwt_write32bitreg(SYS_STATUS_ID, CLEAR_ALLRXERROR_EVENTS);//清除DW1000状态寄存器中的RX错误事件 
					}
						Sleep(100); //延时一段时间
				}
			}
			if(MODULE_ROLE == 1)
			{

				dwt_setsniffmode(1, SNIFF_ON_TIME, SNIFF_OFF_TIME);	
					
				while(1)
				{					
					int u=0;
					int ret_A=0;
					
					/*接收功率参数初始化*/
					dwt_rxdiag_t rxdiag_t;	
					uint32 D17F=pow(2,17);
					rxdiag_t.maxGrowthCIR=0;
					rxdiag_t.rxPreamCount=0;

					memset(rx_buffer,0,24);
					memset(SendBuff,0,130);
									
					dwt_setrxtimeout(0);//接收不超时
					dwt_rxenable(DWT_START_RX_IMMEDIATE);//开启立即接收
					
					while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | CLEAR_ALLRXERROR_EVENTS)))//不断查询寄存器直到接收成功
					{ };

					if (status_reg & SYS_STATUS_RXFCG)
					{
						uint32 frame_len_A = 0;
						
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);		//清除接收标志位				
						
						frame_len_A = dwt_read32bitreg(RX_FINFO_ID) &  RX_FINFO_RXFL_MASK_1023;//获得接收到的数据长度
						dwt_readrxdata(rx_buffer, frame_len_A, 0);//读取接收数据
						
						if(rx_buffer[9] == 0x21)//判断poll包
						{		
							poll_rx_ts = get_rx_timestamp_u64();//获得Poll包接收时间T2					
							Response_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;//计算Response发送时间T3。
							dwt_setdelayedtrxtime(Response_tx_time);//设置Response发送时间T3							
							dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);//设置发送完成后开启接收延迟时间
							dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);//接收超时时间
												 						
							
							/*response包发送*/
							dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); 
							dwt_writetxfctrl(sizeof(tx_resp_msg), 0);
							
							ret_A= dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED );	//开启延迟发送并等待接收final包

							if (ret_A == DWT_ERROR)//判断是否出现错误
							{
								printf("3");
									continue;							
							}
							while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | CLEAR_ALLRXERROR_EVENTS)))//不断查询寄存器直到接收成功
							{ };
							if (status_reg & SYS_STATUS_RXFCG)
							{								
								dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//清楚接收发送标志位
							
								frame_len_A = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;//获得接收到的数据长度

								dwt_readrxdata(rx_buffer, frame_len_A, 0);//读取接收数据
							
								if(rx_buffer[9] == 0x23 )//判断final包
								{
									uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
									uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
									double Ra, Rb, Da, Db;
									int64 tof_dtu;
								
									resp_tx_ts = get_tx_timestamp_u64();//获得response发送时间T3
									final_rx_ts = get_rx_timestamp_u64();//获得final接收时间T6
									
									final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);//从接收数据中读取T1，T4，T5
									final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
									final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

									/* 将读取到的时间转换为32进制数. */
									poll_rx_ts_32 = (uint32)poll_rx_ts;
									resp_tx_ts_32 = (uint32)resp_tx_ts;
									final_rx_ts_32 = (uint32)final_rx_ts;
									Ra = (double)(resp_rx_ts - poll_tx_ts);//Tround1 = T4 - T1  
									Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);//Tround2 = T6 - T3 
									Da = (double)(final_tx_ts - resp_rx_ts);//Treply2 = T5 - T4  
									Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);//Treply1 = T3 - T2  
									tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));//计算公式	
									
									tof = tof_dtu * DWT_TIME_UNITS;
									distance = tof * ((float)SPEED_OF_LIGHT);	//计算距离									
									if(distance < 0)
										distance = 0;
									/* 读取接收功率参数. */
									dwt_readdignostics(&rxdiag_t);
									RX_level_C=(int)rxdiag_t.maxGrowthCIR;
									RX_level_N=(int)rxdiag_t.rxPreamCount;
									
									/* 接收功率计算. */
									RX_level=RX_level_C*D17F;
									RX_level=RX_level/(RX_level_N*RX_level_N);
									RX_level=10*log10(RX_level);
									RX_level=RX_level-RX_level_A;

									u+=sprintf((char *)&SendBuff[0],"  NUMBER: %d POWER:%lf dBM DIS:%3.2fm\r\n\r\n",(rx_buffer[2]-'0')+48,RX_level,distance);

									USB_TxWrite(SendBuff, u);	
									printf("%s",SendBuff);
									dwt_setleds(1);/* 开启W1000接收发送指示灯 */
							}
						}
						else
						{
								/*清除DW1000状态寄存器中的RX错误事件. */
								dwt_write32bitreg(SYS_STATUS_ID, CLEAR_ALLRXERROR_EVENTS);
								/* 关闭DW1000接收发送指示灯 */
								dwt_setleds(0);
								/* 重启接收器 */
								dwt_rxreset();
						}			
					}			
				}
				else
				{
						/*清除DW1000状态寄存器中的RX错误事件. */
						dwt_write32bitreg(SYS_STATUS_ID, CLEAR_ALLRXERROR_EVENTS);
						/* 重启接收器 */
						dwt_rxreset();
				}	
		}		
	 }		
 }
}

/*! ------------------------------------------------------------------------------------------------------------------
		获取TX时间戳
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
		获取RX时间戳
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}
/*! ------------------------------------------------------------------------------------------------------------------
		final包数据设置
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
		final包数据读取
 */
 static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
