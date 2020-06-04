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
static uint8 rx_buffer[127];//���û�����

static uint32 status_reg = 0;

#define MODULE_ROLE  	  0   //0�Ǳ�ǩ��1�ǻ�վ
#define MODULE_CHANNEL	2		//2��Ƶ��  2 5


//��������ʱ��
#define SNIFF_ON_TIME 1
#define SNIFF_OFF_TIME 16

static dwt_config_t config = {
	MODULE_CHANNEL,  /* �ŵ�. */
	DWT_PRF_16M,     /* Ƶ��. */
	DWT_PLEN_1024,    /* ֡��. */
	DWT_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
	4,               /* TX preamble code. */
	4,               /* RX preamble code.  */
	1,               /* 0 to use standard SFD, 1 to use non-standard SFD.  */
	DWT_BR_110K,      /* ����. */
	DWT_PHRMODE_STD, /* PHY header mode. */
	0,							//*DWT_BR_110K=0,DWT_BR_6M8=1*    nSmart Power enable / disable/
	(1025+64-32)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. 
										 64 for 110k (always)   8 for 6.81M*/
};


/* �����ӳ� */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* ���Ͱ���Ϣ */
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
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536


#define POLL_TX_TO_RESP_RX_DLY_UUS 150 //poll�����ͺ��ӳٽ���response��ʱ��

#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100//final������ʱ�����

#define RESP_RX_TIMEOUT_UUS 2700//respon�����ճ�ʱʱ��

#define POLL_RX_TO_RESP_TX_DLY_UUS 2600//response������ʱ�����

#define RESP_TX_TO_FINAL_RX_DLY_UUS 500//response�����ͺ��ӳٽ���final��ʱ��

#define FINAL_RX_TIMEOUT_UUS 3300//final�����ճ�ʱʱ��


#define SPEED_LIGHT 299702547  //���� m/s
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
static double  RX_level = 0; //���չ���
static int  RX_level_C = 0;	 //0x12 CIR_PWR ���չ��ʲ���
static int  RX_level_N = 0;  //0x10 RXPACC  ���չ��ʲ���
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

    /*��ȡDW1000оƬID���ж�*/
    devID = instancereaddeviceid() ;
    if(DWT_DEVICE_ID != devID) //�����ȡ�豸IDʧ�ܣ�DW1000���ܴ�������״̬
    {
        port_SPIx_clear_chip_select();  //CS����
        Sleep(1);   
        port_SPIx_set_chip_select();  //CS ����
        Sleep(7);
        devID = instancereaddeviceid() ;
        
        if(DWT_DEVICE_ID != devID)//�ٴ��ж�ID
            return(-1) ;
        //���˯��λ-������Ӳ��λ����DW������˯��
        dwt_softreset();
    }
		
		/*��ȡDW1000оƬID*/
		partID=dwt_getpartid();
		lotID=dwt_getlotid();
		otprevision=dwt_otprevision();
		ldo=dwt_getldotune();
		/*��ӡIDֵ*/
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

		//delay_init();	    	 //��ʱ������ʼ��
		//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
		uart_init(115200);	 //���ڳ�ʼ��Ϊ115200 	
		GPIO_Configuration();//��ʼ����LED���ӵ�Ӳ���ӿ�
		SPI_Configuration();//SPI��ʼ��
		peripherals_init();

		USB_Config();//���⴮�ڳ�ʼ��
		MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)DMA_USART_Buff,130);//DMA1ͨ��4,����Ϊ����1,�洢��ΪSendBuff,����130.  

		port_DisableEXT_IRQ(); //��ʹ��IRQ

		/*����ָʾ*/
		if(inittestapplication(s1switch) == (uint32)-1)
		{
			led_on(LED_ALL); //to display error....
			return 0; //error
		}	

		reset_DW1000();//����DW1000
		spi_set_rate_low();//����SPIƵ��
		
		dwt_initialise(DWT_LOADUCODE | DWT_LOADLDOTUNE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM);//��ʼ��DW1000
		
		spi_set_rate_high();//�ָ�SPIƵ��
		dwt_configure(&config, DWT_LOADXTALTRIM);//����DW1000
		dwt_setrxantennadelay(RX_ANT_DLY);		//���ý��������ӳ�
		dwt_settxantennadelay(TX_ANT_DLY);		//���÷��������ӳ�

		/*����ָʾ����˸*/
		i=50;
		while(i--)
		{
					if (i & 1) led_off(LED_ALL);
					else    led_on(LED_ALL);

					Sleep(50);
		}
		i = 0;	
		led_off(LED_ALL);
		
    port_EnableEXT_IRQ(); //ʹ��IRQ
		if(config.prf== DWT_PRF_64M)
				RX_level_A = 121.74;
		if(config.prf== DWT_PRF_16M)
				RX_level_A = 113.77;
		
		/*���÷��书��*/	
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
				dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS );//���÷�����ɺ��������ӳ�ʱ��
				dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);	//���ý��ճ�ʱʱ��
				while(1)
				{
										
					/*��Poll������д��DW1000�����ڿ�������ʱ����ȥ*/
					dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
					dwt_writetxfctrl(sizeof(tx_poll_msg), 0);
					
					dwt_setleds(1);//�������շ���ָʾ��

					dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);//�����������Ͳ��ȴ�����response��
					

					while (!(status_reg=dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_RXFCG| CLEAR_ALLRXERROR_EVENTS)))//���ϲ�ѯ�Ĵ���ֱ�����ճɹ�
					{ };

					 if (status_reg & SYS_STATUS_RXFCG)//����ɹ�����
					 {
							uint32 frame_len_T = 0;
							
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//������ͽ��ձ�־λ
							
							frame_len_T = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;	//��ý��յ������ݳ���

							dwt_readrxdata(rx_buffer, frame_len_T, 0);   //��ȡ���յ�������
					
							if (rx_buffer[9] == 0x10)//�ж�response��
							{
									uint32 final_tx_time;	
																	
									poll_tx_ts = get_tx_timestamp_u64();//pollʱ���
									resp_rx_ts = get_rx_timestamp_u64();//respʱ���

									/*��������final������ʱ�� */
									final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
									dwt_setdelayedtrxtime(final_tx_time);
								
									final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;//finalʱ��������Ǳ�̵Ĵ���ʱ�����TX�����ӳ�

									/* final������������ */
									final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
									final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
									final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);				
									
									/*final������*/
									dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); 
									dwt_writetxfctrl(sizeof(tx_final_msg), 0); 
									dwt_starttx(DWT_START_TX_DELAYED);		//�����ӳٷ���					
									
									while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))//���ϲ�ѯ״ֱ̬�����ͳɹ�
									{ };
							
									dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);//������ͱ�־λ	
									tx_final_msg[2] = count;
									count++;

							}	
					}
					else
					{						
						dwt_write32bitreg(SYS_STATUS_ID, CLEAR_ALLRXERROR_EVENTS);//���DW1000״̬�Ĵ����е�RX�����¼� 
					}
						Sleep(100); //��ʱһ��ʱ��
				}
			}
			if(MODULE_ROLE == 1)
			{

				dwt_setsniffmode(1, SNIFF_ON_TIME, SNIFF_OFF_TIME);	
					
				while(1)
				{					
					int u=0;
					int ret_A=0;
					
					/*���չ��ʲ�����ʼ��*/
					dwt_rxdiag_t rxdiag_t;	
					uint32 D17F=pow(2,17);
					rxdiag_t.maxGrowthCIR=0;
					rxdiag_t.rxPreamCount=0;

					memset(rx_buffer,0,24);
					memset(SendBuff,0,130);
									
					dwt_setrxtimeout(0);//���ղ���ʱ
					dwt_rxenable(DWT_START_RX_IMMEDIATE);//������������
					
					while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | CLEAR_ALLRXERROR_EVENTS)))//���ϲ�ѯ�Ĵ���ֱ�����ճɹ�
					{ };

					if (status_reg & SYS_STATUS_RXFCG)
					{
						uint32 frame_len_A = 0;
						
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);		//������ձ�־λ				
						
						frame_len_A = dwt_read32bitreg(RX_FINFO_ID) &  RX_FINFO_RXFL_MASK_1023;//��ý��յ������ݳ���
						dwt_readrxdata(rx_buffer, frame_len_A, 0);//��ȡ��������
						
						if(rx_buffer[9] == 0x21)//�ж�poll��
						{		
							poll_rx_ts = get_rx_timestamp_u64();//���Poll������ʱ��T2					
							Response_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;//����Response����ʱ��T3��
							dwt_setdelayedtrxtime(Response_tx_time);//����Response����ʱ��T3							
							dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);//���÷�����ɺ��������ӳ�ʱ��
							dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);//���ճ�ʱʱ��
												 						
							
							/*response������*/
							dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); 
							dwt_writetxfctrl(sizeof(tx_resp_msg), 0);
							
							ret_A= dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED );	//�����ӳٷ��Ͳ��ȴ�����final��

							if (ret_A == DWT_ERROR)//�ж��Ƿ���ִ���
							{
								printf("3");
									continue;							
							}
							while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | CLEAR_ALLRXERROR_EVENTS)))//���ϲ�ѯ�Ĵ���ֱ�����ճɹ�
							{ };
							if (status_reg & SYS_STATUS_RXFCG)
							{								
								dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//������շ��ͱ�־λ
							
								frame_len_A = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;//��ý��յ������ݳ���

								dwt_readrxdata(rx_buffer, frame_len_A, 0);//��ȡ��������
							
								if(rx_buffer[9] == 0x23 )//�ж�final��
								{
									uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
									uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
									double Ra, Rb, Da, Db;
									int64 tof_dtu;
								
									resp_tx_ts = get_tx_timestamp_u64();//���response����ʱ��T3
									final_rx_ts = get_rx_timestamp_u64();//���final����ʱ��T6
									
									final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);//�ӽ��������ж�ȡT1��T4��T5
									final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
									final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

									/* ����ȡ����ʱ��ת��Ϊ32������. */
									poll_rx_ts_32 = (uint32)poll_rx_ts;
									resp_tx_ts_32 = (uint32)resp_tx_ts;
									final_rx_ts_32 = (uint32)final_rx_ts;
									Ra = (double)(resp_rx_ts - poll_tx_ts);//Tround1 = T4 - T1  
									Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);//Tround2 = T6 - T3 
									Da = (double)(final_tx_ts - resp_rx_ts);//Treply2 = T5 - T4  
									Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);//Treply1 = T3 - T2  
									tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));//���㹫ʽ	
									
									tof = tof_dtu * DWT_TIME_UNITS;
									distance = tof * ((float)SPEED_OF_LIGHT);	//�������									
									if(distance < 0)
										distance = 0;
									/* ��ȡ���չ��ʲ���. */
									dwt_readdignostics(&rxdiag_t);
									RX_level_C=(int)rxdiag_t.maxGrowthCIR;
									RX_level_N=(int)rxdiag_t.rxPreamCount;
									
									/* ���չ��ʼ���. */
									RX_level=RX_level_C*D17F;
									RX_level=RX_level/(RX_level_N*RX_level_N);
									RX_level=10*log10(RX_level);
									RX_level=RX_level-RX_level_A;

									u+=sprintf((char *)&SendBuff[0],"  NUMBER: %d POWER:%lf dBM DIS:%3.2fm\r\n\r\n",(rx_buffer[2]-'0')+48,RX_level,distance);

									USB_TxWrite(SendBuff, u);	
									printf("%s",SendBuff);
									dwt_setleds(1);/* ����W1000���շ���ָʾ�� */
							}
						}
						else
						{
								/*���DW1000״̬�Ĵ����е�RX�����¼�. */
								dwt_write32bitreg(SYS_STATUS_ID, CLEAR_ALLRXERROR_EVENTS);
								/* �ر�DW1000���շ���ָʾ�� */
								dwt_setleds(0);
								/* ���������� */
								dwt_rxreset();
						}			
					}			
				}
				else
				{
						/*���DW1000״̬�Ĵ����е�RX�����¼�. */
						dwt_write32bitreg(SYS_STATUS_ID, CLEAR_ALLRXERROR_EVENTS);
						/* ���������� */
						dwt_rxreset();
				}	
		}		
	 }		
 }
}

/*! ------------------------------------------------------------------------------------------------------------------
		��ȡTXʱ���
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
		��ȡRXʱ���
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
		final����������
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
		final�����ݶ�ȡ
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
