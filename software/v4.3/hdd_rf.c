#include <msp430f169.h>
#include "Config.h"
#include <string.h>
#include "ecc.h"

                          
#define SCLK_HIGH			(P4OUT |= (1<<2) )            
#define FSYNC_HIGH			(P4OUT |= (1<<1) )
#define SDATA_HIGH			(P4OUT |= (1<<3) )

#define SCLK_LOW			(P4OUT &= ~(1<<2) )            
#define FSYNC_LOW			(P4OUT &= ~(1<<1) )
#define SDATA_LOW			(P4OUT &= ~(1<<3) )

#define RF_TX				(P1OUT |= (1<<2))
#define RF_RX				(P1OUT &= ~(1<<2))
#define LED_ON                          (P1OUT |= (1<<5))
#define LED_OFF                         (P1OUT &=~ (1<<5))


#define FSK_TX				(P3OUT |= (1<<1))
#define FSK_RX				(P3OUT &= ~(1<<1))

#define D15				(1<<15)
#define	D14				(1<<14)
#define	D13				(1<<13)
#define D12				(1<<12)
#define D11_F0                          (0<<11)
#define D11_F1			        (1<<11)
#define D10_P0                          (0<<10)
#define D10_P1			        (1<<10)
#define D8				(1<<8)
#define D7				(1<<7)
#define D6				(1<<6)
#define	D5				(1<<5)
#define D3				(1<<3)
#define D1				(1<<1)


#define F1                              64
#define F2                              66
#define F3                              68
#define F4                              70
#define F5                              72
#define F6                              74
#define F7                              76
#define F8                              78
#define F9                              80
#define F10                             82
#define F11                             84
#define F12                             86
#define F13                             88
#define F14                             90
#define F15                             92
#define F16                             94
#define F17                             116


#define Fmclk			1000000    //1M HZ
#define NPAR 4                                                     
#define FRAME_SIZE        (sizeof(msg)+NPAR)

unsigned char msg[] = "Nervously I loaded the twin ducks aboard the revolving platform.";
//unsigned char codeword[256];
uchar send_str[256];
uint  str_len=0,bit_len=0,start_bit;
uchar rece_finish_flag,send_finish_flag=1;
uchar tmp=0;
uint timcnt=0;
 

void port_init()
{
    P4DIR = 0xF;
    P4OUT = 0xff;
    
    //P1DIR &= 0xdf;   //p1.4 input
    //P1IES &= 0xdf;   //上升沿中�?	P1IE  |= 0x20;
    
    //P1DIR |= 0x24;     //p1.2 OUTPUT RF SWITCH
    //P1OUT |= 0x4;
    // P1IE |= 0X40;
    
    
    //P3DIR |= 0x2;     //p3.1 fsk tx/rx switch
    
    P1DIR = 0xff;      //p1.6 input
    P1OUT=0XFF;
    //P1IE  = 0x40;     
    //P1OUT &= !(0XC);
    // P1OUT |= 0X4;
    
    
    //P6SEL = 0X80;
    P6SEL = 0Xf0;      //p6.3-7 ADC
    
    
    P3DIR &= ~0x30;  //p3.4 p3.5 input
    
}
void serial_write(uint data)
{
    uint i=0;
    SCLK_HIGH;
    FSYNC_HIGH;
    FSYNC_LOW; 
    for(i=0;i<16;i++)
    {	
        if(data&0x8000)
            SDATA_HIGH;
        else	
            SDATA_LOW;
        delay_us(1);   //t7
        SCLK_LOW;
        delay_us(2);  //t6
        SCLK_HIGH;
        data<<=1;
    }
    FSYNC_HIGH;
    SCLK_LOW;
}


void set_req0(uint msb,uint lsb)
{

    serial_write(D13);  //D13=1
    serial_write(D14 | lsb);
    serial_write(D14 | msb); 
}
void set_req1(uint msb,uint lsb)
{

    serial_write(D13  );  //D13=1 
    serial_write(D15 | lsb);
    serial_write(D15 | msb);
}

void init_ad9833(void)
{ 
    serial_write(D8);  //RESET=1
    serial_write(0xd000);
    serial_write(0xf7ff);
    
}
void set_frq0(uint freq)
{	
    uint msb,lsb;
    unsigned long tmp;
    tmp = freq*268.435;   
    lsb = tmp&0x3fff;
    msb = (tmp>>14)&0x3fff;                
    set_req0(msb,lsb);	
}
void set_frq1(uint freq)
{	
  /*
    uint msb,lsb;
    unsigned long tmp;
    tmp = Fout1*268.435;
    lsb = tmp&0x3fff;
    msb = (tmp>>14)&0x3fff;
    set_req1(msb,lsb);	
   // delay_us(10);
  */
}


void UART_Init()
{
   U0CTL|=SWRST;               //��λSWRST
  U0CTL|=CHAR;                //8λ����ģʽ 
  U0TCTL|=SSEL1;              //SMCLKΪ����ʱ��
  U0BR1=baud_h;               //BRCLK=8MHZ,Baud=BRCLK/N
  U0BR0=baud_l;               //N=UBR+(UxMCTL)/8
  U0MCTL=0x00;                //΢���Ĵ���Ϊ0��������9600bps
  ME1|=UTXE0;                 //UART1����ʹ��
  ME1|=URXE0;                 //UART1����ʹ��
  U0CTL&=~SWRST;
  IE1|=URXIE0;                //�����ж�ʹ��λ
  
  P3SEL|= BIT4;               //����IO��Ϊ��ͨI/Oģʽ
  P3DIR|= BIT4;               //����IO�ڷ���Ϊ���
  P3SEL|= BIT5;
  
  
  U1CTL|=SWRST;               //��λSWRST
  U1CTL|=CHAR;                //8λ����ģʽ 
  U1TCTL|=SSEL1;              //SMCLKΪ����ʱ��
  U1BR1=baud_h;               //BRCLK=8MHZ,Baud=BRCLK/N
  U1BR0=baud_l;               //N=UBR+(UxMCTL)/8
  U1MCTL=0x00;                //΢���Ĵ���Ϊ0��������9600bps
  ME2|=UTXE1;                 //UART1����ʹ��
  ME2|=URXE1;                 //UART1����ʹ��
  U1CTL&=~SWRST;
  IE2|=URXIE1;                //�����ж�ʹ��λ
  

    
    P3SEL|= BIT6;               //设置IO口为普通I/O模式
    P3DIR|= BIT6;               //设置IO口方向为输出
    P3SEL|= BIT7;
}

void ADC_Init()
{
  P6SEL|=0x10;                                    //ѡ��ADCͨ����p6.4
  ADC12CTL0|= ADC12ON + SHT0_2 + REF2_5V + REFON; //ADC��Դ���ƿ���16��CLK���ڲ���׼2.5V
  ADC12CTL1|= ADC12SSEL1 + ADC12SSEL0;            //SMCLK��ʱ��Դ
  ADC12MCTL0= SREF0 + INCH_4;                     //�ο�����λ��ͨ��ѡ������ѡ��ͨ��
  //ADC12MCTL5= SREF0 + INCH_5;
 // ADC12MCTL6= SREF0 + INCH_6;
  //ADC12MCTL7= SREF0 + INCH_7;
  
  ADC12IE|= 0x1;                                 //�ж�����
  ADC12CTL0|= ENC;                                //ʹ��ת����
}
void DAC12_Init(void)
{
  DAC12_1CTL |= DAC12CALON;             //����У��DAC
  while((DAC12_1CTL & DAC12CALON) != 0){_NOP();}//�ȴ�У�����
  
  ADC12CTL1 = REFON + REF2_5V;                        // �ο���ѹΪ�ڲ�2.5v
  DAC12_1CTL = DAC12AMP_5 + DAC12IR + DAC12LSEL_0;//���ƼĴ������ã��Զ��������ݣ��ڻ�׼Ϊ������
  DAC12_1CTL |= DAC12SREF_0;
  DAC12_1CTL |= DAC12ENC; // ����DACģ��(DAC12LSEL_0ʱ�˾����ʡ����������Ϊ1)
  DAC12_1DAT = 0xfff/2;    //DAC��ֵΪ0����������Ĵ����Ϳ��Ը���DAC�������
  
  
  /*
  DAC12_0CTL |= DAC12CALON;             //����У��DAC
  while((DAC12_0CTL & DAC12CALON) != 0){_NOP();}//�ȴ�У�����
 
  ADC12CTL0 = REFON + REF2_5V;                        // �ο���ѹΪ�ڲ�2.5v
  DAC12_0CTL = DAC12AMP_5 + DAC12IR + DAC12LSEL_0;//���ƼĴ������ã��Զ��������ݣ��ڻ�׼Ϊ������
  DAC12_0CTL |= DAC12SREF_0;
  DAC12_0CTL |= DAC12ENC; // ����DACģ��(DAC12LSEL_0ʱ�˾����ʡ����������Ϊ1)
  DAC12_0DAT = 0xfff;    //DAC��ֵΪ0����������Ĵ����Ϳ��Ը���DAC�������
  */

}


void TIMERA_Init(void)                                      //UPģʽ��������������ΪCCR0+1
{
 
    TACTL |= TASSEL1 + TACLR + ID0 + ID1 + MC0 + TAIE;     //SMCLK��ʱ��Դ��8��Ƶ�����Ӽ���ģʽ�����ж�
    TBCTL |= TBSSEL1 + TBCLR + ID0 + ID1 + MC1;             //SMCLK��ʱ��Դ��8��Ƶ�����Ӽ���ģʽ
    TACCR0 = 9999;                                         //CCR0=9999��10ms�ж�һ��
   // TACCR0 = RF_BAUD*100-1;
    TACCR0 = 4999*10; 
    TACCR0 = 50000; 
    TACCR0 = 31250;

   
}

#pragma vector=TIMERA1_VECTOR  
__interrupt void Timer_A(void)  
{ 
   uchar twobit;
    switch(TAIV)  
   {
      case 2:break;  
      case 4:break;  
      case 10:
        {
          timcnt++;
          if(timcnt != 5) break;

          timcnt=0;
          if(!send_finish_flag)
          {
              if(str_len==FRAME_SIZE) 
              { 
                  send_finish_flag=1;
                  bit_len=0;
                  str_len=0;

              } 
              
              if( (bit_len>=1) &&(bit_len<=8))
              {

                tmp = send_str[str_len]; 
                if(bit_len>4)
                {
                  twobit = tmp &0xF;
                }
                else
                {
                  tmp>>=4-(bit_len-1);     
                  twobit = tmp &0xF;
                }
                
                switch(twobit)
                {
 #if 1
                case 0:
                  set_frq0(F1);  
                  break;                
                case 1:
                  set_frq0(F2);
                  break;
                case 2:
                  set_frq0(F3);
                  break;
                case 3:
                  set_frq0(F4);
                  break;
                case 4:
                  set_frq0(F5);  
                  break;
                case 5:
                  set_frq0(F6);
                  break;
                case 6:
                  set_frq0(F7);
                  break;
                case 7:
                  set_frq0(F8);
                  break;
                case 8:
                  set_frq0(F9);  
                  break;
                case 9:
                  set_frq0(F10);
                  break;
                case 10:
                  set_frq0(F11);
                  break;
                case 11:
                  set_frq0(F12);
                  break;
                case 12:
                  set_frq0(F13);  
                  break;
                case 13:
                  set_frq0(F14);
                  break;
                case 14:
                  set_frq0(F15);
                  break;
                case 15:
                  set_frq0(F16);
                  break;  
#endif
                  
                }
                //set_frq0(0);  
                serial_write(D11_F0);  
                bit_len++;
                bit_len++;
                bit_len++;
                bit_len++;

                if(bit_len>=9)
                {
                  bit_len=0;
                  str_len++;
                }
              }
              else if(bit_len == 0)    
              {
                bit_len++;
                set_frq0(F17);
                serial_write(D11_F0);
              }   
          }
      
        }
      break;  
    }  
}  
#pragma vector=UART1RX_VECTOR
__interrupt void UART1_RX_ISR(void)
{
  uchar data=0;
  data=U1RXBUF;         
} 
#pragma vector=UART1TX_VECTOR
__interrupt void UART1_TX_ISR(void)
{

}
#pragma vector = PORT1_VECTOR
__interrupt void P1_IRQ(void)
{
#if 0
    switch(P1IFG &0x40)
    {
      /*
          case 0x01: // 引脚0
          case 0x02: // 引脚1
                  
          case 0x04: // 引脚2
          case 0x08: // 引脚3
          case 0x10: // 引脚4
          case 0x20: // 引脚5
      */
          case 0x40: // 引脚6             
            P1IFG=0x00;
            //difftimer=TBR;
            //TBCTL |= TBCLR;            
            difftimer=TBR-curtimer;
            curtimer =TBR;    
            diff_timer[cnt1++]=difftimer;
            if(difftimer >=CARRIER0_TIME-E_RANG && difftimer <=CARRIER0_TIME+E_RANG ) 
            {
              carrier0_cnt++;  
              bit1_cnt= (carrier1_cnt+NUM_OFF_BIT1-1)/NUM_OFF_BIT1;
              switch(bit1_cnt)
              {
                case 0: break;
                case 1: {rev_char += (0x1<<bit_cnt); break;}           
                case 2: {rev_char += (0x3<<bit_cnt); break;}
                case 3: {rev_char += (0x7<<bit_cnt); break;}
                case 4: {rev_char += (0xf<<bit_cnt); break;}
                case 5: {rev_char += (0x1f<<bit_cnt);break;}
                case 6: {rev_char += (0x3f<<bit_cnt);break;}
                case 7: {rev_char += (0x7f<<bit_cnt);break;}
                case 8: {rev_char += (0xff<<bit_cnt);break;}
              } 
              bit_cnt += bit1_cnt;
              carrier1_cnt=0;
            }
            else if(difftimer >=CARRIER1_TIME-E_RANG && difftimer <= CARRIER1_TIME+E_RANG)
            {
              carrier1_cnt++;
              bit0_cnt = (carrier0_cnt+NUM_OFF_BIT0-1)/NUM_OFF_BIT0;
              bit_cnt += bit0_cnt;    
              carrier0_cnt=0;
            }  
            else if(difftimer >(BIT_CYCLE - BIT_CYCLE*3/10))  // start bit 
            {
              bit_cnt=0;
              bit0_cnt=0;
              bit1_cnt=0;
              rev_char=0;              
              carrier0_cnt=0;
              carrier1_cnt=0; 
              rece_finish_flag=0;
            }
            if(bit_cnt>=8)   //receive 8 bit
            {
              rev_str[byte_cnt]=rev_char;  
              /*
              if(rev_char!=send_str[str_len-1])
              {
                err++;
              }*/
              
              byte_cnt++;
              rev_char=0;
              bit_cnt=0;
            }
            if(byte_cnt>=FRAME_SIZE && rece_finish_flag==0 )  //receive one frame data
            {
              rece_finish_flag=1;
              cnt1=0;
              byte_cnt=0;
              cnt2++;
            }              
            break;   
    }
#endif
}
void Data_do(uint temp_d)
{
  ulong temp_3;
  temp_3=(ulong)(temp_d)*100;            //ת����ʽ��ADC������ת��Ϊ��ѹ��С��ע����������
  //Precent = temp_3/0xFFF;	             //����ٷֱ�

}

//*************************************************************************
#pragma vector=ADC_VECTOR
__interrupt void ADC12ISR(void)
{
  uchar j;
  while((ADC12CTL1&0x01)==1);           //���ADCæ����ȴ��������ȡADCת����ֵ
  //Flag = 1 ;
 // TEMP = ADC12MEM0;                     //��ȡADCת��ֵ
 // Data_do(TEMP);                        //����ADCֵ��������ʾ��������
//  Voltage_do(TEMP);                     //����ADCֵ��������ʾ��ģ���ѹֵ
//  for(j=0;j<15;j++)
 // {
   //LCD_DisplayADC();                   //��ʾADC�����ݣ�ADC������
   // LCD_DisplayVoltage();               //��ʾADC�ĵ�ѹֵ
 // }
}

void Send_Byte(uchar data)
{
  while(!(IFG2&UTXIFG1));          //���ͼĴ����յ�ʱ��������
    U1TXBUF=data;
}

void write_send_data(void)
{
}
void enable_send(void)
{
   send_finish_flag=0; 
}

int main(void)
{
    int cnt,s1,s2;
    unsigned int sum;
    WDT_Init();
    Clock_Init();                         
   // UART_Init();
    port_init();
    ADC_Init();
    init_ad9833();
//    set_frq0(Fout0);
//    set_frq1(Fout1);
    TIMERA_Init();
    DAC12_Init();
    _EINT();   
    delay_ms(10);
    enable_send();
    LED_ON;
    //Flag=1;
    initialize_ecc ();
    encode_data(msg, sizeof(msg), send_str);
    //LED_OFF ;
    cnt=0;
    sum=0;
    while(1)
    {   
 
      if(send_finish_flag)    // send finish one frame
      {
        enable_send();  //second send one frame

     }
      if(rece_finish_flag)   // receive one frame
      {
         rece_finish_flag=0;
          
      }
    }
}


