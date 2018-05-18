#include <msp430f169.h>
#include "Config.h"
#include <string.h>

                          
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

#if 0
#define F1                              64
#define F2                              68
#define F3                              72
#define F4                              76
#define F5                              80
#define F6                              84
#define F7                              88
#define F8                              92
#define F9                              96
#define F10                             100
#define F11                             104
#define F12                             108
#define F13                             112
#define F14                             116
#define F15                             120
#define F16                             124
#define F17                             128

#endif

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
//#define FREQ0_REG_VAL	(Fout0*256*1024*1024/Fmclk)
//#define FREQ1_REG_VAL	(Fout1*256*1024*1024/Fmclk)

                                                      //�����ʵ�
#define Fout0		  20					//200HZ
#define Fout1		  28				//180HZ

#define RF_BAUD           100      
#define BIT_CYCLE         500000        //1000*1000*10/RF_BAUD
#define NUM_OFF_BIT1      2           //Fout0/RF_BAUD
#define NUM_OFF_BIT0      5           //Fout1/RF_BAUD
#define CARRIER1_TIME     5000      //(1000*1000/Fout0)
#define CARRIER0_TIME     2000      //(1000*1000/Fout1)
#define E_RANG            800
#define FRAME_SIZE        1




/*
#define Fout0		  20					//200HZ
#define Fout1		  50					//180HZ
#define RF_BAUD           10        
#define BIT_CYCLE         100000        //1000*1000*10/RF_BAUD
#define NUM_OFF_BIT1      2           //Fout0/RF_BAUD
#define NUM_OFF_BIT0      5           //Fout1/RF_BAUD
#define CARRIER1_TIME     50000      //(1000*1000/Fout0)
#define CARRIER0_TIME     20000      //(1000*1000/Fout1)
#define E_RANG            1000
#define FRAME_SIZE        16
*/

uchar send_str[FRAME_SIZE]={0x23};
//uchar send_str[FRAME_SIZE]={0xaa,0x7e,0xab,0x57,0x12,0x0c,0xa8,0xd1};
//uchar send_str[FRAME_SIZE]={0xaa,0x7E,'a','b','c','d','e','f','g','h','i','j','k','l','m','n','1','2','3','4','5','6','7','8','9','o','p','q','r','s','t','w'};
//uchar send_str[FRAME_SIZE]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,};
uchar rev_str[FRAME_SIZE],rev_char;
//unsigned long difftimer;
uint  curtimer,difftimer,str_len=0,bit_len=0,start_bit;
uint  carrier0_cnt,carrier1_cnt,bit_cnt,bit1_cnt,bit0_cnt,byte_cnt;
uchar rece_finish_flag,send_finish_flag=1;
uint i=0,err=0,cnt=0;
int aa=0;
uchar tmp=0;
uint diff_timer[FRAME_SIZE*40],cnt1,cnt2;
uint timcnt=0;
uint TEMP=0;   
static uchar Flag=0;   

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
   // serial_write(D13 | D8);  //D13=1 
    serial_write(D13);  //D13=1
    serial_write(D14 | lsb);
    serial_write(D14 | msb); 
}
void set_req1(uint msb,uint lsb)
{
   // serial_write(D13 | D8 );  //D13=1 
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
      uint msb,lsb;
    unsigned long tmp;
    tmp = Fout1*268.435;
    lsb = tmp&0x3fff;
    msb = (tmp>>14)&0x3fff;
    set_req1(msb,lsb);	
   // delay_us(10);
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
          /*
              switch(tmp)
              { 
              case 0:  {serial_write(D11_F1); tmp++;break;}
              case 1:  {serial_write(D11_F0); tmp++;break;}  
              case 2:  {serial_write(D11_F0); tmp++;break;}
              case 3:  {serial_write(D11_F0); tmp++;break;}
              case 4:  {serial_write(D11_F0); tmp++;break;}
              case 5:  {serial_write(D11_F0); tmp++;break;}  
              case 6:  {serial_write(D11_F0); tmp++;break;}
              case 7:  {serial_write(D11_F1); tmp++;break;}    
              
             
              case 8:  {serial_write(D11_F1); tmp++;break;}
              case 9:  {serial_write(D11_F0); tmp++;break;}  
              case 10:  {serial_write(D11_F1); tmp++;break;}
              case 11:  {serial_write(D11_F0); tmp++;break;}
              case 12:  {serial_write(D11_F1); tmp++;break;}
              case 13:  {serial_write(D11_F0); tmp++;break;}  
              case 14:  {serial_write(D11_F1); tmp++;break;}
              case 15:  {serial_write(D11_F1); tmp=0;break;}
              }
            */  

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
                //set_frq0(Fout0);
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

                /*               
                if(tmp &(0x1<<(7-(bit_len-1))))   
                { 
                serial_write(D11_F0); }
                else                 
                {//   serial_write(D8);
                  serial_write(D11_F1);
                }
                bit_len++;*/
                if(bit_len>=9)
                {
                  bit_len=0;
                  str_len++;
                }
              }
              else if(bit_len == 0)    //
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
  Flag = 1 ;
  TEMP = ADC12MEM0;                     //��ȡADCת��ֵ
  Data_do(TEMP);                        //����ADCֵ��������ʾ��������
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

  static int j='a';
// bit_len=0;
//  str_len=0;
  //serial_write(D11_F1);
  //delay_ms(BIT_CYCLE/1000);

  
  
 // for(j=0;j<FRAME_SIZE-2;j++)
    //for(j=0;j<FRAME_SIZE;j++)
    //send_str[2+j]= 0x55;
    send_str[0]= j++;
    //send_str[0]= 0x32;
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
    set_frq0(Fout0);
    set_frq1(Fout1);
    TIMERA_Init();
    DAC12_Init();
    _EINT();   
    delay_ms(10);
    enable_send();
    LED_ON;
    Flag=1;

    //LED_OFF ;
    cnt=0;
    sum=0;
    while(1)
    {   
#if 0   
      s2 = P3IN;
      s2 &= 0x30;
      s2 >>= 4;
      
      if(s2!=s1)
      {
        s1=s2;
       
        switch(s2)
        {
        case 0:
          set_frq0(15);
          set_frq1(15);
          break;
        case 1:
          set_frq0(5);
          set_frq1(5);
          break;
        case 2:
          set_frq0(10);
          set_frq1(10);
          break;
        case 3:
          set_frq0(20);
          set_frq1(20);
          break;
        }
        
      }
#endif  
      
      //delay_ms(10);
      //Send_Byte(0X55);
      if(send_finish_flag)    // send finish one frame
      {
        //delay_ms(500);
        //set_req2();
        //serial_write(D11_F0);
        enable_send();  //second send one frame
      //  Send_Byte(0X55);
       // DAC12_1DAT+=10;
      //}
      while(Flag==1)
      {
        ADC12CTL0 |= ADC12SC;           //����ת��
        ADC12CTL0 &= ~ADC12SC;          //����
        Flag=0;                         //�����־λ
        cnt++;
        sum+=TEMP;
        if(cnt>=20)
        {
          cnt=0;
          if(sum/20 > 2300)
          {
          //  P4OUT&=(~0x1);
          }
          sum=0;
        }
        
      }
      }
    }
      if(rece_finish_flag)   // receive one frame
      {
         rece_finish_flag=0;
         cnt++;
         byte_cnt=0;
         for(i=0;i<FRAME_SIZE;i++)
         {
           if(rev_str[i]!=send_str[i])
               err++;
         }
         for(i=0;i<FRAME_SIZE;i++)
         {
           rev_str[i]=0;
         }
      
  /*    
      if(cnt==10)
      {
       cnt=0;
      }	
  */    
    }
}


