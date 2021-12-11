
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>


#include <ESP32Encoder.h>
#include <HardwareSerial.h>
#include <driver/uart.h>

ESP32Encoder encoder1;

#define LED_ON LOW
#define LED_OFF HIGH

//See file .../hardware/espressif/esp32/variants/(esp32|doitESP32devkitV1)/pins_arduino.h
#define LED_BUILTIN 22 // Pin D2 mapped to pin GPI22/ADC12 of ESP32, control on-board LED


#define DAC1 25
#define SendReceive 17
#define DoSend true
#define DoReceive false


#define RS485_UART_PORT UART_NUM_2
#define RS485_TXD   (19)
#define RS485_RXD   (18)
#define RS485_RTS   (17)
#define RS485_CTS  UART_PIN_NO_CHANGE
#define BUF_SIZE        (127)
#define BAUD_RATE       (19200)
#define PACKET_READ_TICS        (100 / portTICK_RATE_MS)

#define STW_EIN 1
#define STW_N_AUS2 2
#define STW_N_AUS3 4
#define STW_BETRIEB_FREIGABE 8 //3
#define STW_N_HOCHLAUFGEBER_SPERREN 16 //4
#define STW_HOCHLAUFGEBER_FREIGEBEN 32
#define STW_SOLLWERT_FREIGEBEN 64
#define STW_QUITTIEREN 128 //7
#define STW_TIPPEN1 256
#define STW_TIPPEN2 512
#define STW_FUEHREN 1024 //10
#define STW_DIRECTION 2048 // Micromaster4
#define STW_RIGHT 2048 // rechts Micromaster3
#define STW_LEFT 4096 // links Micromaster3
#define STW_13 8192
#define STW_14 16384
#define STW_15 32768

#define ZSW_Einschaltbereit 1
#define ZSW_Betriebsbereit 2
#define ZSW_Betrieb_freigegeben 4
#define ZSW_STOERUNG 8
#define ZSW_N_AUS2 16
#define ZSW_N_AUS3 32
#define ZSW_Einschaltsperre 64
#define ZSW_Warnung 128
#define ZSW_SOLL_IST_OK 256
#define ZSW_Fuehrung_gefordert 512
#define ZSW_SOLL_ERREICHT 1024
#define ZSW_11 2048
#define ZSW_12 4096
#define ZSW_13 8192
#define ZSW_14 16384
#define ZSW_15 32768

static void rs485_send(const uart_port_t port, const char* str, uint8_t length)
{
    if (uart_write_bytes(port, str, length) != length) {
        ESP_LOGE(TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}

static void uss_send(unsigned char addr,const unsigned char* str, uint8_t length)
{
    int buflen=length+4;
    char buf[500];
    buf[0]=0x02;
    buf[1]=(char)(length+2); // + Adresse und BCC
    buf[2]=addr; 
    unsigned char BCC=0;
    BCC^=buf[0];
    BCC^=buf[1];
    BCC^=buf[2];
    for(int i=0;i<length;i++)
    {
      buf[3+i]=str[i];
      BCC^=buf[3+i];
    }
    buf[3+length]=BCC;
    if (uart_write_bytes(RS485_UART_PORT, buf, buflen) != buflen) {
        ESP_LOGE(TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}

  static int uss_receive(const unsigned char* str)
{
    return uart_read_bytes(RS485_UART_PORT, (uint8_t*)str, 100,PACKET_READ_TICS);
}
void uss_send_PZD(unsigned short STW, unsigned short value)
{
  unsigned char buf[4];
  buf[0]=(STW>>8)&0xff;
  buf[1]=STW&0xff;
  buf[2]=(value>>8)&0xff;
  buf[3]=value&0xff;
  uss_send(0x00,buf,2*2);
}
int readStatus(const unsigned char *buf)
{
  return uss_receive(buf);
}

void printStatus(const unsigned char *buf, int buflen)
{
  
  for(int i=0;i<buflen;i++)
  {
    unsigned char c = (buf)[i];
    for(int n=7;n>=0;n--)
    {
      if(c&1<<n)
          Serial.print(1);
      else
        Serial.print(0);
    }
    Serial.print(" ");
  }
  Serial.printf("r%d v%x %x %x %x\n",buflen,((unsigned short *)buf)[0],((unsigned short *)buf)[1],((unsigned short *)buf)[2],((unsigned short *)buf)[3]);
}

void setup()
{
  //set led pin as output
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(32,INPUT_PULLUP);
  pinMode(33,INPUT_PULLUP);
  pinMode(26,OUTPUT);
  pinMode(25,OUTPUT);
  pinMode(SendReceive,OUTPUT);
  
  digitalWrite(SendReceive,DoReceive);
  
  //digitalWrite(LED_BUILTIN,LED_ON);
  Serial.begin(115200);
  while (!Serial);
  const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
 
    //Serial.println("Start RS485 application test and configure UART.");
 
    // Configure UART parameters
    uart_param_config(uart_num, &uart_config);
 
    //Serial.println("UART set pins, mode and install driver.");
    // Set UART1 pins(TX: IO23, RX: I022, RTS: IO18, CTS: IO19)
    uart_set_pin(uart_num, RS485_TXD, RS485_RXD, RS485_RTS, RS485_CTS);
 
    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);
 
    // Set RS485 half duplex mode
    uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
  	

  delay(500);
  //digitalWrite(LED_BUILTIN,LED_ON);

	ESP32Encoder::useInternalWeakPullResistors=UP;

	encoder1.attachHalfQuad(32, 33);
  encoder1.setCount(0);



}


void toggleLED()
{
  //toggle state
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}


void loop()
{
  
  //Serial.println(degree); 
  unsigned int buf[100];
  unsigned int PKE;
  PKE = 9&1<<12;
  buf[0]=PKE;
  buf[1]=0;
  buf[2]=30;
  buf[3]=PKE;
  buf[4]=0;
  buf[5]=30;
  buf[6]=PKE;
  buf[7]=0;
  buf[8]=30;
  //uss_send((unsigned char *)buf,2*2);
  
  buf[9]=0x047F;
  buf[10]=0x3333;
  buf[11]=0;
  buf[12]=0;
  buf[13]=0x047F;
  buf[14]=0x3333;
  buf[15]=0;
  buf[16]=0;
  
  int numRead;
  /*for(int i=0;i<50*2;i++)
  {
  uss_send((unsigned char *)buf,i);

  numRead = uss_receive((unsigned char *)buf);
  Serial.printf("i%d v%d\n",i,numRead);
  }*/
/*for(int i=0;i<255;i++)
{
  buf[0]=0x047E;
  buf[1]=0x3333;
  buf[2]=0;
  buf[3]=0;
  uss_send(0x01,(unsigned char *)buf,i*2);

  numRead = uss_receive((unsigned char *)buf);
  Serial.printf("i%d v%d\n",i,numRead);
  delay(10*i);
}*/
  /*buf[0]=0x047F;
  buf[1]=0x4000;
  buf[2]=0;
  buf[3]=0;
  uss_send(0x00,(unsigned char *)buf,4*2);

  numRead = uss_receive((unsigned char *)buf);
  Serial.printf("i%d v%d\n",1,numRead);*/
/*
#define STW_EIN 1
#define STW_N_AUS2 2
#define STW_N_AUS3 4
#define STW_BETRIEB_FREIGABE 8 //3
#define STW_N_HOCHLAUFGEBER_SPERREN 16 //4
#define STW_HOCHLAUFGEBER_FREIGEBEN 32
#define STW_SOLLWERT_FREIGEBEN 64
#define STW_QUITTIEREN 128 //7
#define STW_TIPPEN1 256
#define STW_TIPPEN2 512
#define STW_FUEHREN 1024 //10
#define STW_11 2048
#define STW_12 4096
#define STW_13 8192
#define STW_14 16384
#define STW_15 32768*/
  unsigned char cbuf[100];
  for(int i=0;i<100;i++)
  {
    cbuf[i]=0;
  }
  
  uss_send_PZD(STW_N_AUS2|STW_N_AUS3|STW_BETRIEB_FREIGABE|STW_N_HOCHLAUFGEBER_SPERREN|STW_FUEHREN,0);

  /*
  for (int i = 1150; i < 1151; i++)
  {
    unsigned char cbuf2[100];
    uss_send_PZD(i, 5000);
    int n = readStatus(cbuf2);
    bool changed = false;
    for (int m = 0; m < n; m++)
    {
      if (cbuf2[m] != cbuf[m])
      {
        changed = true;
        break;
      }
    }
    if((i%0x100)==0)
    {
      Serial.println(i);
    }
    if (changed)
    {
      Serial.println("change");
      Serial.println(i);
      printStatus(cbuf2, n);
    }
    delay(1000);
  }*/
  Serial.println("Init");
 uss_send_PZD(STW_EIN|STW_SOLLWERT_FREIGEBEN|STW_N_AUS2|STW_N_AUS3|STW_BETRIEB_FREIGABE|STW_N_HOCHLAUFGEBER_SPERREN|STW_FUEHREN,1000);
  readStatus(cbuf);
printStatus(cbuf, 8);
  uss_send_PZD(STW_FUEHREN,0);
  readStatus(cbuf);
printStatus(cbuf, 8);
  uss_send_PZD(STW_N_AUS2|STW_N_AUS3|STW_BETRIEB_FREIGABE|STW_N_HOCHLAUFGEBER_SPERREN|STW_FUEHREN,0);
  readStatus(cbuf);
printStatus(cbuf, 8);

  uss_send_PZD(STW_EIN|STW_SOLLWERT_FREIGEBEN|STW_N_AUS2|STW_N_AUS3|STW_BETRIEB_FREIGABE|STW_N_HOCHLAUFGEBER_SPERREN|STW_HOCHLAUFGEBER_FREIGEBEN|STW_FUEHREN|STW_RIGHT,0);
  readStatus(cbuf);
printStatus(cbuf, 8);
  Serial.println("InitDone");
for(int i=0;i<5000;i+=200)
{
  uss_send_PZD(STW_EIN|STW_SOLLWERT_FREIGEBEN|STW_N_AUS2|STW_N_AUS3|STW_BETRIEB_FREIGABE|STW_N_HOCHLAUFGEBER_SPERREN|STW_HOCHLAUFGEBER_FREIGEBEN|STW_FUEHREN|STW_RIGHT,i);
  readStatus(cbuf);
}
/*Serial.println("disable");
Serial.println(STW_EIN|STW_SOLLWERT_FREIGEBEN|STW_N_AUS2|STW_N_AUS3|STW_BETRIEB_FREIGABE|STW_N_HOCHLAUFGEBER_SPERREN|STW_HOCHLAUFGEBER_FREIGEBEN|STW_FUEHREN|STW_RIGHT);
while(true)
{
  
  uss_send_PZD(STW_N_AUS2|STW_N_AUS3|STW_BETRIEB_FREIGABE|STW_N_HOCHLAUFGEBER_SPERREN|STW_FUEHREN,0);
  readStatus(cbuf);
}*/
for(int i=5000;i>200;i-=200)
{
  uss_send_PZD(STW_EIN|STW_SOLLWERT_FREIGEBEN|STW_N_AUS2|STW_N_AUS3|STW_BETRIEB_FREIGABE|STW_N_HOCHLAUFGEBER_SPERREN|STW_HOCHLAUFGEBER_FREIGEBEN|STW_FUEHREN|STW_RIGHT,i);
  readStatus(cbuf);
}

printStatus(cbuf, 8);
for(int i=0;i<5000;i+=200)
{
  uss_send_PZD(STW_EIN|STW_SOLLWERT_FREIGEBEN|STW_N_AUS2|STW_N_AUS3|STW_BETRIEB_FREIGABE|STW_N_HOCHLAUFGEBER_SPERREN|STW_HOCHLAUFGEBER_FREIGEBEN|STW_FUEHREN|STW_LEFT,i);
  readStatus(cbuf);
}
for(int i=5000;i>200;i-=200)
{
  uss_send_PZD(STW_EIN|STW_SOLLWERT_FREIGEBEN|STW_N_AUS2|STW_N_AUS3|STW_BETRIEB_FREIGABE|STW_N_HOCHLAUFGEBER_SPERREN|STW_HOCHLAUFGEBER_FREIGEBEN|STW_FUEHREN|STW_LEFT,i);
  readStatus(cbuf);
}

printStatus(cbuf, 8);

while(true)
{
  int dir = 0;
  float degree = (encoder1.getCount()%1200)/1200.0 * 360.0;
  Serial.println(degree);
  if(degree > 0)
  {
      dir=STW_RIGHT;
  }
  else
  {
      dir=STW_LEFT;
  }
      Serial.println(dir);
  uss_send_PZD(STW_EIN|STW_SOLLWERT_FREIGEBEN|STW_N_AUS2|STW_N_AUS3|STW_BETRIEB_FREIGABE|STW_N_HOCHLAUFGEBER_SPERREN|STW_HOCHLAUFGEBER_FREIGEBEN|STW_FUEHREN|dir,10000*(fabs(degree)/360.0));

  readStatus(cbuf);
}


/*
  for(int i=0;i<32;i++)
  {
  buf[0]=0xffff;
  buf[1]=0xffff;
  uss_send(0x00,(unsigned char *)buf,i);
  Serial.println(i);
  printStatus(cbuf,readStatus(cbuf));
  }*/
  

  delay(500);
  
  /*buf[0]=0x047E;
  buf[1]=0x4000;
  buf[2]=0;
  buf[3]=0;
  uss_send(0x00,(unsigned char *)buf,4*2);

  numRead = uss_receive((unsigned char *)buf);
  Serial.printf("i%d v%d\n",3,numRead);*/

   // rs485_send(UART_NUM_2, "Start RS485 UART test.\r\n", 24);
  //Serial.println(digitalRead(33));
  //Serial.println(digitalRead(32)); 
  /*if(degree < 0)
  {
    digitalWrite(26,true);
  }
  else
  {
    digitalWrite(26,false);
  }
  
  dacWrite(DAC1, 255*(fabs(degree)/360.0));*/

  
  delay(10);
}

