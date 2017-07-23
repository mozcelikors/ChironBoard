/*
License:
	ChironBoard is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	ChironBoard is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with ChironBoard.  If not, see <http://www.gnu.org/licenses/>.

Description:
	This software is used to interpret and react upon several AT commands.
	Developed for Arduino boards, interfaced with nRF24L01 and internal EEPROM.

Authors: 
	Mustafa Ozcelikors 
		thewebblog.net
		github.com/mozcelikors
		<mozcelikors@gmail.com>
	Metin Kundakcioglu
		<kundakcioglum@gmail.com>
		
Note:
	This code may contain Turkish commenting.
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <string.h>
#include <EEPROM.h>

//Bir kerede transmit bufferdan paketlenip atilacak byte sayisi
#define PAYLOAD_BYTES 32

#define AFTERSEND_DELAY 10//100
#define RECEIVE_DELAY 20//20

//Transmit Mode mu, read mode mu(RFMode degiskeni)...
#define TRANSMIT_MODE 1
#define READ_MODE 2
#define TRANSCEIVE_MODE 3

//Maksimum ve minimum konfigurasyon degerleri
#define PIPE_MIN ((int)0)
#define PIPE_MAX ((int)255)
#define CHANNEL_MIN 0 
#define CHANNEL_MAX 90

//EEPROMa baslangic ayari yazmak icin alttaki tanimlanmali
#define eeprom_first_config
#define DEVICE_SN 245
#define DEVICE_SN2 223
//EEPROM boyutu (byte)
#define EEPROM_SIZE 1024

//EEPROMda tutulacak RF Konfigurasyon Adresleri
#define MODE_EEPROM_ADDRESS 0
#define KPIPE_EEPROM_ADDRESS 1
#define YPIPE_EEPROM_ADDRESS 2
#define CHANNEL_EEPROM_ADDRESS 3
#define RXTXMODE_ADDRESS 4
#define SN_ADDRESS 5 //Device Serial Number
#define SN2_ADDRESS 6

RF24 radio(9,10);

//RF pipe adresleri
const uint64_t pipes[2] = { 0xF0F0F1F0E1LL, 0xF0F0F0F0D2LL };

//RFden yollanacak datayi tutan degiskenler
unsigned  char data[PAYLOAD_BYTES];
unsigned  char *pdata = data;

//RFden okunacak datayi tutan degiskenler
unsigned  char rdata[PAYLOAD_BYTES];
unsigned  char *prdata = rdata;

unsigned int DEVICE_SN_READ;
unsigned int DEVICE_SN2_READ;
unsigned int CURRENT_MODE;
unsigned int CURRENT_KPIPE, CURRENT_YPIPE, CURRENT_CHANNEL;
unsigned int NEW_MODE;
unsigned int NEW_KPIPE, NEW_YPIPE, NEW_CHANNEL;

unsigned long lastMillisCheck;
unsigned long nrfwaitMillis;
byte checkMillisFlag=0;

byte eeprom_reset_flag=0;

//Digital pinler
const int PINS[5]={3,4,5,6,7};
unsigned int PINS_DIR_ARR[5]={0,0,0,0,0};//Hepsi input

//Seri porttan alinan veriyi tutacak buffer
String buf="";

int count, virgul_sayisi=0;
int virguller[3];
int startl = 0, endl = 5, existsl = 0;
int startr = 0, endr = 5, existsr = 0;
int undetected_error=0;
String filterString, filterString2;
int failed_count = 0;

//RF ile yollanacak buffer
String transmit_buf;

//Baslangicta transmit modundayiz
unsigned int RFMode;

int incomingDataSize = 0;

unsigned int first_time_read_mode_flag = 0;
byte command_detected_flag = 0;

void CHIRON_SERIALBUFFER_TASK(void);
void CHIRON_BUFFILTER_TASK(void);
void CHIRON_RFSEND_TASK(void);
void CHIRON_EEPROM_READSETTINGS_TASK(void);
void CHIRON_EEPROM_WRITECHANGES_TASK(void);
void CHIRON_RFREAD_TASK(void);
void CHIRON_ACKNOWLEDGE(void);
void CHIRON_TRANSCEIVE_TASK(void);
void(*resetFunc)(void) = 0; //Software reset function resetFunc();

void setup() {
  
  Serial.begin(57600);
  
  //EEPromdan RF ayarlarini okuyalim ve ilgili ayarlari yapalim
  CHIRON_EEPROM_READSETTINGS_TASK();
  
  //Ust fonksiyonda EEPromdan channel okundu ve o channelde baslatildi
  radio.begin(CURRENT_CHANNEL);

  for(char k=0; k<5; k++){
     pinMode(PINS[k],INPUT);
  }
  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  radio.openReadingPipe(1,pipes[CURRENT_YPIPE]);
  radio.openWritingPipe(pipes[CURRENT_KPIPE]);
  delay(2);
}



void loop () {
    //---Seri porttan buffera(buf) veri Al----------------
    CHIRON_SERIALBUFFER_TASK();
    CHIRON_ACKNOWLEDGE();
    if(checkMillisFlag==1){
        //---Bufferdaki veriyi isle------------------------
        CHIRON_BUFFILTER_TASK();
        
        //---Transmit bufferdan RFle veri yolla----------------
        //Transmit bufferimiz (transmit_buf) doluysa o bufferdaki stringi 32lik paketler halinde
        //yollayip bufferi bosaltalim
        CHIRON_RFSEND_TASK();
        
        CHIRON_TRANSCEIVE_TASK();
    }
    //Read_data_flag set olduysa veriyi okuyup flagi clear edelim
    CHIRON_RFREAD_TASK();
    
    //EEPROMda bir degisiklik yapilacaksa o degisiklikleri yapalim
    CHIRON_EEPROM_WRITECHANGES_TASK();
}

void CHIRON_EEPROM_WRITECHANGES_TASK(void)
{
    if(CURRENT_MODE != NEW_MODE)
    {
        eeprom_reset_flag = 1;
        EEPROM.write(MODE_EEPROM_ADDRESS,NEW_MODE);
        CURRENT_MODE = NEW_MODE;
        buf="";
    }
    if(CURRENT_KPIPE != NEW_KPIPE)
    {
        eeprom_reset_flag = 1;
        EEPROM.write(KPIPE_EEPROM_ADDRESS,NEW_KPIPE);
        CURRENT_KPIPE = NEW_KPIPE;
        buf="";
    }
    if(CURRENT_YPIPE != NEW_YPIPE)
    {
        eeprom_reset_flag = 1;
        EEPROM.write(YPIPE_EEPROM_ADDRESS,NEW_YPIPE);
        CURRENT_YPIPE = NEW_YPIPE;
        buf="";
    }
    if(CURRENT_CHANNEL != NEW_CHANNEL)
    {
        eeprom_reset_flag = 1;
        EEPROM.write(CHANNEL_EEPROM_ADDRESS,NEW_CHANNEL);
        CURRENT_CHANNEL = NEW_CHANNEL;
        buf="";
    }
    
    //?????????
    //Burada RFi kapat tekrar baslat
    //?????????
    
    if(eeprom_reset_flag)
    {
        resetFunc();
    }
    
    eeprom_reset_flag = 0;  
}

void CHIRON_EEPROM_READSETTINGS_TASK(void)
{
#ifdef eeprom_first_config
    //Baslangic configi
    EEPROM.write(MODE_EEPROM_ADDRESS,0);
    EEPROM.write(KPIPE_EEPROM_ADDRESS,1);
    EEPROM.write(YPIPE_EEPROM_ADDRESS,0);
    EEPROM.write(CHANNEL_EEPROM_ADDRESS,76);
    EEPROM.write(RXTXMODE_ADDRESS,READ_MODE);//1-TX 2
    EEPROM.write(SN_ADDRESS, DEVICE_SN);
    EEPROM.write(SN2_ADDRESS, DEVICE_SN2);
#endif
    
    CURRENT_MODE = EEPROM.read(MODE_EEPROM_ADDRESS); //0-  1-   2-
    CURRENT_KPIPE = EEPROM.read(KPIPE_EEPROM_ADDRESS); // 0~255
    CURRENT_YPIPE = EEPROM.read(YPIPE_EEPROM_ADDRESS); // 0~255
    CURRENT_CHANNEL = EEPROM.read(CHANNEL_EEPROM_ADDRESS); // 0~90
    RFMode = EEPROM.read(RXTXMODE_ADDRESS);
    if(RFMode == READ_MODE) 
        first_time_read_mode_flag=1;
    NEW_MODE = CURRENT_MODE;
    NEW_KPIPE = CURRENT_KPIPE;
    NEW_YPIPE = CURRENT_YPIPE;
    NEW_CHANNEL = CURRENT_CHANNEL;
    DEVICE_SN_READ = EEPROM.read(SN_ADDRESS);
}

void CHIRON_SERIALBUFFER_TASK(void)
{
  bool mussss=0;
  incomingDataSize = 0;
  incomingDataSize = Serial.available();
  while (Serial.available() > 0) {
    mussss=1;
    char inChar = (char)Serial.read();
    buf += inChar;
    lastMillisCheck = millis();
  }
  /*if(mussss==1&&millis()>5000)
  {
    Serial.print("hard:");
    Serial.println(incomingDataSize);
    Serial.print("real: ");
    Serial.println(buf.length());
    }*/
  //Eger buffer boyutu kucukse
  if(incomingDataSize < 100)
  {
      //Alttaki filtreyle seri porttan veri parcali gelemez.
      if(millis()-lastMillisCheck > 1) checkMillisFlag=1;
      else checkMillisFlag = 0;
  }
  else
  {
      checkMillisFlag = 1;
  }
  
}

void CHIRON_ACKNOWLEDGE(void)
{
    if(buf=="ACK" && checkMillisFlag)
    {
        //--------------------------------------
        Serial.write('M');
        Serial.print(CURRENT_MODE);
        Serial.write('K');
        if(CURRENT_KPIPE<=9){
          Serial.write('0');
          Serial.write('0');
          Serial.print(CURRENT_KPIPE);
        }
        else if(CURRENT_KPIPE <= 99)
        {
          Serial.write('0');
          Serial.print(CURRENT_KPIPE);
        }
        else if(CURRENT_KPIPE <= 999)
        {
          Serial.print(CURRENT_KPIPE);
        }
        Serial.write('Y');
        if(CURRENT_YPIPE<=9){
          Serial.write('0');
          Serial.write('0');
          Serial.print(CURRENT_YPIPE);
        }
        else if(CURRENT_YPIPE <= 99)
        {
          Serial.write('0');
          Serial.print(CURRENT_YPIPE);
        }
        else if(CURRENT_YPIPE <= 999)
        {
          Serial.print(CURRENT_YPIPE);
        }
        Serial.write('C');
        if(CURRENT_CHANNEL <= 9)
        {
          Serial.write('0');
          Serial.print(CURRENT_CHANNEL);
        }
        else if(CURRENT_CHANNEL <= 99)
        {
          Serial.print(CURRENT_CHANNEL);
        }
        Serial.write('R');
        Serial.print(RFMode);
        Serial.write('S');
        if(DEVICE_SN_READ<=9){
          Serial.write('0');
          Serial.write('0');
          Serial.print(DEVICE_SN_READ);
        }
        else if(DEVICE_SN_READ <= 99)
        {
          Serial.write('0');
          Serial.print(DEVICE_SN_READ);
        }
        else if(DEVICE_SN_READ <= 999)
        {
          Serial.print(DEVICE_SN_READ);
        }
        if(DEVICE_SN2_READ<=9){
          Serial.write('0');
          Serial.write('0');
          Serial.print(DEVICE_SN2_READ);
        }
        else if(DEVICE_SN2_READ <= 99)
        {
          Serial.write('0');
          Serial.print(DEVICE_SN2_READ);
        }
        else if(DEVICE_SN2_READ <= 999)
        {
          Serial.print(DEVICE_SN2_READ);
        }
        Serial.println("");
        //------------------------------
        buf="";
    }
}

void CHIRON_BUFFILTER_TASK(void)
{
  //Serial.print("buf: ");
  //Serial.println(buf);
  
  if(buf.length()>9)//12 idi read icin 9 yaptik
  {
      //Serial.print("Length");
      //Serial.println(buf.length());
      
      for(startl = 0; startl <= buf.length() - 5; startl++)
      {
          //Serial.print("Length");
          //Serial.println(buf.length());
          //Serial.println("donuyorum mk");
          if(buf.length()<9)break;//12 idi read icin 9 yaptik
          if(buf.substring(startl, startl+5) == "CHR>E")
          {
              command_detected_flag = 1;
              //Serial.println(startl,DEC);
              filterString = buf.substring(0, startl);
              buf = buf.substring(startl+5,buf.length());
              
              startl = 0;
              //Serial.println(filterString);
              for(startr = 0; startr <= filterString.length() - 5 ; startr++)
              {
                if(filterString.length()<6)break;
                if(filterString.substring(startr, startr+5) == "CHR>S")
                {
                    filterString2 = filterString.substring(startr+5,filterString.length());
                    filterString2 = filterString2.substring(1, filterString2.length()-1);
                    //filterString2 yi nrfden yolla
                    //Serial.println(filterString2);
                    transmit_buf += filterString2;
                }
                else if(filterString.substring(startr, startr+5) == "CHR>C")
                {
                    filterString2 = filterString.substring(startr+5,filterString.length());
                    //Serial.println(filterString2);
                    virgul_sayisi = 0;
                    for(int stC = 0; stC < filterString2.length(); stC++)
                    {
                        if(filterString2.charAt(stC) == ',')
                        {
                            virguller[virgul_sayisi++]=stC;
                        }
                    }
                    //Serial.println("Virgul sayisi");
                    //Serial.println(virgul_sayisi,DEC);
                    if(virgul_sayisi != 3)
                    {
                        undetected_error = 2; //Wrong configuration pattern
                    }
                    else
                    {
                        String configString;
                        unsigned int mode, kpipe, ypipe, channel;
                        
                        configString = filterString2.substring(1,virguller[0]); //Mode

                        mode=configString.toInt();
                        configString = filterString2.substring(virguller[0]+1,virguller[1]); //Kendi pipe
                        
                        
                        kpipe=configString.toInt();
                        configString = filterString2.substring(virguller[1]+1,virguller[2]); //Yollanan pipe
                        ypipe=configString.toInt();
                        configString = filterString2.substring(virguller[2]+1,filterString2.length()-1); //Channel
                        channel=configString.toInt();
                        if(mode == 1 || mode==2 || mode==3)
                        {
                            NEW_MODE = mode;
                        }
                        else
                        {
                            undetected_error = 3; //Mode configuration is faulty.
                        }
                        if(kpipe>=PIPE_MIN && kpipe<=PIPE_MAX)
                        {
                            NEW_KPIPE = kpipe;
                        }
                        else
                        {
                            undetected_error = 4; //Invalid cihaz pipei
                        }
                        if(ypipe>=PIPE_MIN && ypipe<=PIPE_MAX)
                        {
                            NEW_YPIPE = ypipe;
                        }
                        else
                        {
                            undetected_error = 5; //Invalid yollanan pipe
                        }
                        if(channel>CHANNEL_MIN && channel<CHANNEL_MAX)
                        {
                            NEW_CHANNEL = channel;
                        }
                        else
                        {
                            undetected_error = 6; //Invalid channel
                        }
                        //Serial.print("Mode : ");
                        //Serial.println(NEW_MODE);
                        //Serial.print("Kpipe : ");
                        //Serial.println(NEW_KPIPE);
                        //Serial.print("Ypipe : ");
                        //Serial.println(NEW_YPIPE);
                        //Serial.print("Channel : ");
                        //Serial.println(NEW_CHANNEL);
                    }
                    
                }
                else if(filterString.substring(startr, startr+5) == "CHR>B")
                {
                    filterString2 = filterString.substring(startr+5,filterString.length());
                    //Serial.println(filterString2);
                }
                else if(filterString.substring(startr, startr+5) == "CHR>R")
                {
                    //Read RF Data
                    if(RFMode != READ_MODE)
                        EEPROM.write(RXTXMODE_ADDRESS,READ_MODE);
                    RFMode = READ_MODE;
                    first_time_read_mode_flag = 1;
                    buf="";
                    break;
                }
                else if(filterString.substring(startr, startr+5) == "CHR>T")
                {
                    //Read RF Data
                    if(RFMode != TRANSMIT_MODE)
                        EEPROM.write(RXTXMODE_ADDRESS,TRANSMIT_MODE);
                    RFMode = TRANSMIT_MODE;
                    buf="";
                    break;
                }
                else if(filterString.substring(startr, startr+5) == "CHR>V")
                {
                    //Read RF Data
                    if(RFMode != TRANSCEIVE_MODE)
                        EEPROM.write(RXTXMODE_ADDRESS,TRANSCEIVE_MODE);
                    RFMode = TRANSCEIVE_MODE;
                    buf="";
                    break;
                }
		else if(filterString.substring(startr, startr+5) == "CHR>Z")
		{
		    //Software reset
                    //Serial.println("Resetd");
                    buf="";
		    resetFunc();
		}
                else if(filterString.substring(startr, startr+5) == "CHR>D")
                {
                    filterString2 = filterString.substring(startr+5,filterString.length());
                    filterString2 = filterString2.substring(1, filterString2.length()-1);
                    //Serial.println(filterString2);
                    virgul_sayisi = 0;
                    for(int stC = 0; stC < filterString2.length(); stC++)
                    {
                        if(filterString2.charAt(stC) == ',')
                        {
                            virguller[virgul_sayisi++]=stC;
                        }
                    }
                    //Serial.println("Virgul sayisi");
                    //Serial.println(virgul_sayisi,DEC);
                    if(virgul_sayisi != 2)
                    {
                        undetected_error = 7; //Wrong IO pattern
                    }
                    else
                    {
                        String configString;
                        int pin, dir, value;
                        configString = filterString2.substring(0,virguller[0]); //Mode
                        //Serial.println(configString);
                        pin=configString.toInt();
                        configString = filterString2.substring(virguller[0]+1,virguller[1]); //Kendi pipe
                        dir=configString.toInt();
                        configString = filterString2.substring(virguller[1]+1,filterString2.length()); //Yollanan pipe
                        value=configString.toInt();
                        if(pin==1 || pin==2 || pin==3 || pin==4 || pin==5)
                        {}
                        else
                        {
                            undetected_error = 8; //Pin not available 
                        }
                        if(dir==0 || dir==1 || dir==2)
                        {}
                        else
                        {
                            undetected_error = 9; //Invalid direction data
                        }
                        //Right PWM Configuration
                        if(dir==2 && (pin==1 || pin==3 || pin==4) && (value>=0 && value<256))
                        {
                            if(dir == PINS_DIR_ARR[pin-1])
                            {
                                analogWrite(PINS[pin-1],value);
                            }
                            else
                            {
                                pinMode(PINS[pin-1],OUTPUT);
                                analogWrite(PINS[pin-1],value);
                                PINS_DIR_ARR[pin-1] = 2; //PWM
                            }
                        }
                        else
                        {
                            undetected_error = 10; //Wrong Pwm Configuration
                        }
                        //Right GPIO Configuration Output
                        if((dir==1) && (value==0 || value==1) && (pin==1 || pin==2 || pin==3 || pin==4 || pin==5))
                        {
                            if(dir == PINS_DIR_ARR[pin-1])
                            {
                                digitalWrite(PINS[pin-1],(value)?:(HIGH, LOW));
                            }
                            else{
                                pinMode(PINS[pin-1],(dir)?:(OUTPUT,INPUT));
                                digitalWrite(PINS[pin-1],(value)?:(HIGH, LOW));
                                PINS_DIR_ARR[pin-1] = dir; // IO
                            }
                        }
                        else
                        {
                            undetected_error = 11; //Wrong GPIO Configuration
                        }
                        if((dir==0) && (pin==1 || pin==2 || pin==3 || pin==4 || pin==5))
                        {
                            pinMode(PINS[pin-1],(dir)?:(OUTPUT,INPUT));
                            Serial.print("CHR>PD\"");
                            Serial.print(pin);
                            Serial.print(",");
                            if(digitalRead(PINS[pin-1])==HIGH)
                                Serial.print("1");
                            else
                                Serial.print("0");
                            Serial.print("\"CHR+E");
                        }
                        else
                        {
                            undetected_error=12; //Wrong Read Config
                        }
                        if((dir==3) && (pin==1 || pin==2 ||pin==3))
                        {
                            Serial.print("CHR>PA\"");
                            Serial.print(pin);
                            Serial.print(",");
                            Serial.print(analogRead(pin));
                            Serial.print("\"CHR>E");
                        }
                        else
                        {
                          
                        }
                    } 
                }
                else
                {
                    undetected_error = 1; // Non recognizable input
                }
             }
          }
          else
          {
              /*Serial.print(buf.length());
              Serial.print("+");
              Serial.println(startl);
              if(RFMode == TRANSMIT_MODE && startl==buf.length())
              {
                  //Serial.print("X");
                  transmit_buf += buf;
                  buf="";
              }
            
              if(buf.length()>1022)
              {
                buf="";
              }*/
          }
      }
      //Serial.println("Isim bitti");
  }
  else if(buf.length()>0)
  {
      /*if(RFMode == TRANSMIT_MODE)
      {
          
          //Serial.print("X1");
          transmit_buf += buf;
          buf="";
      }*/
  }
  
  if(command_detected_flag == 0 && (RFMode == TRANSMIT_MODE || RFMode == TRANSCEIVE_MODE))
  {
      transmit_buf += buf;
      buf="";
  }
  command_detected_flag = 0;
}

void CHIRON_RFREAD_TASK(void)
{
    if(RFMode == READ_MODE)
    {
        if(first_time_read_mode_flag ==1)
        {
            radio.startListening();
            //Serial.println("Okuma moduna gecildi..");
            first_time_read_mode_flag = 0;
        }
        //Alttaki if'e girilmiyor
        if(radio.available())
        {
            bool done = false;
            while(!done)
            {
                done = radio.read( prdata, sizeof(rdata));
                for(int k=0; k<rdata[PAYLOAD_BYTES-2]; k++)
                {
                    Serial.write(rdata[k]);
                }
                for(int k=0; k<=PAYLOAD_BYTES-1; k++)
                {
                    rdata[k]=0;
                }
                delay(RECEIVE_DELAY);
            }
        }
    }
}



void CHIRON_RFSEND_TASK(void)
{
  if(RFMode == TRANSMIT_MODE)
  {
      //Serial.print("transmit_buf: ");
      //Serial.println(transmit_buf);
      while(transmit_buf.length() > 0)
      {   
          //Serial.println("xs");
          for(int a=0; a<=PAYLOAD_BYTES-3; a++)
          {
              data[a] = transmit_buf.charAt(a);
          }
          if(transmit_buf.length()>30)
            data[PAYLOAD_BYTES-2] = 30;
          else
            data[PAYLOAD_BYTES-2] = transmit_buf.length();

          radio.stopListening();
          
          nrfwaitMillis = millis();
          while(millis()-nrfwaitMillis<5)
          {
             CHIRON_SERIALBUFFER_TASK(); 
          }
          
          bool ok = radio.write( pdata, sizeof(data) );
          
          if (ok)
          {
            /*for(int k=0; k<=29; k++)
            {
                Serial.print(data[k]);
            }*/
            //Serial.print("ok...");
            transmit_buf = transmit_buf.substring(data[PAYLOAD_BYTES-2], transmit_buf.length());
          }
          else
          {
            failed_count = failed_count + 1;
            //Serial.print("buf:");
            //Serial.println(buf.length());
            Serial.print("Failed.");
          }
          
          if(failed_count > 5)
          {
              //Gonderilemedi
              failed_count = 0;
              transmit_buf = "";
          }
      
          // Now, continue listening
          //radio.startListening();
          nrfwaitMillis = millis();
          while(millis()-nrfwaitMillis<AFTERSEND_DELAY)
          {
             CHIRON_SERIALBUFFER_TASK(); 
          }
      }
   }
}



void CHIRON_TRANSCEIVE_TASK(void)
{
    if(RFMode == TRANSCEIVE_MODE)
    {
        //Serial.println(buf.length());
        //Serial.println(transmit_buf.length());
        while(transmit_buf.length() > 0)
        {   
            //Serial.println("C");
            for(int a=0; a<=PAYLOAD_BYTES-3; a++)
            {
                data[a] = transmit_buf.charAt(a);
            }
            if(transmit_buf.length()>30)
              data[PAYLOAD_BYTES-2] = 30;
            else
              data[PAYLOAD_BYTES-2] = transmit_buf.length();
            
            radio.stopListening();
            delay(5);
  
            bool ok = radio.write( pdata, sizeof(data) );
            
            if (ok)
            {
              /*for(int k=0; k<=29; k++)
              {
                  Serial.print(data[k]);
              }*/
              //Serial.print("ok...");
              transmit_buf = transmit_buf.substring(data[PAYLOAD_BYTES-2], transmit_buf.length());
            }
            else
            {
              failed_count = failed_count + 1;
              //Serial.print("buf:");
              //Serial.println(buf.length());
              Serial.print("Failed.");
            }
            
            if(failed_count > 5)
            {
                //Gonderilemedi
                failed_count = 0;
                transmit_buf = "";
            }   
        }
        
        // Now, continue listening
        radio.startListening();
        
        delay(AFTERSEND_DELAY);
        
        if(radio.available())
        {
            bool done = false;
            while((!done) && transmit_buf.length()==0)
            {
                done = radio.read( prdata, sizeof(rdata));
                for(int k=0; k<rdata[PAYLOAD_BYTES-2]; k++)
                {
                    Serial.write(rdata[k]);
                }
                for(int k=0; k<=PAYLOAD_BYTES-1; k++)
                {
                    rdata[k]=0;
                }
                delay(RECEIVE_DELAY);
            }
        }
    }
}
