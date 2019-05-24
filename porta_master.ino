/* ===============================================================================================
  LoRa HalfDuplex communication wth callback - 13/06/18

  MASTER - teste

  Sends a message every  8-10 seconds, and polls SLAVE (see slave.ino) every 4-5 seconds
  Uses callback for new incoming messages. 
  Not implements callback for transmission (yet!!)

  Formats:

  enq = x05 - 1 byte
  ack = x06 - 1 byte

  msg { 
   header  = 1byte (xFF)
   payloadlen = 1byte (0..xFF)
   payload  = array byte [255]
   crc8      = 1 byte1              //Fast CRC8
   
   //  add in the LoRa link layer  https://www.libelium.com/forum/viewtopic.php?t=24813 - INVESTIGAR - ESTA ALTERANDO O PAYLOD !!!
  }

  Note: while sending, LoRa radio is not listening for incoming messages.
  Note2: when using the callback method, you can't use any of the Stream
  functions that rely on the timeout, such as readString, parseInt(), etc.

  created by Evaldo Nascimento and Paulo Abreu in 27 may 2018
  based on the original version by Tom Igoe - https://github.com/tigoe/NetworkExamples
  
================================================================================================= */

#define OLED            1   // 0 = no OLED display; 1 = otherwise
#define DEBUG           2   // 0 = no debug messages; 1 = error messages only ; 2 = all debug messages

// include libraries
#include <SPI.h>            // comunication with radio LoRa 
#include <LoRa.h>           // radio LoRa sx1276 lib https://github.com/sandeepmistry/arduino-LoRa
#include <Wire.h>           // comunication i2c

#if OLED > 0
#include "SSD1306.h"        // comunication with display OLED SSD1306 https://github.com/ThingPulse/esp8266-oled-ssd1306
#endif

#include <WiFi.h>           // Wi-Fi coonectivity
#include <NTPClient.h>      // for ntp clock synchronization https://github.com/arduino-libraries/NTPClient
#include <WiFiUdp.h>        // for ntp clock synchronization
//#include <AWS_IOT.h>        // AWS IoT interface
#include <FastCRC.h>        // lib for CRC calculations https://github.com/FrankBoesing/FastCRC
#include <TimeLib.h>        // time functions used with ntp time https://github.com/PaulStoffregen/Time
//AWS_IOT PPP_Master;
FastCRC8 CRC8;

#include <ArduinoJson.h>     // Json library to send data to AWS (use version 5. Do not use version 6) by Benoit Blanchon
/*
                      JSON message we want to send:
                      prettified:
                      {
                        "waterTempTimestamp": CP1CP2CP3CP4,  ---> 4 bytes representing time
                        "waterTempValue": b1b2               ---> 2 bytes (value is in temp1)
                      }

                      minified:
                      {"waterTempTimestamp": 5B3CD58B, "waterTempValue": 345}
*/



// sensor de temperatura AWS do Evaldo
//char HOST_ADDRESS[]="aljq8kgezlxbs.iot.us-east-1.amazonaws.com";
//char CLIENT_ID[]= "MeuSensorTempClient";
//char TOPIC_NAME[]= "$aws/things/TemperatureSensor/shadow/update";


/*
// sensor de temperatura AWS do Paulo
char HOST_ADDRESS[]="amnuodn317xr5.iot.us-west-2.amazonaws.com";
char CLIENT_ID[]= "SensorTemperaturaESP32";
char TOPIC_NAME[]= "$aws/things/sensorTemperatura1/shadow/update";
*/

 
// Protocol command definitions 
#define HEADER     0xFF        // header da mensagem
#define ENQ        0x05        // ASCII ENQ - enquire command
#define ACK        0x06        // ASCII ACK - acknoledge response
//#define GETTIME     0X01     //  slave asks ntp time to master

#define ALARM             0x01   // the message is an alarm

#define COMMAND           0x02   // Msgtype = byte 02H - COMMAND to the slave
#define   GPIOSET           0x00 // gpio setup (2 additional bytes: gpio_pin and gpio_value)

// mensagem que vai chegar do sensor_porta:
//      RESP        = indica que a mensagem é uma resposta enviada pelo Slave.
//      SLAVEID     = ID que identifica o Slave que enviou a mensagem.
//      ALARMID     = qual é o tipo de alarme que o Slave está informando.
//      ALARMVALUE  = qual é o valor do alarme informado.

// definições compatíveis com o sensor_porta
#define RESP              0x03   // Msgtype = byte 03H - RESP - response

// SLAVEID vai ser transmitido por sensor_porta

// definições de ALARMID compatíveis com sensor_porta
#define   ALARMEPORTA       0x01

// definições de ALARMVALUE compatíveis com sensor_porta
#define   PORTAABERTA       0x01
#define   PORTAFECHADA      0x02

// depois apagar tudo isso aqui.
#define   motor_on          0x01 // motor_on              OK|NG    byte   (OK=01, NG=00)
#define   motor_off         0x02 //- motor_off             OK|NG   
#define   water_level       0x03 //- water_level        OK|NG   
#define   WATER_TEMPERATURE 0x04 //- WATER_TEMPERATURE  XX     byte   (XX temp oC)
#define   current_ac_motor  0x05 //- current-ac_motor   YY.Y      byte   (XX Amps) 
#define   master_reset      0x06 //- master-reset           OK|NG    
#define   load_motor_table  0x07 //- load_motor_table   OK|NG   
#define   show_status       0x08 //- show_status           OK|NG  - transferir arquivo texto “status.txt”
// Alarm IDs
#define NOWATER   0x01    //
#define NOCLOCK   0x20    // slave clock is out of synch




#define NTPTIME		        0x09 // payload has 4 bytes with ntp time


// Pinout definition
#define SCK     5          // GPIO5  -- SX127x's SCK
#define MISO    19         // GPIO19 -- SX127x's MISO
#define MOSI    27         // GPIO27 -- SX127x's MOSI
#define SS      18         // GPIO18 -- SX127x's CS
#define RST     14         // GPIO14 -- SX127x's RESET
#define DI00    26         // GPIO26 -- SX127x's IRQ(Interrupt Request)

// LoRa frequency definition
#define BAND    915E6       // Lora Radio frequency -  433E6 for China, 868E6 for Europe, 915E6 for USA, Brazil

// Variables definitions
byte    TxmsgCount =                    0;              // counter of outgoing messages
byte    RxmsgCount =                    0;              // counter of incomming messages
long    lastSendTime, lastSendTime2 =   0;              // last send time
int     interval, interval2 =        2000;              // interval between sends
int     i =                             0;              // general counter

/*
 *     rx_buffer is the buffer used to store data received through the LoRa radio.
 *     
 *      +---+---+---+---+---+---+---+...+---+
 *      |   |   |   |   |   |   |   |   |   |
 *      +---+---+---+---+---+---+---+...+---+
 *        |           |
 *        |           +-- rx_buffer_head
 *        |
 *        +-- rx_buffer_tail
 *  
 *      If rx_buffer_tail != rx_buffer_head, then something was received by the LoRa radio.
 *      Otherwise rx_buffer_tail == rx_buffer_head, meaning the buffer is empty (nothing else was received).
 *      
 *      When reading content from rx_buffer, rx_buffer_tail is incremented until 
 *      it reaches rx_buffer_head. This is a FIFO (First In First Out) mechanism.
 *      
 *      When writing content to rx_buffer, rx_buffer_head is incremented until
 *      the difference (rx_buffer_head - rx_buffer_tail) contains the message received.
 *  
 *      rx_buffer is a circular buffer, so when rx_buffer_head == 255, rx_buffer_head++
 *      will point to the first position of the buffer (rx_buffer[0]). Same for rx_buffer_tail.
 *      
 */
byte    rx_buffer[256];                                 // holds received bytes in callback function
byte    rx_byte;                                        // holds 1 received byte
byte    rx_buffer_head =                0;              // array index of rx_buffer  in callback  onReceive function
byte    rx_buffer_tail =                0;              // array index of rx_buffer  in foreground  
byte    p_rx_msg =                      0;              // array index of RX_msg 
byte    RX_msg [256];                                   // holds received bytes in foreground functions

// test - FF - 10 bytes - ABCDEFGHIJ
byte    tx_test[] =                    {0xFF, 10, 65, 66, 67, 68, 69, 70, 71, 72, 73,74}; 

/*
 *    tx_buffer is the buffer used to store data that will be sent through the LoRa radio.
 *    
 *      +---+---+---+---+---+---+---+...+---+
 *      |   |   |   |   |   |   |   |   |   |
 *      +---+---+---+---+---+---+---+...+---+
 *        |           |
 *        |           +-- tx_buffer_head
 *        |
 *      tx_buffer_tail
 *  
 *      If tx_buffer_tail != tx_buffer_head, then something is ready to be transmitted by Master.
 *      Otherwise tx_buffer_tail == tx_buffer_head, meaning the buffer is empty (there is nothing else to be transmitted).
 *      
 *      When reading content of tx_buffer (in order to transmit that byte), tx_buffer_tail is incremented until 
 *      it reaches the value of tx_buffer_head. This is a FIFO (First In First Out) mechanism.
 *      
 *      When writing content into tx_buffer (preparing for later tranmission), tx_buffer_head is incremented until
 *      the difference (tx_buffer_head - tx_buffer_tail) contains the message to be transmitted.
 *      
 *      tx_buffer is a circular buffer, so when tx_buffer_head == 255, tx_buffer_head++
 *      will point to the first position of the buffer (tx_buffer[0]). Same for tx_buffer_tail.
 *
 */
byte    tx_buffer[256];
byte    tx_byte;                                        // holds 1 transmitted  byte in foreground sendBuffer function
byte    tx_buffer_head =                240;            // array index of tx_buffer  set before calling sendBuffer function
byte    tx_buffer_tail =                240;            // array index of tx_buffer  in "callback" actually: foreground sendBuffer function. Callback for transmission not implemented yet 
byte    msglength=                      0;              // size of received message
byte    crc =                           0;
byte    SSM_Status =                    0;              // SSM - Software State Machine controler initial status

int status = WL_IDLE_STATUS;
int tick=0,msgCount=0,msgReceived = 0;
char payload[512];
char rcvdPayload[512];

#if OLED > 0
//parameters: address,SDA,SCL 
SSD1306 display(0x3c, 4, 15);   //define OLED object
#endif

//String rssi = "RSSI --";
//String packSize = "--";
//String packet ;

// ssid and password of WiFi network you will connect to
/*   put the 2 lines below in a file and call it a name ------
const char *ssid     = "put your wifi ssid here";
const char *password = "put your wifi password here";
----  then the line below will include it in the code (adjust the path to your case) ---------*/
#include "/home/paulo/wifi_credenciais.txt" // wi-fi do Paulo


WiFiUDP ntpUDP; // creates a UDP instance to send and receive packets
int16_t utc = -3; // UTC -3:00 for Brasilia time

// Defines ntp parameters:
// Usage: NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);
// a.st1.ntp.br == NIC.br ntp server
NTPClient timeClient(ntpUDP, "a.st1.ntp.br", utc*3600, 60000);

unsigned long ntpTime; // to capture returned value of function call for ntp time

// CP1 to CP4 are used to store ntp time, byte by byte
unsigned char CP1 = 0;		// most significant byte
unsigned char CP2 = 0;
unsigned char CP3 = 0;
unsigned char CP4 = 0;		// least significant byte


// Para que server esta função?
void mySubCallBackHandler (char *topicName, int payloadLen, char *payLoad)
{
    strncpy(rcvdPayload,payLoad,payloadLen);
    rcvdPayload[payloadLen] = 0;
    msgReceived = 1;
}

//-------------------------------------------------------------------
// callback function setup to process  data received by the LoRa radio
// TO-DO: fazer função retornar TRUE ou FALSE ou algo assim
//
void onReceive(int packetSize) {
  if (packetSize == 0) return;                          // if there's no packet, return
#if DEBUG >= 2
  Serial.println("Pointers na entrada da função onReceive():");
  Serial.println("onReceive::  rx_buffer_head = " + String(rx_buffer_head) );
  Serial.println("onReceive::  rx_buffer_tail = " + String(rx_buffer_tail) );
#endif  
  do {
    rx_byte = LoRa.read();                              // receives one byte
    rx_buffer[rx_buffer_head] = rx_byte;                // salves received byte in the rx_buffer
    rx_buffer_head++;                                   // points to the next position in the rx_buffer
  } while (LoRa.available());                           // until all bytes are received
#if DEBUG >= 2
  Serial.println("Pointers na saída da função onReceive():");
  Serial.println("onReceive::  rx_buffer_head = " + String(rx_buffer_head) );
  Serial.println("onReceive::  rx_buffer_tail = " + String(rx_buffer_tail) );
#endif
}

//--------------------------------------------------------------------
// send just 1 byte - used to send commands and responses (ENQ, ACK)
void sendByte (byte tx_byte) {   
  LoRa.beginPacket();                                   // start packet
  LoRa.write(tx_byte);                                  // add byte to be transmitted
  LoRa.endPacket();                                     // finish packet and send it
  LoRa.receive();                                       // you must make Lora listen to reception again !!
}

//----------------------------------------------------------------
// Function used to calculate the length of the message
// stored at the rx_buffer or tx_buffer
byte difference (byte b, byte a) {
  if ((b-a)<0) {
    return (b-a+256);
  }else{
    return (b-a);
  }
}

//-----------------------------------------------------------------
// sendBuffer(): send a buffer of bytes
//
void sendBuffer() {
  byte temp_buffer [256];
  byte i = 0 , j = 0, crcCalculated =0;
  byte buffer_size = 0;
  
  // add crc at the end of the txbuffer
  j = tx_buffer_tail;
  buffer_size = difference (tx_buffer_head, tx_buffer_tail);
  for (i = 0; i< buffer_size; i++) {
    temp_buffer[i] = tx_buffer [j];
    j++;
  }
  crcCalculated = CRC8.smbus(temp_buffer, buffer_size); 
  
#if DEBUG >= 2
  Serial.print(theDateIs()); Serial.println("sendBuffer::  tamanho do nuffer = "+ String(difference (tx_buffer_head, tx_buffer_tail),HEX));
#endif
  tx_buffer[tx_buffer_head++] = crcCalculated;             // add the crc at the end of the msg to be transmitted
#if DEBUG >= 2
  Serial.print(theDateIs()); Serial.println("sendBuffer::  CRC = "+ String(crcCalculated,HEX));
#endif
#if DEBUG >= 1
  Serial.print(theDateIs()); Serial.println("sendBuffer:: Sending LoRa message");
#endif
  LoRa.beginPacket();                                  // start packet
  do {
     tx_byte = tx_buffer[tx_buffer_tail++];              // get the byte from the buffer
#if DEBUG >= 2
     Serial.print(theDateIs()); Serial.println("sendBuffer:: TxByte= " + String(tx_byte,HEX)+" txhead= " + String(tx_buffer_head)+" txtail= " + String(tx_buffer_tail));
#endif
     LoRa.write(tx_byte);                              // add byte to be transmitted
  } while (tx_buffer_tail != tx_buffer_head);         // while these 2 pointers are different, there are bytes to be trasnmitted
  LoRa.endPacket();                                   // finish packet and send it
  LoRa.receive();                                     // you must make Lora listen to reception again !!
  TxmsgCount++;                                       // increment Tx message counter
#if DEBUG >= 1
  Serial.print(theDateIs()); Serial.println("sendBuffer:: LoRa message sent");
#endif

#if OLED > 0
    // displays in the OLED TX and RX msgs counters
    display.clear();
    display.drawString(0, 0, "Master Tx to Slave");
    display.drawString(40, 13, String(TxmsgCount));
    display.drawString(0, 26, "Master Rx from Slave");
    display.drawString(40, 39, String(RxmsgCount));
    display.display(); 
#endif
}


// ------------------------------ received() --------------------------------
// check if there are bytes in the Rx buffer
// returns:
//    FALSE se nada foi recebido em rx_buffer[]
//    TRUE  se algo foi recebido em rx_buffer[] e o CRC está correto (ou seja, a mensagem é válida).
//
// A mensagem recebida (se for somente uma mensagem...) começa 
// na posição apontada por rx_buffer_tail e termina na posição rx_buffer_head-1
// (rx_buffer_head aponta para a próxima posição livre no buffer).
// O CRC está na última posição da mensagem, portanto está na
// posição rx_buffer_head-1.
//
boolean received() {

  byte crcReceived, crcCalculated=0;
  byte temp_buffer [256];
  byte i = 0 , j = 0;
  int  buffer_size = 0;

  if (rx_buffer_head != rx_buffer_tail) { // há algo no rx_buffer
    crcReceived = rx_buffer[rx_buffer_head-1];
#if DEBUG >= 2
      Serial.print(theDateIs()); Serial.print("received::rx_buffer_head = "); Serial.println(rx_buffer_head);
      Serial.print(theDateIs()); Serial.print("received::rx_buffer_tail = "); Serial.println(rx_buffer_tail);
      Serial.print(theDateIs()); Serial.print("received::CRC recebido = "); Serial.println(crcReceived, HEX);
#endif
      j = rx_buffer_tail;
      buffer_size = difference (rx_buffer_head, rx_buffer_tail);
#if DEBUG >= 2
      Serial.print(theDateIs()); Serial.print("received::buffer size = ");Serial.println (buffer_size);
      Serial.print(theDateIs()); Serial.print("received::rx_buffer[] = ");
      for (i = 0; i < (buffer_size); i++) {
          //Serial.print("j = ");Serial.print(j); Serial.print("; i = ");Serial.println(i);
          Serial.print(rx_buffer[j],HEX);Serial.print("-");
          j++;
      }
      Serial.println();
#endif      
      //buffer_size--; // não queremos copiar o CRC para o buffer local pois calcularemos o CRC a partir dos dados recebidos

      // copia rx_buffer (global) para temp_buffer (local) sem o CRC
      j = rx_buffer_tail;
      for (i = 0; i < (buffer_size-1); i++) {
          temp_buffer[i] = rx_buffer [j];
          j++;
      }

#if DEBUG >= 2
      // Mostra temp_buffer[] para conferência.
      Serial.print(theDateIs()); Serial.print("received::temp_buffer[] (sem CRC) = ");
      for (i = 0; i < buffer_size-1; i++) {
        Serial.print(temp_buffer[i],HEX);Serial.print("+");
      }
      Serial.println();
#endif  
      //buffer_size--;
      Serial.print(theDateIs()); Serial.print("received::buffer size sem CRC = "); Serial.println (buffer_size-1);
      crcCalculated = CRC8.smbus(temp_buffer, buffer_size-1); 
      Serial.print(theDateIs()); Serial.print("received::crcCalculated = "); Serial.println(crcCalculated,HEX);
      if (crcCalculated != crcReceived) {   // if we had a crc error
         rx_buffer_tail = rx_buffer_head;    // discard the received msg (ASSUME QUE SÓ 1 MENSAGEM FOI RECEBIDA!)
#if DEBUG >= 1
         Serial.print(theDateIs()); Serial.println("received::CRC ERROR - mensagem apagada!");
#endif
         return (false);
      } else {
          Serial.print(theDateIs()); Serial.println("Há dados válidos em rx_buffer[].");
          return(true);
      }
      
  } else {
      // não há nada em rx_buffer[]
      return(false);
  }
  //return (rx_buffer_head != rx_buffer_tail); // TRUE=?
}  // end of received()



// ------------------------------ theDateIs() --------------------------------
// Use the internal (adjusted by ntp) clock to create a string
// with the current date and time.
// Tipically used for debugging.
String theDateIs() {
    String dateNow;
    int weekNow = weekday();
    switch(weekNow) {
        case 1:
            dateNow = dateNow + "Domingo ";
            break;
        case 2:
            dateNow = dateNow + "Segunda-feira ";
            break;
        case 3:
            dateNow = dateNow + "Terça-feira ";
            break;
        case 4:
            dateNow = dateNow + "Quarta-feira ";
            break;
        case 5:
            dateNow = dateNow + "Quinta-feira ";
            break;
        case 6:
            dateNow = dateNow + "Sexta-feira ";
            break;
        case 7:
            dateNow = dateNow + "Sábado ";
            break;
    } // end switch

    if (day() < 10) {
        dateNow = dateNow + "0" + String(day()) + "/";
    } else {
        dateNow = dateNow + String(day()) + "/";
    }
    if (month() < 10) {
        dateNow = dateNow + "0" + String(month()) + "/";
    } else {
        dateNow = dateNow + String(month()) + "/";
    }
    dateNow = dateNow + String(year()) + " ";
    if (hour() < 10) {
        dateNow = dateNow + "0" + String(hour()) + ":";
    } else {
        dateNow = dateNow + String(hour()) + ":";
    }
    if (minute() < 10) {
        dateNow = dateNow + "0" + String(minute()) + ":";
    } else {
        dateNow = dateNow + String(minute()) + ":";
    }
    if (second() < 10) {
        dateNow = dateNow + "0" + String(second()) + " ";
    } else {
        dateNow = dateNow + String(second()) + " ";
    }
    return dateNow;
} // end theDateIs()


/* ---------------------------------------------------------------------------------------
 *  teste de comando do master para o slave
 *  vamos implementar uma função que envia uma msg de comando para o slave
 *  quando receber a respectiva msg da nuvem
 *  1- verifica se chegou msg da nuvem
 *  2- se chegou e é um comando:
 *  3- monta e transmite a msg ao slave
 *  4- se recebeu ack em xx segundos OK
 *  5 - senão (recebeu Nak ou não recebeu nada) retransmite 3x
 *  6- se conseguiu transmitir OK segue processamento
 *  7- senão sinaliza erro (OLED, console e nuvem)
 *  Msgtype    = byte - 02H = COMMAND
 *  Command_ID = byte - 00H = GPIOSET
 *  gpio_pin   = byte - valores validos {22, 23, 17, 21, 12, 13, 25(buildin LED), 32..39}
 *  gpio_value = byte - 00 or 01
 *  colocada em setup para testes, depois deverá ser adicionada a SSM
 ------------------------------------------------------------------------------------------ */

// ------------------------------ gpioSetup() --------------------------------
// Pra que serve essa função mesmo?
/* ---------------------------------------------------------------------------------------
 *  teste de comando do master para o slave
 *  colocada em setup para testes, depois deverá ser adicionada a SSM
 ------------------------------------------------------------------------------------------ */
void gpioSetup (byte gpio_pin, byte gpio_value) {
  tx_buffer[tx_buffer_head++] = HEADER;     // starts with a HEADER
  tx_buffer[tx_buffer_head++] = 0x04;       // payload length
  tx_buffer[tx_buffer_head++] = COMMAND;    // type of payload  02H = COMMAND to slave
  tx_buffer[tx_buffer_head++] = GPIOSET;    // type of command  00H = GPIOSET
  tx_buffer[tx_buffer_head++] = gpio_pin;   // valores validos {22, 23, 17, 21, 12, 13, 25(buildin LED), 32..39}
  tx_buffer[tx_buffer_head++] = gpio_value; // 00 or 01
  sendBuffer();                             // transmitts to slave
#if DEBUG >= 1
  Serial.print(theDateIs());
  Serial.println("gpioSetup::  Seting the GPIO up no slave?");
#endif
} // end of gpioSetup


// ------------------------------------------------------- setup  ----------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial);
#if DEBUG >= 1

  Serial.println("setup::Serial OK.");

#endif
#if OLED > 0
  // OLED display reset
  pinMode(16,OUTPUT);       // GPIO16 as output
  digitalWrite(16, LOW);    // resets OLED
  delay(50);                // recommendation is to wait at least 5 ms
  digitalWrite(16, HIGH);   // kkeps GPIO16 high during OLED usage
  // initializes the OLED
  display.init(); 
  display.flipScreenVertically(); 
  display.setFont(ArialMT_Plain_10);
  // Displays that we will connect to Wi-Fi
  display.drawString(0, 0, "Conectando ao Wi-Fi");
  display.display();
#endif
#if DEBUG >= 1
    Serial.print("Conectando ao Wi-Fi ");
#endif

  // Connects to Wi-Fi
  WiFi.begin(ssid, password);

  // aguarda a conexão à rede Wi-Fi
  // Sai do loop se 60 * 500ms = 30s se passarem sem
  // que conexão seja estabelecida.
  int count = 0;
  while ( WiFi.status() != WL_CONNECTED && count < 60) {
    // simulates a bar in the display (with the letter o) to show Wi-Fi progress
    delay ( 500 );
    count++;
#if OLED > 0
    display.drawString(count, 10, "o");
    display.display();
#endif
    Serial.print(".");
  }
  if (count == 60) {
#if OLED > 0
    display.clear();
    display.drawString(0, 0, "Wi-Fi - conexão falhou");
    display.display();
#endif
#if DEBUG >= 1

    Serial.println("setup::  Wi-Fi - conexão falhou.");
    Serial.println("Sistema congelado!");

#endif
    while(true);    // infinite loop due to failure
  } else {
#if OLED > 0
    display.clear();
    display.drawString(0, 0, "Conectado ao Wi-Fi");
    display.drawString(0, 10, WiFi.SSID());               // mostra ssid no display
    display.drawString(0, 20, WiFi.localIP().toString()); // mostra IP no display
    display.display();
#endif
    Serial.println("Wi-Fi conectado!");
  }

  // initializes ntp client
  Serial.println("Iniciando cliente ntp.");
  timeClient.begin();
  timeClient.update();
  delay(5000);                          // se não esperarmos, forceUpdate() não atualiza.
  timeClient.forceUpdate();             // se não for chamada, data e horário podem ficar errados (vai entender...).
  ntpTime = timeClient.getEpochTime();  // obtém data e horário do serviço ntp.
  setTime(ntpTime);                     // acerta o relógio local do ESP32.
#if DEBUG >= 1
  Serial.print("setup::relógio ajustado para: ");
  String stringDate = String(weekday()) + " " + String(day()) + "/" + String(month()) + "/" + String(year()) + " " + String(hour()) + ":" + String(minute());
  Serial.print("setup::");
  Serial.println(stringDate);
#endif

/*
// depois apagar isso. Não estamos enviando o horário para o slave.
  // stores ntp time in byte units (to send to slave via LoRa)
  CP1 = (ntpTime & 0xff000000UL) >> 24;		// most significant byte
  CP2 = (ntpTime & 0x00ff0000UL) >> 16;
  CP3 = (ntpTime & 0x0000ff00UL) >>  8;
  CP4 = (ntpTime & 0x000000ffUL)      ;		// least significant byte

#if DEBUG >= 2
  Serial.println("Variáveis do ntp:");
  Serial.print("CP1 = 0x"); Serial.println(CP1, HEX);
  Serial.print("CP2 = 0x"); Serial.println(CP2, HEX);
  Serial.print("CP3 = 0x"); Serial.println(CP3, HEX);
  Serial.print("CP4 = 0x"); Serial.println(CP4, HEX);
#endif
*/
  /*
      How to convert EpochTime to FormattedTime:
      From https://github.com/arduino-libraries/NTPClient/blob/master/NTPClient.cpp
   */
/*  unsigned long hours = (ntpTime % 86400L) / 3600;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);
  unsigned long minutes = (ntpTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);
  unsigned long seconds = ntpTime % 60;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);
  Serial.print("The time now is: "); Serial.println(hoursStr + ":" + minuteStr + ":" + secondStr);
*/
  delay(3000);
#if OLED > 0
  display.clear();
  display.drawString(0, 0, "MASTER OK!");
  display.display(); 
  delay(3000);
  display.clear();      // clears text in OLED display
#endif

  Serial.println("Iniciando rádio LoRa.");
  // overide the default CS, reset, and IRQ pins (optional)
  SPI.begin(SCK,MISO,MOSI,SS);                    // start SPI with LoRa
  LoRa.setPins(SS, RST, DI00);                    // set CS, reset, IRQ pin

  // initialize ratio at BAND MHz
  if (!LoRa.begin(BAND)) {
#if DEBUG >= 1
    Serial.print(theDateIs());      
    Serial.println("setup::LoRa falhou na inicialização.");
#endif
#if OLED > 0
    display.drawString(0, 0, "LoRa - inicialização falhou!");
    display.display();                  
#endif
    while (true);                                 // if failed, do nothing forever
  }

  // setup callback function for reception
  LoRa.onReceive(onReceive);
  LoRa.receive();
#if DEBUG >= 1
  Serial.print(theDateIs());
  Serial.println("LoRa OK");
#endif
#if OLED > 0
  display.drawString(0, 0, " LoRa OK!");
  display.display();
#endif
  delay (2000);



//  acho que não precisamos disso.
/* ---------------------------------------------------------------------------------------
 *  teste de comando do master para o slave
 *  colocada em setup para testes, depois deverá ser adicionada a SSM
 ------------------------------------------------------------------------------------------ */
/*  gpioSetup (25, 1);
#if DEBUG >= 2
  Serial.print(theDateIs());
  Serial.println("gpioSetup::  Seting the LED on !!");
#endif
  delay (200);
  gpioSetup (25, 0);
#if DEBUG >= 2
  Serial.print(theDateIs());
  Serial.println("gpioSetup::  Seting the LED off !!");
#endif
*/



  
  SSM_Status = 0; // initial status of the SSM

} // end of setup()


//-------------------------------------------------------- loop -------------------------------------------------------------

void loop() {

  // SSM (software State Machine)
  switch (SSM_Status) {

      case 0:
      // Estado inicial da Máquina de Estados.
      // Testa se os dados são válidos (não há erros de CRC) e processa o
      // primeiro byte da mensagem.
  
      		if (received()) { // se há dados válidos em rx_buffer[]
        			switch (rx_buffer[rx_buffer_tail]) {   
              
  				        case RESP:
                      // Temos uma mensagem (RESPosta) vinda de um Slave.
                      // Os próximos 4 bytes são SLAVEID, ALARMID, ALARMVALUE e CRC.
#if DEBUG >= 2
                      Serial.print(theDateIs());Serial.println("loop::RESP recebido de um Slave.");
                      //Serial.print(theDateIs());Serial.print("loop::SLAVEID = ");Serial.println(rx_buffer[rx_buffer_tail+1]);
                      //Serial.print(theDateIs());Serial.print("loop::ALARMID = ");Serial.println(rx_buffer[rx_buffer_tail+2]);
                      //Serial.print(theDateIs());Serial.print("loop::ALARMVALUE = ");Serial.println(rx_buffer[rx_buffer_tail+3]);
#endif                    
                      // prepara para processar o próximo byte da mensagem
                      rx_buffer_tail++; 
                      SSM_Status = 1;
                      break; // case RESP
					
      			  } // end of switch
    		  } // end of 'if( received() )'
          break; // case 0


    case 1:
    // Estado anterior deveria ter sido o 0.
    // Estado 1 processa o ID do Slave.
#if DEBUG >= 1
        Serial.print(theDateIs());Serial.print("loop::SLAVEID = ");Serial.println(rx_buffer[rx_buffer_tail]);
#endif

        // Aqui podemos colocar tratamentos diferentes dependendo de qual Slave veio a mensagem.
        
        // prepara para processar o próximo byte da mensagem
        rx_buffer_tail++; 
        SSM_Status = 2;
        break; // case 1


    case 2:
    // Estado anterior deveria ter sido o 1.
    // Estado 2 processa o ALARMID.
#if DEBUG >= 1
        Serial.print(theDateIs());Serial.print("loop::ALARMID = ");Serial.println(rx_buffer[rx_buffer_tail]);
#endif

        // Aqui podemos colocar tratamentos diferentes dependendo do ALARMID.
        
        // prepara para processar o próximo byte da mensagem
        rx_buffer_tail++; 
        SSM_Status = 3;
        break; // case 2


    case 3:
    // Estado anterior deveria ter sido o 2.
    // Estado 3 processa o ALARMVALUE.
#if DEBUG >= 1
        Serial.print(theDateIs());Serial.print("loop::ALARMVALUE = ");Serial.println(rx_buffer[rx_buffer_tail]);
#endif

        // Aqui podemos colocar tratamentos diferentes dependendo do ALARMVALUE.
        
        // prepara para processar o próximo byte da mensagem
        rx_buffer_tail++; 
        SSM_Status = 4;
        break; // case 3


    case 4:
    // Estado anterior deveria ter sido o 3.
    // Estado 4 só descarta o CRC.

        // descarta o byte do CRC.
        rx_buffer_tail++; 
        SSM_Status = 0; // coloca a Máquina de Estados em seu estado inicial
        break; // case 4


    default:
    // Estado desconhecido.
    // Descarta o conteúdo de rx_buffer e coloca a Máquina de Estados em seu estado inicial.
#if DEBUG >= 1
        Serial.print(theDateIs());Serial.print("loop::Estado indefinido = ");Serial.println (SSM_Status);
        Serial.println("Colocando Máquina de Estados no estado inicial.");
#endif
        rx_buffer_tail = rx_buffer_head;
        SSM_Status = 0;
        break;
  } // end of switch(SSM_Status)



} // end of loop()
// ---------------------------------------------  fim do programa  -----------------------------------------------------
