#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Time.h>
#include <PubSubClient.h>

// temperature sensor port
#define TEMP_SENS   1
#define TEMP_COEF   10.0  // conversion parameter (mV per ℃)

// light intensity sensor port
#define LIGHT_SENS  0
#define LIGHT_COEF  0.35  // conversion parameter (mV per Lux)

// toggle SW port
#define SW_TOGGLE   16

// DIP SW ports
#define SW_DIP_1    4
#define SW_DIP_2    7
#define SW_DIP_3    8
#define SW_DIP_4    9

// RGB LED ports
#define LED_POLARITY  1	// 0: cathode common, 1: anode common
#define LED_R   3	// Red
#define LED_G   5	// Green
#define LED_B   6	// Blue

#define MAX_TOPIC_LEN 64

// Parameters
const unsigned int MEASURE_INTERVAL = 1000;  // Upload Interval
const byte LED_ON = 25;                      // brightness of LED (1-255)

// Sensed data field
float temperature;
char str_temperature[7];
float illuminance;
char str_illuminance[7];
int dipsw;
char str_dipsw[5];
int tglsw;
char str_tglsw[5];
char str_epoch[64];

PROGMEM prog_char pgstr_measure_data_format1[] = "%d.%1d\0";
PROGMEM prog_char pgstr_measure_data_format2[] = "%d\0";
PROGMEM prog_char pgstr_measure_data_ON[] = "ON\0";
PROGMEM prog_char pgstr_measure_data_OFF[] = "OFF\0";

//MQTT configuration
char mqtt_topic[MAX_TOPIC_LEN] = "dalchymia.net/device/0";
byte server[] = { 133, 242, 163, 82 };

// MAC address
byte mac[] = {  
  0x00, 0x50, 0xc2, 0x97, 0x22, 0xc3 };

// UDP local port number
unsigned int localPort = 8888;

// NTP time server address (ntp.nict.jp NTP server)
IPAddress timeServer(133, 243, 238, 164);

// NTP packet buffer size
const int NTP_PACKET_SIZE= 48;

// NTP packet buffer for send
byte packetBuffer[NTP_PACKET_SIZE];

// Udp class
EthernetUDP Udp;

// Lastest time of packet sent (ms)
unsigned long lastSendPacketTime = 0;

// callback function for MQTT
char message_buff[100];
void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  int i = 0;
  
  // create character buffer with ending null terminator (string)
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  
  String msgString = String(message_buff); 
}

EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);


void setup() 
{
  led_rgb_set(0, 0, 0);  // LED off
  Serial.begin(9600);
  
  Serial.println("Attempting to obtain a DHCP lease...");

  if ( Ethernet.begin(mac) == 0 ) {
    Serial.println("Failed to configure Ethernet using DHCP");
    for(;;)
      ;
  }

  Serial.println("A DHCP lease has been obtained.");

  Serial.print("My IP address is ");
  Serial.println(Ethernet.localIP());

  Serial.print("Gateway IP address is ");
  Serial.println(Ethernet.gatewayIP());

  Serial.print("DNS IP address is ");
  Serial.println(Ethernet.dnsServerIP());
  Serial.println();

  Udp.begin(localPort);

  // Send NTP packet
  sendNTPpacket(timeServer);
  lastSendPacketTime = millis();
  
  client.connect("arduinoClient");  
  
}


void loop()
{
  
  char tmpbuf[64];
  char pub_message_buf[256];
  
  if ( millis() - lastSendPacketTime > MEASURE_INTERVAL ){
    
    led_rgb_set(0, 0, LED_ON);  // LED blue
    
    // Send a request to NTP Server
    sendNTPpacket(timeServer);
    // Update time
    lastSendPacketTime = millis();
  }

  // Receive a packet from NTP Server
  if ( Udp.parsePacket() ) {
    // read buffer and
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    // Time is included from 40 bytes and its size is 4 bytes
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);

    // NTPタイムスタンプは64ビットの符号無し固定小数点数（整数部32ビット、小数部32ビット）
    // 1900年1月1日0時との相対的な差を秒単位で表している
    // 小数部は切り捨てて、秒を求めている
    //
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = " );
    Serial.println(secsSince1900);

    // Convert NTP timestamp to Unix timestamp
    // Unix time starts at Jan 1st, 1970
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;  
    Serial.print("Unix time = ");
    Serial.println(epoch);
    sprintf(str_epoch, "%lu", epoch);
    Serial.println(str_epoch);

    // Set time to JST (+0900)
    setTime(epoch + (9 * 60 * 60));

    Serial.print("JST is ");
    Serial.print(year());
    Serial.print('/');
    Serial.print(month());
    Serial.print('/');
    Serial.print(day());
    Serial.print(' ');
    Serial.print(hour());
    Serial.print(':'); 
    Serial.print(minute());
    Serial.print(':'); 
    Serial.println(second());
    Serial.println();
    
    // Sensing: Temperature
    temperature = (float)analogRead(TEMP_SENS) * (5.0 / 1024.0) * 1000.0 / TEMP_COEF;
    strcpy_P(tmpbuf,pgstr_measure_data_format1);
    sprintf(str_temperature, tmpbuf, (int)temperature, (int)(temperature * 10) % 10);
    Serial.print("Temperature=");
    Serial.println(str_temperature);

    // Sensing: Illuminance
    illuminance = (float)analogRead(LIGHT_SENS) * (5.0 / 1024.0) * 1000.0 / LIGHT_COEF;
    strcpy_P(tmpbuf,pgstr_measure_data_format2);
    sprintf(str_illuminance, tmpbuf, (int)illuminance);
    Serial.print("Illuminance=");
    Serial.println(str_illuminance);

    // Seinsing: DIP SW
    dipsw = (digitalRead(SW_DIP_4) == HIGH) * 8 + (digitalRead(SW_DIP_3) == HIGH) * 4
          + (digitalRead(SW_DIP_2) == HIGH) * 2 + (digitalRead(SW_DIP_1) == HIGH);
    sprintf(str_dipsw, tmpbuf, dipsw);
    Serial.print("DIP SW=");
    Serial.println(str_dipsw);
  
    // Sensing: Toggle SW
    tglsw = (digitalRead(SW_TOGGLE) == HIGH);
    if (tglsw) {
      strcpy_P(str_tglsw,pgstr_measure_data_ON);
    } else {
      strcpy_P(str_tglsw,pgstr_measure_data_OFF);
    }
    Serial.print("TOGGLE SW=");
    Serial.println(str_tglsw);
    
    // Compose body message
    strcpy(pub_message_buf,str_epoch);
    strcat(pub_message_buf,",");
    strcat(pub_message_buf,str_temperature);
    strcat(pub_message_buf,",");
    strcat(pub_message_buf,str_illuminance);
    strcat(pub_message_buf,",");
    strcat(pub_message_buf,str_dipsw);
    strcat(pub_message_buf,",");
    strcat(pub_message_buf,str_tglsw);
    
    // Publish message via MQTT
    if (!client.connected()){
      client.connect("arduinoClient");  
    }else{
      client.publish(mqtt_topic, pub_message_buf);
      Serial.print("Published message:");
      Serial.println(pub_message_buf);
    }
    
    led_rgb_set(0, 0, 0);  // LED off

  }
}

// send an NTP request to the time server at the given address 
unsigned long sendNTPpacket(IPAddress& address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE); 
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49; 
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  // NTP requests are to port 123
  Udp.beginPacket(address, 123);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

// control RGB LED
void led_rgb_set(byte r, byte g, byte b)
{
  analogWrite(LED_R, LED_POLARITY ? (255 - (r)) : (r));
  analogWrite(LED_G, LED_POLARITY ? (255 - (g)) : (g));
  analogWrite(LED_B, LED_POLARITY ? (255 - (b)) : (b));
}
