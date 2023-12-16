/*
RoverC: https://github.com/m5stack/M5-RoverC/blob/master/examples/RoverC_M5StickC/RunningRoverC/RunningRoverC.ino
MQTT: https://gist.github.com/dzonesasaki/1c2a16650d7570f1aeb62eeeeec62b81
Joy_toy_cat: https://qiita.com/coppercele/items/1596b7b9904eb4403191
*/

#include <string.h>
#include <WiFi.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <M5StickC.h>
#include "M5_RoverC.h"

// RoverC
TFT_eSprite canvas = TFT_eSprite(&M5.Lcd);
M5_RoverC roverc;

// MQTT
/*
set
x to be rotate angle in range [-180,180] >> 4 char
y to be speed in range [-100,100], if !up.is_pressed() && !down.is_pressed() set y=0; >> 4 char
center to be toggle to keep x,y frezze in range [0,1] >> 1 char
*/

char data[11];
int x, y, center;
 
char *ssidC="Nit_2.4";
char *passC="0814038562";
char *servC="192.168.1.141";

const char* gcTopic = "joystick";
const String gsClientId = "M5Stick_RoverC";

WiFiClient myWifiClient;
const int myMqttPort = 1883;
PubSubClient myMqttClient(myWifiClient);

#define MQTT_QOS 0
volatile uint16_t gu16Posx;
volatile uint16_t gu16Posy;
volatile uint16_t gu16Count;

#define N_DOT_FONT_X 16
#define N_DOT_FONT_Y 24
#define MAX_DISP_WIDTH 160 // 240 // replace to 160 for M5StickC
#define MAX_DISP_HEIGHT 80 // 135 // replace to 80 for M5StickC

void initWifiClient(void){
  Serial.print("Connecting to ");
  Serial.println(ssidC);
  uint16_t tmpCount =0;

  WiFi.begin( ssidC, passC);
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
  tmpCount++;
  if(tmpCount>128)
    {
      //gFlagWiFiConnectFailure = true;
      Serial.println("failed  ");
      return;
    }

  }

  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectMqttSub()
{
  myMqttClient.setServer(servC, myMqttPort);
  reconnectMqttSub();
  myMqttClient.setCallback(gotMessageViaMqtt);
}

void reconnectMqttSub()
{
  while( ! myMqttClient.connected() )
  {
    Serial.println("Connecting to MQTT Broker...");
    // String clientId = "myMqttPub" ;
    if ( myMqttClient.connect(gsClientId.c_str()) )
    {
      myMqttClient.subscribe(gcTopic,MQTT_QOS);
      Serial.println("done! connected to Broker and subscribed. ");
    }
    delay(1000);
  }
} 

void gotMessageViaMqtt(char * pctoic, byte * pbpayload, uint32_t uiLen){
  Serial.print(pctoic);
  Serial.print(" : \t");

  char message[256];
  // Convert payload to a null-terminated string
  strncpy(message, (char*)pbpayload, uiLen);
  message[uiLen] = '\0';

  for (uint32_t i = 0; i < uiLen; i++) {
    Serial.print((char)pbpayload[i]);
  }
  strncpy(data, message,11);
  // Serial.println(temp);
  // data[11]=temp;
  // Serial.println(data);
  
  // M5.Lcd.setCursor(gu16Posx, gu16Posy);
  // M5.Lcd.printf(data);
  /*
  gu16Posx +=N_DOT_FONT_X;

  if ((gu16Posx+N_DOT_FONT_X)>(MAX_DISP_WIDTH-1)){
    gu16Posx =0;
    gu16Posy +=N_DOT_FONT_Y;
  }

  if ((gu16Posy+N_DOT_FONT_Y)>(MAX_DISP_HEIGHT-1)){
    gu16Posy =5;
    M5.Lcd.fillScreen(BLACK);
  }
  */
  Serial.println();

}

int data_slice(char* input_data, int start, int stop)
{
    char s[stop - start + 2]; // +2 for null-terminator and extra space
    int j = 0;

    for (int i = start; i <= stop; i++)
    {
        s[j++] = input_data[i];
    }
    s[j] = '\0'; // Null-terminate the string
    int int_s = atoi(s);
    return int_s;
}

void setup() {
  M5.Lcd.begin();
  //RoverC
  M5.begin();
  roverc.begin();
  M5.Lcd.setRotation(1);
  canvas.createSprite(160, 80);
  canvas.setTextColor(ORANGE);
  roverc.setSpeed(0, 0, 0);
  canvas.setTextDatum(MC_DATUM);
  canvas.drawString("RoverC Test", 80, 40, 4);
  canvas.pushSprite(0, 0);
  delay(1000);

  // MQTT
  Serial.begin(115200);
  initWifiClient();
  M5.Lcd.setRotation( 1 );
  M5.Lcd.setTextDatum(0);
  M5.Lcd.fillScreen(BLACK);// WHITE RED GREEN BLUE  BLACK
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);

  gu16Posx =0;
  gu16Posy =5;
  gu16Count =0;
  
  connectMqttSub();
  Serial.println("Done setup");
  M5.Lcd.println("Done setup");

}

void loop(){
  reconnectMqttSub();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0,0);

  x=data_slice(data,0,3);
  y=data_slice(data,5,9);
  center=data_slice(data,10,11);
  
  // report via USB
  Serial.print("X: "); Serial.print(x);
  Serial.print(", Y: "); Serial.print(y);
  Serial.print(", Center: "); Serial.println(center);

  // report via screen
  M5.Lcd.printf("X:%d\nY:%d\nC:%d", x, y, center);

  // roverc.setSpeed(int8_t x, int8_t y, int8_t z)
  roverc.setSpeed(x, y, center*100);

  // M5.update();
  myMqttClient.loop();
  delay(500);
}