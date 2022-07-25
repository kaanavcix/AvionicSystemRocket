#include "LoRa_E32.h"
#include <SoftwareSerial.h>
SoftwareSerial loraSerial(11, 10);
LoRa_E32 loraModul(&loraSerial);


void setup()
{
  Serial.begin(9600);
  delay(100);
  loraModul.begin();
}

typedef struct {
byte altitude[7];
byte latitude[10];
byte longitude[10];
byte irtifa[7];
byte aci[5];
char firstSucess[2];
char secondSuccess[2];
} Rocket;
Rocket data;

void loop()
{
  if (loraModul.available()  > 1){
    ResponseStructContainer rsc = loraModul.receiveMessage(sizeof(Rocket));
    data = *(Rocket*) rsc.data;
    Serial.print(*(float*)data.irtifa,1);
    Serial.print(*(float*)data.altitude,1);
    Serial.print(*(float*)data.latitude,6);
    Serial.print(*(float*)data.longitude,6);
    Serial.print(*(float*)data.aci,1);
if(!data.firstSucess==""){
    Serial.print(data.firstSucess);
    Serial.print(data.secondSuccess);
}
    Serial.println("/");
    rsc.close();
  }
}
