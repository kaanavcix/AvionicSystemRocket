
//Regards bex, everytime for my everything 
 
// Pin adresleri
#define buzzerPin 42
#define anaRole 9
#define dragRole 10

#include "Arduino.h"
#include "LoRa_E32.h"
#include "TinyGPS++.h"
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <SimpleKalmanFilter.h>
#include <KalmanFilter.h> // new other kalman filter add for more correct result

//LORA TİMER KULLANIMI

unsigned long kanalBekleme_zaman=0;
int loraKanal_bekleme = 5000;
unsigned long loraKanal_bekleme2 = 2000;
int kanalBekleme_zaman2 =0;

//LORA TİMER KULLANIMI


TinyGPSPlus gps;
SoftwareSerial portLora(19, 18);
LoRa_E32 loraModel (&portLora);
Adafruit_BMP280 bmp280Sensor; 
SimpleKalmanFilter xKalman(0.001, 0.003, 0.03); // nette buldum bi bakalım sonuçlara
SimpleKalmanFilter  yKalman(0.001, 0.003, 0.03); // nette buldum bi bakalım sonuçlara
SimpleKalmanFilter basincKalman(1, 1, 0.01); 

float oankiDenizSeviyeBasinci=0; 
float anlikIrtifa=0;
float kalmanIrtifa=0;
// iVME VE GYRO İÇİN

float old_x = 0;                          
float old_y = 0;                          
float prev_angle_x = 0;                  
float prev_angle_y = 0;                   

unsigned long previousMillis = 0;        
int interval = 30;                        
int X_offset = 520;                     
int Y_offset = 0;                         
int Z_offset = 0; 
float gyroX =0;
float gyroY  = 0;
float XakisFiltreli ;
float YakisFiltreli;


// İVME GYRO  İÇİN


int rocketTepede=1;
int *ptr = &rocketTepede;

float dragIrtifa=570;
float maksIrtifa = -2800;

bool donme_okey_mi = false;
typedef struct {
byte altitude[7];
byte latitude[10];
byte longtitude[10];
byte irtifa[7];
byte aci[5];
char firstSucess[2];
char secondSuccess[2];
// 43 byte 
} Rocket;
Rocket data;

void setup() {
  bmp280Sensor.begin(0x77,0x60);

bmp280Sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */



  oankiDenizSeviyeBasinci=bmp280Sensor.readPressure()/100;  

    Serial.begin(9600);
    delay(500);

    Serial2.begin(9600);
    delay(500);

    loraModel.begin();
}
void donme_tam_mi(){

  if((180>XakisFiltreli&&135<XakisFiltreli)||(0>XakisFiltreli&&45<XakisFiltreli)){
    donme_okey_mi=true;
    
    }
    else{
      donme_okey_mi=false;
      }
}
void loop() {

    while(Serial2.available())
    {
      
        if(gps.encode(Serial2.read()))
        
        {
            mpu6050();
        XakisFiltreli = xKalman.updateEstimate(gyroX);// X ekseni kalman filtresi
    YakisFiltreli = yKalman.updateEstimate(gyroY);// X ekseni kalman filtresi

           anlikIrtifa= bmp280Sensor.readAltitude(oankiDenizSeviyeBasinci); //Metre cinsinden
   kalmanIrtifa = basincKalman.updateEstimate(anlikIrtifa);// irtifayı noiselerini engeller

   if(kalmanIrtifa>maksIrtifa){

   maksIrtifa=kalmanIrtifa;}
  

  if((maksIrtifa-kalmanIrtifa>20&&rocketTepede==1)||(maksIrtifa-kalmanIrtifa>20&&rocketTepede==1&&donme_okey_mi==true)){ 

    
       

digitalWrite(anaRole,HIGH);
delay(200);
digitalWrite(anaRole,LOW);

data.firstSucess = "F";


   *ptr=0;
   }

 if(dragIrtifa-kalmanIrtifa>10 &&rocketTepede==0){
          Serial.println("İkinci Ayrılma Başarılı");

          digitalWrite(dragRole,HIGH);
          delay(200);
          digitalWrite(dragRole,LOW);
data.firstSucess = "S";


}
           // String msg = Serial2.readStringUntil('\r');
            //Serial.println(msg);

            Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
            *(float*)(data.latitude) = (gps.location.lat());
            Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
            *(float*)(data.longtitude) = (gps.location.lng());

            Serial.print("ALT="); Serial.println(gps.altitude.meters(), 6);
            *(float*)(data.altitude) = (gps.altitude.meters());
              Serial.print("ALT="); Serial.println(kalmanIrtifa);
            *(float*)(data.irtifa) = kalmanIrtifa;
  Serial.print("aci="); Serial.println(YakisFiltreli);
            *(float*)(data.aci) = YakisFiltreli;

if(millis()>kanalBekleme_zaman2+loraKanal_bekleme2)
kanalBekleme_zaman2=millis();


            ResponseStatus rs = loraModel.sendFixedMessage(0, 5, 17, &data, sizeof(Rocket));
Serial.println(rs.getResponseDescription());
        }




    }
}


void mpu6050(){
  unsigned long currentMillis = millis();                             //Millis değeri alınır

 
  if (currentMillis - previousMillis >= interval) {                   //İnterval süresi geçtiğinde içeri girilir
    previousMillis = currentMillis;

    Wire.beginTransmission(0x68);                                 //MPU6050 ile I2C haberleşme başlatılır
    Wire.write(0x43);                                                 //Gyro bilgisinin olduğu 0x43-0x48 için request gönderilir
    Wire.endTransmission(false); 
    Wire.requestFrom(0x68, 6, true);
    int16_t XGyroFull = (Wire.read() << 8 | Wire.read()) + X_offset;   //8-bitlik okunan iki değerden büyük ağırlıklı olanı 8 bit sola kaydırılıp küçük olanla veyalanır. Offset eklenir.
    int16_t YGyroFull = (Wire.read() << 8 | Wire.read()) + Y_offset;   //8-bitlik okunan iki değerden büyük ağırlıklı olanı 8 bit sola kaydırılıp küçük olanla veyalanır. Offset eklenir.
    int16_t ZGyroFull = (Wire.read() << 8 | Wire.read()) + Z_offset;   //8-bitlik okunan iki değerden büyük ağırlıklı olanı 8 bit sola kaydırılıp küçük olanla veyalanır. Offset eklenir.
    float XGyroFinal = (float)XGyroFull / 131;                         //Datasheet'te yazan değerlere göre "deg/s" cinsinden açısal hız bulunur. (X ekseni için)
    float YGyroFinal = (float)YGyroFull / 131;                         //Datasheet'te yazan değerlere göre "deg/s" cinsinden açısal hız bulunur. (Y ekseni için)
    float ZGyroFinal = (float)ZGyroFull / 131;                         //Datasheet'te yazan değerlere göre "deg/s" cinsinden açısal hız bulunur. (Z ekseni için)

    //X ekseni için açı hesabı
    float delta_angle_x = (0.03 * old_x) + ((0.03 * (XGyroFinal - old_x)) / 2); //açısal hız ve geçen süreye bağlı olarak taranan açı hesaplanır
    float angle_x = (prev_angle_x + delta_angle_x);                             //taranan açı değeri ile bir önceki açı değeri hesaplanarak 
    prev_angle_x = angle_x;                                                     //güncel açı değeri bir sonraki döngüde kullanılmak üzere önceki açı değeri olarak kaydedilir
    old_x = XGyroFinal;                                                         //güncel açısal hız değeri bir sonraki döngüde kullanılmak üzere önceki açısal hız değeri olarak kaydedilir

    //Y ekseni için açı hesabı
    float delta_angle_y = (0.03 * old_y) + ((0.03 * (YGyroFinal - old_y)) / 2); //yukarıdaki işlemlerin aynısı Y ekseni için de yapılır
    float angle_y = (prev_angle_y + delta_angle_y);
    prev_angle_y = angle_y;
    old_y = YGyroFinal;


    //Açısal hız değerleri seri porttan basılır
//    Serial.print("X Axis = ");  
//    Serial.print(XGyroFinal);
//    Serial.print("deg/s");
//    Serial.print("\t");
//    Serial.print("Y Axis = ");
//    Serial.print(YGyroFinal);
//    Serial.print("deg/s");
//    Serial.print("\t");

    gyroX = XGyroFinal;
    gyroY =YGyroFinal;
    }
}
