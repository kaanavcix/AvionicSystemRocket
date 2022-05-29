/* 
Eva X roket takımı avyonik kurtarma ve paraşüt sistemi 
Model roketciliğin gelişmesi adına kullandığımız kodlar ve devre şemaları yapılan bir çok aşama paylaşılacaktır 

Saygılarımla bex,
 */


// Pin adresleri
#define buzzerPin 42
#define anaServo 9
#define dragServo 10


/*
Genel Kütüphane tanımlamaları
*/
 

////////////////////////////////////////////////////////////////////
#include <MPU6050.h> //Mpu6050 kütüphanesi
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include<SPI.h> 
#include <Math.h> 
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include "TinyGPS++.h"
#include <SimpleKalmanFilter.h>
//////////////////////////////////////////////////////////////////////////

/*
Nesne Tanımalamaları
*/

 
////////////////////////////////////////////////////////////////////////////////////////////
MPU6050 mpu;
TinyGPSPlus gps;  
SoftwareSerial loraSerial(19, 18);
LoRa_E32 loraModul(&loraSerial);
Adafruit_BMP280 bmp280Sensor; // bmp/bme slave adresi 0x76 dır default(varsayılan olarak bu değer gelmektedir)
SimpleKalmanFilter kalmanFiltresi(1, 1, 0.1); // kalman filtresi gps değerlerini daha sağlıklı almak için
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short int haberlesmeBand=9600;
unsigned short int gpsBand =9600;
unsigned short int loraBand =9600;
////////////////////////////////////////////////////////////////////////

int count=0;  
////////////////////////////////////////////////////////////////////7777
float XakisFiltreli;
float YakisFiltreli;


///////////////////////////////////////////////////////////////////////
int rocketTepede=1;
//////////////////////////////////////////////////////////////////////////////////
Vector filtresizGyro;
Vector filtreliGyro;
/////////////////////////////////////////////////////////
float dragIrtifa=550;
float anlikIrtifa=0;
float kalmanIrtifa=0;
float maksIrtifa = -2200;
float seaLevelPressureLevel= 1019.25;
///////////////////////////////////////////////////////////////////////////////////


typedef  struct {
  double lang;
  double late;
  double irtifa;
 //const char firstSuccess[9] ;
  //const char secondSuccess[15] ;
} Rockets;

Rockets rocketData;






void setup() {
  // Ardunioda mega pro minide bulunan bir çok serial portunu başlatmak adına kullanılır 
  //Serial 1 16-17 Serial 2 18-19 pinler içindir
  Serial.begin(haberlesmeBand);
  Serial.println("Varsayılan serial port başlatıldı");
   
 //buzzerPin çıkış pini olduğunu söyledik
  pinMode(buzzerPin, OUTPUT);  
  Serial.println("Buzzer Başlatıldı");
  
  //Lora başlatılma noktası
  Serial1.begin(loraBand);
  Serial1.println("Lora bağlantısı başlatılıyor...");
  loraModul.begin();
  if(loraModul.begin()){
Serial.println("Lora Modulü bağlantı başarılı");}

 
          
  //Gps başlatma noktası
  Serial2.begin(gpsBand);
  
    
   //Mpu başlama konumu                 
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
   mpu.calibrateGyro();
   mpu.setThreshold(3);
   checkSettings();
  
  //bmp 280 başlatma noktası
 if (!bmp280Sensor.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

   
bmp280Sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
     
//Servo Pinerinin Durumlarının Ayarlanması

pinMode(anaServo,OUTPUT);
pinMode(dragServo,OUTPUT);


//Servo Pinlerin Low yada High Olma Durumlarının ayarlanması

digitalWrite(anaServo,LOW);
digitalWrite(anaServo,LOW);




  


}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:        ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:      ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Gyroscope:         ");
  switch(mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  } 
  
  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());
  
  Serial.println();
}

void loop() { 
  while(Serial2.available()){
    
   filtresizGyro = mpu.readRawGyro();
  filtreliGyro = mpu.readNormalizeGyro();
    
   anlikIrtifa= bmp280Sensor.readAltitude(seaLevelPressureLevel); //Metre cinsinden
   kalmanIrtifa = kalmanFiltresi.updateEstimate(anlikIrtifa);// irtifayı noiselerini engeller
  
   Serial.println(bmp280Sensor.readAltitude(seaLevelPressureLevel));
   Serial.print(" x EKSENİ FİLTRESİZ = ");
  Serial.print(filtresizGyro.XAxis);
  Serial.print(" Y EKSENİ FİLTRESİZ = ");
  Serial.print(filtresizGyro.YAxis);
  Serial.print(" Z EKSENİ FİLTRESİZ = ");
  Serial.println(filtresizGyro.ZAxis);

  Serial.print(" X EKSENİ FİLTRELİ = ");
  Serial.print(filtreliGyro.XAxis);
  Serial.print(" Y EKSENİ FİLTRELİ = ");
  Serial.print(filtreliGyro.YAxis);
  Serial.print(" Z EKSENİ FİLTRELİ = ");
  Serial.println(filtreliGyro.ZAxis);

  XakisFiltreli=filtreliGyro.XAxis;
  YakisFiltreli=filtreliGyro.YAxis;
   
   
   

   if(kalmanIrtifa>maksIrtifa){maksIrtifa=kalmanIrtifa;}
   

   if(maksIrtifa-kalmanIrtifa>40&&rocketTepede==1&&0<YakisFiltreli&&25>YakisFiltreli&&90>XakisFiltreli&&75<XakisFiltreli){ 
    /* Eğer tepede ise irtifamız maxs irtifa ile anlık irtifa arasındaki fark 40 dan fazla ise ve 
    Açıdeğerlerimiz yatay olarak yada 75-25 açı değerini koruyabilirse ilk ayrılma işlemi gerçekleşmelidir.*/
    
   //İlk Ayrılma gerçekleşebilir

   digitalWrite(anaServo,HIGH);

   //rocketData.firstSuccess = "Başarı";
  

   rocketTepede=0;
    }

    if(dragIrtifa-kalmanIrtifa>10 &&rocketTepede==0){
      
      //ikinci Ayrılma gerçekleşebilir

      
digitalWrite(dragServo,HIGH);

   //rocketData.secondSuccess = "Başarı x2 <3";
  
      
      }
     

  
    if(gps.encode(Serial2.read())){

////////////////////////////////////////////////////////////////////////////////
    Serial.print("Lng : "); Serial.println(gps.location.lng(),6);
      rocketData.lang= gps.location.lng(),6;

        Serial.print("Lat : "); Serial.println(gps.location.lat(),6);
      rocketData.late= gps.location.lat(),6;

        Serial.print("İrtifa : "); Serial.println(gps.location.lat(),6);
      rocketData.irtifa= gps.altitude.meters(),6;

/////////////////////////////////////////////////////////////////////////////////////////



      

      

      
      
      }
    
    }

}





void aktifBuzzer()
{
  digitalWrite(buzzerPin, HIGH);
  delay(10);
  Serial.println("ses cikti");
  digitalWrite(buzzerPin, LOW);
  delay(10);
  Serial.println("ses kesildi");
}
void errorBuzzer(){
   digitalWrite(buzzerPin, HIGH);
  delay(200);
  Serial.println("ses cikti");
  digitalWrite(buzzerPin, LOW);
  delay(100);
  Serial.println("ses kesildi");}



static void smartDelay(unsigned long ms) 
{
  unsigned long start = millis();
  do
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
}
