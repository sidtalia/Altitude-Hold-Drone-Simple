#include<Wire.h>
#include "MPU6050.h"

MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int i;
int16_t a[6];
float A[6];
#define LED_PIN 13
bool blinkState = false;

void setup() {
    Wire.setClock(800000);
    Wire.begin();

    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    pinMode(LED_PIN, OUTPUT);
}

void loop() 
{
    for(i = 0;i<10000;i++)
    {
      accelgyro.getMotion6(&a[0], &a[1], &a[2], &a[3], &a[4], &a[5]);
      for(int j = 0;j<6;j++)
      {
        A[j] += a[j];
      }
    }
    for(i =0;i<6;i++)
    {
      A[i]/=10000;
      Serial.print(A[i]);Serial.print("||");
      A[i]=0;
    }
    Serial.println();
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(10);
}
