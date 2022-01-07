//Libraries
#include <Wire.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

 
#define REPORTING_PERIOD_MS     1000
 
PulseOximeter pox;
uint32_t tsLastReport = 0;

void onBeatDetected()
{
    Serial.println("Beat!");
}
  
const int redLed=5;
const int greenLed=6;
const int buzzer =9;
const int relay= 8;
#if 0
  #include <SPI.h>
  #include <PN532_SPI.h>
  #include "PN532.h"

  PN532_SPI pn532spi(SPI, 10);
  PN532 nfc(pn532spi);
#elif 1
  #include <PN532_HSU.h>
  #include <PN532.h>
      
  PN532_HSU pn532hsu(Serial);
  PN532 nfc(pn532hsu);
#else 
  #include <Wire.h>
  #include <PN532_I2C.h>
  #include <PN532.h>
  PN532_I2C pn532i2c(Wire);
  PN532 nfc(pn532i2c);  
#endif
void setup(void) {
  Serial.print("Initializing pulse oximeter..");
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }
     pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
 
    // Register a callback for the beat detection
    pox.setOnBeatDetectedCallback(onBeatDetected);
pinMode(redLed,OUTPUT);
pinMode(greenLed,OUTPUT);
pinMode(relay,OUTPUT);
pinMode(buzzer,OUTPUT);

Serial.begin(115200);
Serial.println("Hello!");

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  
  #include <PN532.h>
  #include <NfcAdapter.h>
  
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);


nfc.setPassiveActivationRetries(0xFF);
nfc.SAMConfig();    
Serial.println("Waiting for an ISO14443A card");
}

void loop(void) {
boolean success;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  
uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength); 
  if (success) {
    digitalWrite(A1,255);
    Serial.println("Found a card!");
    Serial.print("UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("UID Value: ");
    for (uint8_t i=0; i < uidLength; i++) 
    {
      Serial.print(" 0x");Serial.print(uid[i], HEX); 
    }
    Serial.println("");
    // Wait 1 second before continuing
    delay(1000);
      digitalWrite(relay, HIGH); 
 pulse();
delay(3000);
 if(pox.getHeartRate()>100||pox.getHeartRate()<70|| pox.getSpO2()<85)
 {
tone(9, 5000, 200);
digitalWrite(greenLed,LOW);
digitalWrite(redLed,HIGH);

}
else{
digitalWrite(greenLed,HIGH);
digitalWrite(redLed,LOW);
tone(9, 1000, 200);

}
}
  else
{Serial.println("Timed out waiting for a card");
digitalWrite(relay, LOW); //drive relay
digitalWrite(redLed,LOW);
digitalWrite(greenLed,LOW);

  }

}
void pulse()
{
    pox.update();
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
 Serial.print("Heart rate:");
Serial.print(pox.getHeartRate());
Serial.print("bpm / SpO2:");
 Serial.print(pox.getSpO2());
Serial.println("%"); 
tsLastReport = millis();
    }}
