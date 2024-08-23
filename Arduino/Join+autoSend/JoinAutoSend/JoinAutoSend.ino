#include<SoftwareSerial.h>
#include<String.h>

//serial
#define RX 5
#define TX 6
SoftwareSerial LoRaSerial(RX,TX);


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600); // Initialize USB Serial
    LoRaSerial.begin(9600); // Initialize Software Serial

    LoRaSerial.write("AT+JOIN\r\n");
}

void loop() {
    while(Serial.available())
    {
        int datas = Serial.read();
        //Serial.print(">");
        Serial.write(datas);
        LoRaSerial.write(datas);
    }
    //LoRaSerial.print("AT+ID=DevEui");
    while(LoRaSerial.available())
        Serial.write(LoRaSerial.read());

    delay(15000);
    LoRaSerial.write("AT+MSG=\"Bonjour\"\r\n");

}
