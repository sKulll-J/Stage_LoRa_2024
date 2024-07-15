#include<SoftwareSerial.h>
#include<DHT.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include "rgb_lcd.h"
#include "Adafruit_SHT31.h"
#include <time.h>

//#include"lora_e5.h"

//serial
#define RX 5
#define TX 6
SoftwareSerial LoRaSerial(RX,TX);


//TTN Infos
#define DevEui "70B3D57ED0067B7A"
#define DevAddr "260BEC30"
#define AppEui "0000000000000000"
#define NwksKey "E072735F512B40B4853D8CB694A68FDF"
#define AppsKey "1CB83DF99A750DA8DB9459891332E79D"
#define Mode "ABP" //"OTAA"
#define DR "7"
#define PORT "8"
#define CH "1" //changer pour chaques ruches

//DHT22 part
#define brocheBranchementDHT 4   // Si la ligne de données du DHT22 est branchée sur la pin D6 de votre Arduino, par exemple
#define typeDeDHT DHT22          // Si votre DHT utilisé est un DHT22 (mais cela pourrait aussi bien être un DHT11 ou DHT21, par exemple)

//SWITCH SENDER - switch qui permet de savoir si on envoie ou pas des trames


DHT dht(brocheBranchementDHT, typeDeDHT);

float t1 = 0;
int h1 = 0;
char command[50];
char t1Str[10];
char h1Str[10];

bool as_joined=false;

bool exist = false;
unsigned long int st_temp;

//AT command vars
static char recv_buf[512];


// fonctions definitions
int init_lora_e5();
static int at_send_check_response(char *p_ack, int timeout_ms, char*p_cmd, ...);
int at_send(char* str1, char* str2, char* verif, int t_out);

rgb_lcd lcd;

const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

bool enableHeater = false;
uint8_t loopCnt = 0;

Adafruit_SHT31 sht31 = Adafruit_SHT31();

//f setup
void setup() {
    Serial.begin(9600); // Initialize USB Serial
  
    LoRaSerial.begin(9600); // Initialize Software Serial
    delay(2000);
    Serial.println("Setup");
    dht.begin();
    
    while (!Serial)
        delay(10);     // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("SHT31 test");
    if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
        Serial.println("Couldn't find SHT31");
        while (1) delay(1);
    }

    

	Serial.println("\n_____\nBegin Init :");
    
    if(init_lora_e5())
    {
        exist=true;
        Serial.println("InitDone");
    }    
    else
        Serial.println("Init Error");
}

// main loop
void loop() {
  // Check if data is available on USB Serial
    while(Serial.available())
    {
        int datas = Serial.read();
        LoRaSerial.write(datas);
        st_temp = millis();
    }
    //LoRaSerial.print("AT+ID=DevEui");
    if(LoRaSerial.available())
    {
        delay(100); //permet d'attendre ue toutes les donées soient dans le buffer de récéption
        int cnt=0;
        while(LoRaSerial.available())
        {
            int ch = LoRaSerial.read();
            recv_buf[cnt] = (char) ch ;
            cnt++;
        }   
        Serial.write(recv_buf);
        if(recv_buf=="+JOIN: Network joinded")
        {
            Serial.println("Messages can be sent");
            as_joined = true;
        }
        Serial.print("dt :");
        Serial.println(millis()-st_temp-100); //remove le delai imposée
    }
    if(as_joined)  //if(exist)
    {
        delay(2100);
        Serial.println("________");
        Serial.print("ruche ");
        Serial.println(CH);
        t1 = dht.readTemperature();
        Serial.print("ti : ");
        Serial.println(t1);
        delay(2500);
        h1 = dht.readHumidity();
        Serial.print("hi : ");
        Serial.println(h1);
        delay(2500);

        //Serial.print("Ti : ");
        dtostrf(t1, 3, 1, t1Str); //transforme la valeur en string
        dtostrf(h1, 3, 0, h1Str); //transforme la valeur en string

        sprintf(command, "\"T1:%s, H1:%d\"", t1Str, h1);
        at_send("AT+MSG=, ", command, "+MSG: Done", 5000);
        

        delay(10000);
    }

}

//fonctions 
int at_send(char* str1, char* str2, char* verif, int t_out)
{
    char buff[50];
    char c[] = "\r\n";
    sprintf(buff, "%s%s%s", str1,str2,c);
    //Serial.println(buff);
    return at_send_check_response(verif, t_out, buff);
}

int init_lora_e5()
{
    int valid=0;
    if(at_send_check_response("+AT: OK", 20000, "AT\r\n"))
    {
        valid+=at_send("AT+ID=DevEui, ", DevEui, "+ID: DevEui", 2000);
        valid+=at_send("AT+ID=DevAddr, ", DevAddr, "+ID: DevAddr", 2000);
        valid+=at_send("AT+KEY=NWKSKEY, ", NwksKey, "+KEY: NWKSKEY", 2000);
        valid+=at_send("AT+KEY=APPSKEY, ", AppsKey, "+KEY: APPSKEY", 2000);
        valid+=at_send("AT+MODE=, ", Mode, "+MODE: ", 2000);
        valid+=at_send("AT+DR=, ", DR, "+DR: ", 2000);
        valid+=at_send("AT+CH=, ", CH, "+CH: ", 2000);

        valid+=at_send("AT+MSG", "", "+MSG: Done", 4000);

        if(valid==7) //depend du nombre d'AT command envoyée
            return 1;
        else
            return 0;
    }  
    else
    {
        Serial.println("AT : error");        
        return 0;

    }
}


static int at_send_check_response(char *p_ack, int timeout_ms, char*p_cmd, ...)
{
    int ch;
    int num = 0;
    int index = 0;
    unsigned long  startMillis = 0;
    va_list args;
    char cmd_buffer[256];  // Adjust the buffer size as needed
    memset(recv_buf, 0, sizeof(recv_buf));
    va_start(args, p_cmd);
    vsprintf(cmd_buffer, p_cmd, args);  // Format the command string
    LoRaSerial.print(cmd_buffer);
    Serial.print(cmd_buffer);
    va_end(args);
    delay(50);
    startMillis = millis();


    if (p_ack == NULL)
    {
        Serial.println("p_ack none");
        return 0;
    }

    do
    {
        while (LoRaSerial.available() )
        {
            ch = LoRaSerial.read();
            recv_buf[index++] = ch;
            Serial.print((char)ch);
            delay(2);
        }
        if((strstr(recv_buf, "ERROR")||strstr(recv_buf, "error")) != NULL)
        {
            Serial.println("< ERROR IN AT COMMAND >");
            return 0;
        }
        if (strstr(recv_buf, p_ack) != NULL)
        {
            return 1;
        }
    //Serial.println(millis() - startMillis);
    } while (millis() - startMillis < timeout_ms);
    
    Serial.println("< ERROR TIME-OUT >");
    return 0;
}