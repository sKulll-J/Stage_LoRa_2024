#include<SoftwareSerial.h>
#include<DHT.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include "rgb_lcd.h"
#include "Adafruit_SHT31.h"
#include "Ultrasonic.h"
#include "HX711.h"
#include <LowPower.h>

#include "chrono.h"
//#include"lora_e5.h"


//serial
#define RX 5
#define TX 6
SoftwareSerial LoRaSerial(RX,TX);

//poids
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
float Calibration_Weight = 213.60;
HX711 scale;

#define LORAINFO false //définie si on doit modifier les infos Lora lors d'un nouveau flash
//TTN Infos
#define DevEui "70b3d57ed0067d1d"
#define DevAddr "26 0b b1 1c"
#define AppEui "0000000000000000"
#define NwksKey "1af79eb4c416f0dbdc835eda04297741"
#define AppsKey "3f4cb38212b261d1a831fc4ec6fa3995"
#define Mode "ABP" //"OTAA"
#define DR "7"
#define PORT "2"
#define CH "2" //changer pour chaques ruches

//DHT22 part
#define brocheBranchementDHT 4   // Si la ligne de données du DHT22 est branchée sur la pin D6 de votre Arduino, par exemple
#define typeDeDHT DHT22          // Si votre DHT utilisé est un DHT22 (mais cela pourrait aussi bien être un DHT11 ou DHT21, par exemple)

#define pinultrasonic 7

//datas struct
struct datas{
    int Hi; 
    float Ti;
    int Ho;
    float To;
    int presence;
    int poids;
    int dataRstCnt; //permet de compter le nombre de data qui ont été mises a jour
    bool allDatasReset ; //identifie si toutes les datas ont été mises a jour
};
datas myDatas={0,0,0,0,1,0,0,false};

//chrono struct - pour déompter le temps


//chrono ch_sht31;
//chrono ch_dht22;
//chrono ch_sender;
chrono ch_lcd;
chrono ch_lcdstop;
chrono ch_send;


//SWITCH SENDER - switch qui permet de savoir si on envoie ou pas des trames

//composants declarations
DHT dht(brocheBranchementDHT, typeDeDHT);
rgb_lcd lcd;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Ultrasonic ultrasonic(pinultrasonic);


//global var
float t1 = 0;
int h1 = 0;


int people = false;
bool newPresence = false; // permet d'envoyer un  message lorsqu'une nouvelle présence est détéctée, une fois le message envoyé la variable repasse a false
bool dht22_TH = false;

bool exist = false;

int lcd_inf = 0;

//AT command vars
static char recv_buf[55];

char command[50];
char t1Str[10];
char h1Str[10];

// fonctions definitions
int init_lora_e5(); //initialise le module LoRa avec toutes les commandes AT
static int at_send_check_response(char *p_ack, int timeout_ms, char*p_cmd, ...);
int at_send(char* str1, char* str2, char* verif, int t_out);



bool enableHeater = false;
uint8_t loopCnt = 0;


//f setup
void setup() {
    Serial.begin(9600); // Initialize USB Serial
  
    LoRaSerial.begin(9600); // Initialize Software Serial
    delay(2000);
    Serial.println("Setup");
    dht.begin();
    lcd.begin(16,2);

    //begin poids
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_scale(Calibration_Weight);
    scale.tare();
    
    lcd.setCursor(0, 0);
    lcd.print("Initialising...");
    lcd.setCursor(0, 1);
    lcd.print("Ruche ");
    lcd.print(CH);
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


    //chrono init
    //ch_dht22.startChrono(4000);
    //ch_sht31.startChrono(4000);
    ch_lcd.startChrono(2500); // temps pour affihcer une infos et passer a la suivante
    ch_lcdstop.startChrono(30000); // temps aprés lequel le lcd s'éteint
    ch_send.startChrono(15000);

    lcd.clear();
 
}

// main loop
void loop() {
  // Check if data is available on USB Serial
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

    //ultrasonic mesurement
    long RangeInCentimeters;
	RangeInCentimeters = ultrasonic.MeasureInCentimeters();
	delay(150);


    //lcd prints
    if(ch_lcd.isFinished())
    {   
        lcd.clear();
        // ligne 1 : captor type
        lcd.setCursor(0, 0); 
        switch(lcd_inf)
        {
            case 0:
                lcd.print("T out :");
                break;
            case 1:
                lcd.print("H out :");
                break;
            case 2:
                lcd.print("T in :");
                break;
            case 3:
                lcd.print("H in :");
                break;
            case 4:
                lcd.print("Poids :");
                break;
            default:
                lcd.print("<NoData>");
        }
        lcd.setCursor(14, 0);
        lcd.print("R");
        lcd.print(CH);
        //ligne 2 : captor value
        lcd.setCursor(0, 1);

        switch(lcd_inf)
        {
            case 0:
                myDatas.To = sht31.readTemperature();
                Serial.println(myDatas.To);
                lcd.print(myDatas.To);
                lcd.print(" C");
                break;
            case 1:
                myDatas.Ho = sht31.readHumidity();
                Serial.println(myDatas.Ho);
                lcd.print(myDatas.Ho);
                lcd.print(" %");
                break;
            case 2:
                myDatas.Ti = dht.readTemperature();
                Serial.println(myDatas.Ti);
                lcd.print(myDatas.Ti);
                lcd.print(" C");
                break;
            case 3:
                myDatas.Hi = dht.readHumidity();
                Serial.println(myDatas.Hi);
                lcd.print(myDatas.Hi);
                lcd.print(" %");
                break;
            case 4:
                if (scale.is_ready()) {
                    Serial.print("HX711 reading: ");
                    //int output = (int)(reading);
                    myDatas.poids== (int)scale.get_units(10);;
                    //Serial.print(myDatas.poids);
                    //Serial.println(" g");
                } else {
                    Serial.println("HX711 not found.");
                }

                lcd.print(myDatas.poids);
                lcd.print(" g");
                Serial.print(myDatas.poids);
                Serial.println(" g");
                break;
            default:
                lcd.print("<NoData>");
        }
        Serial.print(lcd_inf);
        lcd_inf++;
        if(lcd_inf>4) lcd_inf=0; 

        ch_lcd.reset();
    }

    //capteurs reception
    //sht31
    /*if(ch_sht31.isFinished())
    {
        myDatas.To = sht31.readTemperature();
        myDatas.Ho = sht31.readHumidity();
        lcd.setCursor(15, 1);
        lcd.print("~");
        ch_sht31.reset();
    }*/

    //dht22
    /*if(ch_dht22.isFinished())
    {
        if(dht22_TH)
            myDatas.Ti = dht.readTemperature();
        else
            myDatas.Hi = dht.readHumidity();

        dht22_TH!=dht22_TH;
        Serial.println(dht22_TH);
        ch_dht22.reset();
    }*/

    //lcd traitement -> permet d'eteindre ou d'allumer le lcd
    if(RangeInCentimeters<100)
    {
        if(!people)
        {
            Serial.println(RangeInCentimeters);
            Serial.println("People In");
            lcd.display();
            people = true;
            newPresence = true;
            myDatas.presence=true;
        }
        ch_lcdstop.reset();
    }
    else
    {
        people=false;
        newPresence = false;
        lcd.setCursor(14,1);
        lcd.print("  ");
        lcd.setCursor(14,1);
        lcd.print(ch_lcdstop.getDelta()/1000);
        if(ch_lcdstop.isFinished())
        {

            lcd.noDisplay();
            myDatas.presence=false;
            //Serial.print((deltatime-(millis()-passtime))/1000);
        }
        
    }

    //Serial.println(ch_send.getDelta());

    if(ch_send.isFinished() || newPresence) // lance la fonction si le timer d'envoie est fini ou si une nouvellle présence a ete detectee
    {
        newPresence = false;
        char t1str[6];
        char t2str[6];
        char pdstr[6];
        dtostrf( myDatas.Ti, 3, 1,t1str );//pour formate sous forme de string pour la fonction 'sprintf()'
        dtostrf( myDatas.To, 3, 2,t2str );
        dtostrf( myDatas.poids, 5, 0,pdstr );
        Serial.println("\n__\nSending");
        Serial.print("Ti :");
        Serial.println(myDatas.Ti);
        sprintf(command, "\"Ti:%s,Hi:%d,To:%s,Ho:%d,Pr:%d,Pd:%d\"", t1str, myDatas.Hi, t2str, myDatas.Ho,myDatas.presence, myDatas.poids );
        Serial.println("--");
        Serial.println(command);
        Serial.println("--");
        at_send("AT+MSG=, ", command, "+MSG: Done", 5000);

        at_send("AT+LW=ULDL, ","", "+LW:", 5000);//get frame counter
        Serial.println("__\n");
        ch_send.reset();
    }

    /*if(0)  //if(exist)
    {
        Serial.println("\n----start sending");

        //Serial.print("Ti : ");
        dtostrf(myDatas.Ti, 3, 1, t1Str); //transforme la valeur en string
        dtostrf(myDatas.Hi, 3, 0, h1Str); //transforme la valeur en string
        //dtostrf(myDatas.To, 3, 0, t2Str); //transforme la valeur en string
        //dtostrf(myDatas.Ho, 3, 0, h2Str); //transforme la valeur en string
        //dtostrf(myDatas.poids, 3, 0, pdStr); //transforme la valeur en string
        //dtostrf(myDatas.presence, 3, 0, prStr); //transforme la valeur en string

        sprintf(command, "\"T1:%s, H1:%d\"", t1Str, h1);
        at_send("AT+MSG=, ", command, "+MSG: Done", 5000);
        Serial.println(command);

        ch_send.reset();
    }*/
}

//fonctions 
int at_send(char* str1, char* str2, char* verif, int t_out)
{
    char buff[60];
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
        #if LORAINFO
            Serial.println("Setting LoRa infos");
            valid+=at_send("AT+ID=DevEui, ", DevEui, "+ID: DevEui", 2000);
            valid+=at_send("AT+ID=DevAddr, ", DevAddr, "+ID: DevAddr", 2000);
            valid+=at_send("AT+KEY=NWKSKEY, ", NwksKey, "+KEY: NWKSKEY", 2000);
            valid+=at_send("AT+KEY=APPSKEY, ", AppsKey, "+KEY: APPSKEY", 2000);
            valid+=at_send("AT+MODE= ", Mode, "+MODE: ", 2000);
            valid+=at_send("AT+DR= ", DR, "+DR: ", 2000);
            valid+=at_send("AT+CH= ", CH, "+CH: ", 2000);
            
        #endif
        #if !LORAINFO
            valid+=7;
        #endif

        valid+=at_send("AT+MSG", "", "+MSG: Done", 4000);

        if((valid>=8)) //depend du nombre d'AT command envoyée
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