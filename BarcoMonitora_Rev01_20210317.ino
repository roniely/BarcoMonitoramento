 

/*********
 * 
 * PROJETO DE POS- GRADUACAO SENAI MARIANO FERRAZ
 * BARCO REMOTO PARA MONITORAR E COLETAR OS DADOS DA AGUA
 * 
  AUTOR:RONIELY PATRIOTA
*********/
  
//Bibliotecas
#include <Wire.h>                     // Biblioteca I2c pino D22-SCL e pino D21-SDA
#include <Adafruit_BMP085.h>          // Biblioteca barometro deve ser ligado em 3-VCC
#include <Adafruit_Sensor.h>          // Biblioteca Adafruit para sensores
#include <OneWire.h>                  // Biblioteca para o sensor de temperatura de agua
#include <DallasTemperature.h>        // Biblioteca Adafruit para sensores
#include <TinyGPS++.h>                // Biblioteca do GPS
#include <SoftwareSerial.h>           // Biblioteca da comunicacao serial         
#include <Servo.h>                    // Biblioteca do Servo motor-Sg90
#include <DHT.h>                      // Biblioteca sensor DHT-11
#include <analogWrite.h>              // Biblioteca de leitura das portas analogicas
#include <ESPmDNS.h>
#include <DFRobot_ESP_PH.h>           // Biblioteca do sensor pH
#include <EEPROM.h>                   // Biblioteca habilitar memoria EEPROM
#include <WiFi.h>                     // Biblioteca do Wifi
#include <WiFiClientSecure.h>         // Biblioteca do WifiSecure
#include "secrets.h"                  // Chamada do arquivo anexado com as informacacoes de seguranca da plataforma AWS
#include <MQTTClient.h>               // Biblioteca para comunicação MQTT
#include <ArduinoJson.h>              // Biblioteca para envio de protocolo Json
#include <HTTPClient.h>               // Biblioteca para comunicacao do protocolo HTTP
Servo myservo;                        // Propriedade da biblioteca do Servo Motor
DFRobot_ESP_PH ph;                    // Propriedade da biblioteca do sensor pH
#define ESPADC 4096.0                 // Conversao Analogico Digital
#define ESPVOLTAGE 5000               // Voltage de Alimentacao ESP32
//#define ESPVOLTAGE 3300             // Voltage de Alimentacao ESP32 via bateria
#define PH_PIN A0                     // Definicao da entrada Analogica do sensor pH
                                                   
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub" // MQTT topicos de escrita na plataforma AWS publish/subscribe
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"


WiFiClientSecure net = WiFiClientSecure();   // Definicao da variavel para uso da biblioteca WiFiClientSecure
MQTTClient client = MQTTClient(256);         // Definicao da variavel para uso da biblioteca MQTT 

//==================== Credencial da rede Wifi===========================================

const char* ssid = "Shirley";
const char* password = "joao29112008";

//==================== FIM - Credencial da rede Wifi=====================================



//==================== DEFINIÇÃO DA PINAGEM=================================================


const int trigPin = 12;                   // Entrada do sonar
const int echoPin = 13;                   // Saida do sonar

#define RXD2 16                           // Comunicacao serial para o sensor gps
#define TXD2 17                           // Comunicacao serial para o sensor gps
static const uint32_t GPSBaud = 9600;     // Define a Velocidade de comunicacao do GPS


Adafruit_BMP085 bmp;
static const int PinServoMotor=23;        // Define a pinagem utilizada para o servo motor
const int output26 = 26;                  // Definição dos pinos de saida do motor hélice
const int output27 = 27;                  // Definição dos pinos de saida do motor hélice
uint8_t SensorTempHumi = 4;               // Entrada do sensor de temperatura DHT-11

#define DHTTYPE    DHT11                  // Modelo do sensor utilizado DHT-11
DHT dht(SensorTempHumi, DHTTYPE);         // Define a pinagem utilizada e o modelo do sensor 
OneWire  ds(19);                          // Definicao do pino do sensor de temperatura submerso a 4.7K resistor e necessario)
TinyGPSPlus gps;                          // Objeto da biblioteca gps
SoftwareSerial ss(RXD2, TXD2);

//==================== INICIALIZACAO DA PORTA DO SERVIDOR===========================================
//AsyncWebServer server(80);                    // Criação AsyncWebServer  na porta 80
WiFiServer server(80);
HTTPClient http;                                //Inicio do HTTP
//==================== VARIAVEIS INTERNAS===========================================
long duration,cm,Sonar;                        //Variavel para o calculo da distancia
int TempAgua=0;                                //Temperatura da agua
int action = 0;                                // 0 = Stop, 1 = Forward, 2 = Reverse
int speed = 80;                                // Velocidade do Fan Motor velocidade minimia 80 até 255 
int velocidade=0;
String SAIDA26_status = "off";                  // Saidas logicas do Fan Motor 
String SAIDA27_status = "off";                 // Saidas logicas do Fan Motor 
String awsUpdate = "off";
int motorState = 0;                            // Valor do Status do Motor
int lastMotorState = 0;                        // Último estado do Motor
                                             
String valueString = String(5);
String valueTmp = String(5);
int pos1 = 0;
int pos2 = 0;
int pos3 = 0;

int valueINT;
                                              
String header;
                                                  // Variaveis para os dados provenientes do sensor GPS
float latitude , longitude,pressaAtm,Pressure,Altitude,TemperaturaBmp;
String date_str , time_str , lat_str , lng_str;
int pm;    
                                                 // Variaveis para armazenar os valores do sensor DHT11
String TempAmbiente,UmidadeAr;
int j=0 ;                                        // variavel de memoria 

                                                 // Variaveis para armazenar os comandos automatico, manual.
byte comandoAutoManual;
float voltage, phValue, temperature = 25;        // Variaveis do sensor Ph.


volatile int interruptCounter;
int totalInterruptCounter;
 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//==================== FIM ===================================================


//==================== DEFINICAO IP ===========================================

IPAddress local_IP (192, 168, 0, 50);            // Defini um endereco IP estatico
IPAddress gateway(192, 168, 0, 1);               // Defini um gateway
IPAddress subnet(255, 255, 255, 0);              // Defini uma submascara
IPAddress primaryDNS(8, 8, 8, 8);   //optional   // Defini o DNS padrao

//==================== FIM===========================================

//================Função do sensor Barometro=========================

String readBarometro() { 

   // Pressure =bmp.readPressure();
    pressaAtm=bmp.readPressure();
    Altitude =bmp.readAltitude();
    TemperaturaBmp =bmp.readTemperature();   
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");    
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");
    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");
 
  if (isnan(pressaAtm)) {    
    Serial.println("Falha na leitura do sensor Barometro!");
    return "--";
  }
  else {
    Serial.println(pressaAtm);
    return String(pressaAtm);
  }
}

//================TEMPERATURA- AGUA=========================
void readTemperaturaAgua() { 
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Dispositivo não é da familia DS18x20.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        
  delay(1000);    
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);        
  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3;             // 9 bit resolução padrão
    if (data[7] == 0x10) {
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolucao, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit resolucao, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit resolucao, 375 ms
    
  }
  celsius = (float)raw / 16.0;
  Serial.println("  Temperature Celsius = ");
  Serial.print(celsius);
  TempAgua=celsius;
 
}
//================SENSOR DTH 11- TEMPERATURA-AR=========================
String readDHTTemperature() {  
  // A leitura da temperatura é realizada em graus Celsius.
  float t = dht.readTemperature();
  TempAmbiente=dht.readTemperature();  
  if (isnan(t)) {    
    Serial.println("Falha para ler o DTH sensor!!");   
    return "--";
  }
  else {
    Serial.println(t);
    return String(t);
  }
}
//================SENSOR DTH 11 - UMIDADE =========================

String readDHTHumidity() {
  float h = dht.readHumidity();
  UmidadeAr=dht.readHumidity();

  if (isnan(h)) {
        Serial.println("Falha para ler o DTH sensor!!");
    return "--";
  }
  else {
    Serial.println(h);
    return String(h);
  }
}

//===================CHAMADAS DAS FUNCOES DE AVANCO,PARADA,REVERSO DO MOTOR-FAN==========
void reverse(){
  analogWrite(output26,0);
  analogWrite(output27,speed);   
}
void forward(){
   
 if (cm <=10){
    Serial.print("Atencao obstaculo");
    stopFan();
             }  
}

void stopFan(){
  Serial.print("PARADA");  
  digitalWrite(output27,HIGH);
  digitalWrite(output26, HIGH);  
}

//=====================================FIM-MOTOR FAN======================================

//=====================================CHAMADAS DAS FUNCOES SONAR=========================
void sonar(){
  
// The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Conversão do tempo pela distancia
  cm = (duration/2) / 29.1;     // Dividido por 29.1             
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  Sonar=cm;
  
    
}

//=====================================FIM-SONAR====================================
//=================================================GPS===================================
void callGPS()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
 //   latitude=(gps.location.lat(), 6);
  //  longitude=(gps.location.lng(), 6);
    latitude=-23,85;
    longitude=-46,74;
    
  }
  else
  {
    Serial.print(F("INVALID"));
    latitude=-23,85;
    longitude=-46,74;
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}  //END callGPS
//=================================================FIM GPS===================================

//=================================================Sensor PH===================================

void SensorpH()
{
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) //time interval: 1s
  {
    timepoint = millis();
    //voltage = rawPinValue / esp32ADC * esp32Vin
    voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
    Serial.print("voltage:");
    Serial.println(voltage, 4);
    
    //temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
    Serial.print("temperature:");
    Serial.print(TempAgua, 1);
    Serial.println("C");

    phValue = ph.readPH(voltage, TempAgua); // convert voltage to pH with temperature compensation
    
    Serial.print("pH:");
    Serial.println(phValue, 4);
  }
  ph.calibration(voltage, TempAgua); // calibration process by Serail CMD
}

//=================================================END PH===================================
//=================================================INICIO-ENVIO DOS DADOS VIA AWS===================================

void connectAWS()
{

  // Configuracao das credencias AWS
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Conexao AWS via protocolo MQTT
  client.begin(AWS_IOT_ENDPOINT, 8883, net);
  client.onMessage(messageHandler);
  Serial.println("Conectando para AWS IOT ");
  while (!client.connect(THINGNAME)) {
    Serial.println(".");
    delay(100);
  }
  if(!client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
  Serial.println("AWS IoT Connected!");
  j= 1;
}

void publishMessage()   // publicacao das variaveis na plataforma Aws.
{
   int rowTableValue, posTableValue,i;
   rowTableValue=0;
   posTableValue=0;
  
 for ( i = 0; i < 9; i++) {           // O numero de tabela que sera escrito no Dynamo DB
    i = i++;
    Serial.println(i);
  
  
  StaticJsonDocument<200> doc;
  //doc["time"] = millis();
 // doc["sensor_a0"] = analogRead(0);
 
 doc["row"] = i;    //Linha que sera gravada no Dynamo DB
 doc["pos"] = i;   // Linha que sera gravada no Dynamo DB
 doc["Pressao_Atm"] = pressaAtm;
 doc["sensor_pH"] = phValue;
 doc["sensor_TempAgua"] = TempAgua;
 doc["sensor_TempAr"] = TempAmbiente;
 doc["sensor_UmidadeAr"] =  UmidadeAr;
 doc["Pressao Atmosferica"] = pressaAtm;
 //doc["Latitude"] = gps.location.lat(), 6;
 //doc["Longitude"] = gps.location.lng(), 6;
  doc["Latitude"] = latitude;
  doc["Longitude"] = longitude;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
  }
  
}

void messageHandler(String &topic, String &payload) {
Serial.println("incoming: " + topic + " - " + payload);

}


//=================================================END-ENVIO DOS DADOS VIA AWS===================================

// INTERRUPCAO
void IRAM_ATTR onTimer() {
  
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
  Serial.println("Interrupcao Ativada");}
  


//==================================SETUP===================================
void setup(){

//=================ENTRADAS DIGITAIS===========================

pinMode(echoPin, INPUT);  //entrada do sonar
//=================SAIDAS DIGITAIS===========================
pinMode(output26,OUTPUT); 
pinMode(output27,OUTPUT); 
digitalWrite(output26,LOW);
digitalWrite(output27,LOW);
pinMode(trigPin, OUTPUT); //saida do sonar

//Declaracao da Interrupcao
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 12000000, true);  // DECLARACAO DO TEMPO DA INTERRUPCAO ~12 s. 
  timerAlarmEnable(timer);
 
//=================INICIALIZA AS BIBLIOTECAS===========================
 
  Serial.begin(115200);//porta serial
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.println("Serial Txd is on pin: "+String(TXD2));
  Serial2.println("Serial Rxd is on pin: "+String(RXD2));
  
  dht.begin(); //sensor de temperatura umidade DHT 11
  myservo.attach(PinServoMotor); // Define a pinagem do servo motor
  bmp.begin();
  ss.begin(GPSBaud);
  EEPROM.begin(32);//needed to permit storage of calibration value in eeprom
  ph.begin();
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);
  if (!WiFi.config(local_IP, gateway, subnet,primaryDNS)) {
  Serial.println("Nao conectado");}

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Conectando para ");
  Serial.println(ssid);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");}
                                                    // Inicia o servidor Web e mostra o Ip da Rede
  Serial.println("");
  Serial.println("WiFi conectado.");
  Serial.println("Endereco IP: ");
  Serial.println(WiFi.localIP()); 
  Serial.println("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Sub mascara: ");
  Serial.println(WiFi.subnetMask());
  server.begin();                                     //    Inicia o ESP32-Servidor
  Serial.println("HTTP Servidor Iniciado..."); }




//========================================================================CHAMADAS DA LOOP========================================

void loop(){


  while (j<=0) {
    connectAWS();                 //    Chama a rotina para conectar na plataforma AWS e verificar as chaves de segurança
    }
   WiFiClient client = server.available();   // Escuta quando houver um cliente 

    pressaAtm=92368;
    Altitude =790;
    
  
    
//=======================================================================INTERRUPCAO INTERNA===========================================  
  
if (interruptCounter > 0) {
 
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    totalInterruptCounter++;
    Serial.print("An interrupt as occurred. Total number: ");   
    Serial.println(totalInterruptCounter);

    sonar();                    //chamada da funcao Sonar.
    callGPS();                  //chamada da funcao GPS.
  //  readBarometro();            //chamada da funcao barometro
    publishMessage();           //chamada da funcao AWS para publicacao dos dados.
    readDHTTemperature();       //chamada da funcao sensor temperatura ambiente
    readDHTHumidity();          //chamada da funcao sensor umidade
    readTemperaturaAgua();      //chamada da funcao temperatura agua
    SensorpH();                 //chamada da funcao pH 
    
    //j= 0; // flag desligada para nao gerar cobranca AWS
   }  
// ========================================================================FIM CHAMADAS DE ROTNAS=======================================
  
  if (client) {                             // Conexao de um novo cliente
    Serial.println("Nova conexao.");        // Mostra a mensagem de uma nova conexão
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connecte
    
    if (client.available()) {               // if there's bytes to read from the client,
        char c = client.read();             // Leitura de nova byte
        Serial.write(c);                    
        header += c; 
        if (c == '\n') { 
          if (currentLine.length() == 0) {
            // HTTP cabeçalho aguarda o inicio da resposta (e.g. HTTP/1.1 200 OK)
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
       
              //Teste do valor vindo do app do celular
            if(header.indexOf("GET /pos/20")>=0) 
            {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              //valueString = header.substring(pos1+1, pos2);             
              valueString = 20;              
              myservo.write(valueString.toInt()); //Rotação do Servo Motor
              Serial.println(valueString);}  
              else if (header.indexOf("GET /pos/90") >= 0) {
              valueString = 90;              
              myservo.write(valueString.toInt()); //Rotação do Servo Motor
              Serial.println(valueString);}
              else if (header.indexOf("GET /pos/55") >= 0) {
              valueString = 52;              
              myservo.write(valueString.toInt()); //Rotação do Servo Motor
              Serial.println(valueString);}    
             // Comando para ligar/desligar o Fan Motor
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              SAIDA26_status = "on";
              if (cm < 3){
              digitalWrite(output26, LOW);
              }
            
              digitalWrite(output26, HIGH);     
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              SAIDA26_status = "off";
              digitalWrite(output26, LOW);
                  
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              SAIDA27_status = "on";
              digitalWrite(output27, HIGH); 
            
              
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              SAIDA27_status = "off";
              digitalWrite(output27, LOW);
              
            }       
            
            // Cabecalho página em  HTML 
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");   
            client.println("<link rel=\"icon\" href=\"data:,\">");

            // Utiliza a bilioteca CSS para montar o estilo dos botões e as cores utilizadas.   
            client.println("<style>body { text-align: center; font-family: \"Trebuchet MS\", Arial; margin-left:auto; margin-right:auto;}");
            client.println(".slider { width: 300px; }</style>");
            client.println("<script src=\"https://ajax.googleapis.com/ajax/libs/jquery/3.3.1/jquery.min.js\"></script>");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 12px 45px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
           
           // VALOR DO POSICIONAMENTO DO MOTOR
            client.println("</head><body><h1>BARCO MONITORAMENTO </h1>");
            client.println("<p>Position: <span id=\"servoPos\"></span></p>");          
            client.println("<input type=\"range\" min=\"10\" max=\"100\" class=\"slider\" id=\"servoSlider\" onchange=\"servo(this.value)\" value=\""+valueString+"\"/>");       
            client.println("<script>var slider = document.getElementById(\"servoSlider\");");
            client.println("var servoP = document.getElementById(\"servoPos\"); servoP.innerHTML = slider.value;");
            client.println("slider.oninput = function() { slider.value = this.value; servoP.innerHTML = this.value; }");
            client.println("$.ajaxSetup({timeout:1000}); function servo(pos) { ");
            client.println("$.get(\"/?value=\" + pos + \"&\"); {Connection: close};}</script>");
            
//==========================================BOTOES==================================================================================
           
            // Botoes de comando na inteface da pagina web.
                  
            client.println("<p>GPIO 26 - State " + SAIDA26_status+ "</p>");
           
            if (SAIDA26_status=="off") {
            
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");          
              
            } else 
                  {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            
            client.println("<p>GPIO 27 - State " + SAIDA27_status + "</p>");
            if (SAIDA27_status=="off") {
              
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
             
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

           
//==================================================SENSOR DE TEMPERATURA e UMIDADE====================================

          client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
          client.println("body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n");
          client.println("p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n");
          client.println( "</style>\n");
          client.println("<div id=\"webpage\">\n");
          client.println("<p>Temperatura Ar: ");
          client.println(TempAmbiente);
          client.println("°C</p>");
          client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
          client.println("body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n");
          client.println("p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n");
          client.println( "</style>\n");
          client.println("<div id=\"webpage\">\n");
          client.println("<p>Umidade Ar: ");
          client.println(UmidadeAr);
          client.println("°%</p>");
//================================================== FIM =====================================================================
//==================================================SONAR====================================================================
          client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
          client.println("body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n");
          client.println("p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n");
          client.println( "</style>\n");
          client.println("<div id=\"webpage\">\n");
          client.println("<p>Sonar: ");
          client.println(Sonar);
          client.println("Cm</p>");
//================================================== FIM =====================================================================

//===============================================SENSOR DE TEMPERATURA AGUA e PH . ====================================

          client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
          client.println("body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n");
          client.println("p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n");
          client.println( "</style>\n");
          client.println("<div id=\"webpage\">\n");
          client.println("<p>Temperatura Agua: ");
          client.println(TempAgua);
          client.println("°C</p>");    
          client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
          client.println("body{margin-top: 10px;} h1 {color: #444444;margin: 80px auto 30px;}\n");
          client.println("p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n");
          client.println( "</style>\n");
          client.println("<div id=\"webpage\">\n"); 
          client.println("<p>pH Agua: ");
          client.println(phValue);
          client.println("</p>");
          
//================================================== FIM =====================================================================

///=============================================== BAROMETRO ================================================================
          client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: right;}\n");
          client.println("body{margin-top: 10px;} h1 {color: #444444;margin: 90px auto 10px;}\n"); // estava 150 e 30
          client.println("p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n");
          client.println( "</style>\n");
          client.println("<div id=\"webpage\">\n");
          client.println("<p>Barometro: ");
          client.println(pressaAtm);
          client.println("Pa</p>");
//================================================== FIM =====================================================================

///============================================ Valor GPS ====================================================================
          client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
          client.println("body{margin-top: 50px;} h1 {color: #444444;margin: 150px auto 10px;}\n"); // estava 15 e 30
          client.println("p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n");
          client.println( "</style>\n");
          client.println("<div id=\"webpage\">\n");
          client.println("<p>Latitude: ");
          //client.println(gps.location.lat(), 6);
          client.println(latitude);
    
          client.println("-</p>");
          client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
          client.println("body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n");
          client.println("p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n");
          client.println( "</style>\n");
          client.println("<div id=\"webpage\">\n");
          client.println("<p>Longitude: ");
          //client.println(gps.location.lng(), 6);
          client.println(longitude);
          client.println("-</p>");
   
//========================================================FIM GPS==========================================================================

            client.println("</body></html>"); 
                       
            if(header.indexOf("GET /?value=")>=0) {  //GET /?value=180& HTTP/1.1
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1+1, pos2);             
              myservo.write(valueString.toInt()); //Rotação do Servo Motor
              Serial.println(valueString);
     
               
            }         
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;//verificar esse
            }  else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
 
}//Fim do VOID LOOP



  










  
