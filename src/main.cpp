#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

//define Pin para dimmers
//Pin Motor
//#define PinPWM_left 2
#define PinPWM_rigth 18
#define channelPinA 5
//Señalizacion - paro de emergencia
#define  PinbtnStop 17 // NARANJA es el PARO Pin 17 dañado 
#define  PinbtnRun 23 // VERDE es el RUN
#define  PinRun  19
#define  PinStop 26
//valves 
#define PinOpen 32 
#define PinClose 33
#define PinOnOff 25

#define TIMER_INTERRUPT_DEBUG  3
#include "ESP32_TimerInterrupt.h"

//variables para establecer conexion
const char* ssid = "ESPE";//"Carlos";//"Depar"; //PSmart 2019";
const char* password = "";//"+-+/adgjl--++";//"12345678";//
const char* mqttServer = "10.2.129.73";//"192.168.43.167";//"192.168.137.19";//"192.168.43.76"; //192.168.0.100
const int mqttPort = 1883;

WiFiClient espClient;
PubSubClient client(espClient);
TaskHandle_t Task0;

//Variables globales******************************
volatile int Speed_mt=0;  //velocidad del motor 
volatile int pot_res=0;   //valor para variar la pontencia de la resistencia de la tolva
volatile int pot_res2=0;  //valor para varia la potencia de la resistencia del reactor
volatile int prc_vlv=0;   //valor para variar la apertura de la vlv proporcional
volatile int bit_vlAlv=0; // bit para abrir o cerrar el vlv de alivio

/*ESP32Timer ITimer1(1);
ESP32Timer ITimer2(2);
portMUX_TYPE crossMux = portMUX_INITIALIZER_UNLOCKED;
volatile int valor = 7650;
volatile int interruptCounter;*/

//variables motor y ajuste PWM**********************
const double  freq = 1000;
const uint8_t Channel_left = 0;
const uint8_t Channel_rigth = 5;
const uint8_t resolution = 8;

//lectura de velocidad del motor*********************
volatile int count=0;
portMUX_TYPE mux_enc = portMUX_INITIALIZER_UNLOCKED;
double rpm=0,aux=0;
//parametros filttro motor****************************
float Y_d = 0;  
float alpha_d=0.1;
float S_d=0.0;
volatile int S_d_int=0;
int porc = 0;
int porcenv = 0;
char rpmString[8];
//variables para control PID***************************
//float err_ac=0;
float err_an=0;
//float derror=0;
float err_int=0;
//float Parm_U=0;
float kp=0.80; 
float ki=0.85;  
float kd=0.04; 
unsigned long current_time, prev_time;
uint8_t dt_ms = 100;
//vlv proporcional*************************************
int valor_vlvAnt = 0;
int diff = 0; 
uint32_t tm_open = 3500000; //en microsegundos
uint32_t tm_close = 3400000; //en microsegundos
uint32_t tm_ac_close = 0;
uint32_t tm_ac_open = 0;
volatile bool band_op = false;
volatile bool band_cl = false;
ESP32Timer ITimer0(0);


bool inicia = false;
bool band = false;
bool publ = false;
char crr[8];

//****FUNCIONES *********//
void on_message(char* topic, byte* payload, unsigned int length);
void reconnect();
void setup_wifi();
void IRAM_ATTR encoder();
void IRAM_ATTR onTimer_vlv(void);
void IRAM_ATTR gate_off(void);
void IRAM_ATTR gate_pulse(void);
void IRAM_ATTR fnc_cruce();

int PID(int SP, int PV, uint8_t sp_ms);


void loop2(void *parameter){
  for (;;){
    if(digitalRead(PinbtnRun)){ //digitalRead(PinbtnRun)
      //Serial.println("\t\t\t\tEstoy corriendo");
      //Serial.println(valor);
      digitalWrite(PinRun,HIGH);
      digitalWrite(PinStop,LOW);
      current_time = millis();
      if(current_time-prev_time > dt_ms){ 
        aux=(count*1000)/(dt_ms);
        rpm=((aux*60)/334);
        
        Y_d = (float)rpm;
        S_d=(alpha_d*Y_d)+((1-alpha_d)*S_d);
        S_d_int=S_d;//velocidad en rpm filtrada
        //Serial.println(S_d_int);
        //ledcWrite(Channel_rigth,Speed_mt); //sin control
        porc = map(Speed_mt,0,100,0,255);
        //Serial.println(porc);
        ledcWrite(Channel_rigth,porc); //Con control //PID(Speed_mt,S_d_int,dt_ms)
        //ledcWrite(Channel_left,0);
        prev_time = current_time;
        portENTER_CRITICAL(&mux_enc);
        count = 0;
        portEXIT_CRITICAL(&mux_enc);
      }
      
      if(bit_vlAlv==1){
        digitalWrite(PinOnOff,HIGH);
      }else{
        digitalWrite(PinOnOff,LOW);
      }
      //Open of the valve
      diff = prc_vlv - valor_vlvAnt;
      if(diff>0 and diff<=100){
        //Open valve
        band_op = true;
        tm_ac_open = tm_open*(float)diff/100;
        //Serial.print(diff);Serial.print("; Abreindo ;");Serial.println(tm_ac_open); //float(73/5) no es lo mismo que (float)73/5
        ITimer0.setInterval(tm_ac_open,onTimer_vlv);
        digitalWrite(PinOpen,HIGH); //digitalRead(PinClose)^1
      }
      if(diff>=-100 and diff <0){
        //Close valve
        band_cl = true;
        tm_ac_close = tm_close*(float)diff/-100;
        //Serial.print(diff);Serial.print("; Cerrando ;");Serial.println(tm_ac_close);
        ITimer0.setInterval(tm_ac_close,onTimer_vlv);
        digitalWrite(PinClose,HIGH);//digitalRead(PinOpen)^1
      }
      valor_vlvAnt  = prc_vlv;
    }if(!digitalRead(PinbtnRun)){//digitalRead(PinbtnStop)
      //Serial.println("PARO DE EMERGENCIA ACTIVADO");
      digitalWrite(PinRun,LOW);
      digitalWrite(PinStop,HIGH);
      digitalWrite(PinOnOff,LOW);
      digitalWrite(PinOpen,LOW);
      digitalWrite(PinClose,LOW);
      //valor = 7650; //para que no se dispare nada de la onda en la resistencia de la tolva
      //ITimer1.stopTimer();
      //rst_tolva.setBrightness(0); 
      //rst_reactor.setBrightness(0);
      ledcWrite(Channel_rigth,0);
    }
    vTaskDelay(10);
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi(); 
  //Para establecer con el servidor
  client.setServer(mqttServer, mqttPort);
  client.setCallback(on_message);

  //ledcSetup(Channel_left,freq,resolution);
  //ledcAttachPin(PinPWM_left,Channel_left);
  ledcSetup(Channel_rigth,freq,resolution);
  ledcAttachPin(PinPWM_rigth,Channel_rigth);

  pinMode(PinbtnStop, INPUT_PULLDOWN);
  pinMode(PinbtnRun,INPUT_PULLDOWN);
  pinMode(PinRun, OUTPUT);
  pinMode(PinStop,OUTPUT);
  pinMode(PinOnOff,OUTPUT);
  pinMode(PinClose,OUTPUT);
  pinMode(PinOpen,OUTPUT);

  //pinMode(syncPin,INPUT);  
  //pinMode(dimmerPin, OUTPUT);

  digitalWrite(PinRun,LOW);
  digitalWrite(PinStop,LOW);
  digitalWrite(PinOnOff,LOW);
  digitalWrite(PinOpen,LOW);
  digitalWrite(PinClose,LOW);

  pinMode(channelPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelPinA),encoder, FALLING);
  //xTaskCreatePinnedToCore(loop2,"Actuators",1000,NULL,1,&Task0,0); 
  prev_time = millis();
}

void loop() {
  if(!client.connected()){
    if(inicia == true){
      Serial.println("Estoy dentro de loop 1");
      vTaskDelete(Task0);
      digitalWrite(PinRun,LOW);
      digitalWrite(PinStop,HIGH);
      digitalWrite(PinOnOff,LOW);
      digitalWrite(PinOpen,LOW);
      digitalWrite(PinClose,LOW);
      //detachInterrupt(digitalPinToInterrupt(syncPin));
      //rst_tolva.setBrightness(0);
      //rst_reactor.setBrightness(0);
      ledcWrite(Channel_rigth,0);
      inicia = false;
    }
    reconnect();
  }
  if(client.connected()){
    //Aqui siempre va ha estar entrando si es que estamos conectados al servidor 
    //Aqui se deben de enviar los mensajes hacia el servidor 
    //valor = map(pot_res2,0,100,7650,50);
    if(digitalRead(PinbtnRun)){
      if(band == true){
        dtostrf(1,5,2,crr);
        client.publish("planta/Stop",crr);//uno corriendo
        band = false;
      }
    }
    if(!digitalRead(PinbtnRun)){
      if (band == false){
        dtostrf(2,5,2,crr);
        client.publish("planta/Stop",crr);//dos paro de emergencia
        band = true;
      }
    }
    //porcenv = map(S_d_int,0,47,0,100);
    //Serial.println(porcenv);
    dtostrf(S_d_int,5,2,rpmString);
    client.publish("planta/reactor/rpm",rpmString);
    if(inicia==false){
      xTaskCreatePinnedToCore(loop2,"Actuators",10000,NULL,1,&Task0,0);
      inicia = true;
    }
  }
  client.loop(); 
}

void IRAM_ATTR encoder(){
  portENTER_CRITICAL_ISR(&mux_enc);
  count++;
  portEXIT_CRITICAL_ISR(&mux_enc);
}

void IRAM_ATTR onTimer_vlv(void) {  
 if (band_op){
    band_op = false;
    digitalWrite(PinOpen,LOW);
    //Serial.println("Termino ese porcentaje de Apertura");
    }
  if(band_cl){
    band_cl= false;
    digitalWrite(PinClose,LOW);
    //Serial.println("Termino ese  porcentaje Cerrar");
    }
}

int PID(int SP, int PV, uint8_t sp_ms){
  float err_ac = SP - PV;
  float derror = (err_ac - err_an)/(sp_ms/1000);
  err_int = err_int + err_ac*(sp_ms/1000);
  float Parm_U=kp*err_ac+ki*err_int+kd*derror;
  if(Parm_U>=255)Parm_U=255;
  if(Parm_U<0)Parm_U=0; 
  if(SP==0)Parm_U=0;
  err_an=err_ac;
  return Parm_U;
}

void setup_wifi() {
  //Conectamos a una red Wifi
  delay(10);
  Serial.println();
  Serial.print("Conectando a ssid: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println("Direccion IP:");
  Serial.println(WiFi.localIP());
}


void reconnect(){
  while (!client.connected()){
    Serial.println("Connecting to MQTT.................................................................");
    //Crea un cliente ID
    String clientId = "Micro_ESP32";
    clientId += String(random(0xffff),HEX);
    //Intentamos conectar
    if (client.connect(clientId.c_str())) {
      Serial.println("*******CONNECTED*******");
      //Sucribirse a los temas 
      client.subscribe("planta/tolva/val_Pro");
      client.subscribe("planta/tolva/res_elec");
      client.subscribe("planta/reactor/resi_elec");
      client.subscribe("planta/reactor/motor");
      client.subscribe("planta/reactor/val_alv");
    }else{
      Serial.print("failed with state -->");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void on_message(char* topic, byte* payload, unsigned int leng){
  String incoming = "";
  for(int i=0; i< leng; i++){
        incoming += (char)payload[i];
    } 
  incoming.trim();

  if(String(topic) == "planta/tolva/val_Pro"){
    //Serial.print(topic);
    prc_vlv = atoi(incoming.c_str());
    //Serial.println("Mensaje ->" + incoming);
  }
  
  if(String(topic) == "planta/tolva/res_elec"){
    //Serial.print(topic);
    pot_res = atoi(incoming.c_str());
    //Serial.println(" Mensaje ->" + incoming);
  }

  if (String(topic) == "planta/reactor/resi_elec"){
    //Serial.print(topic);
    pot_res2 = atoi(incoming.c_str());
    //Serial.println(" Mensaje ->" + incoming);
  }
  
  if(strcmp(topic,"planta/reactor/motor") == 0){
    Speed_mt = atoi(incoming.c_str());
    Serial.println(" Mensaje ->" + incoming);
   }
   
  if(String(topic) == "planta/reactor/val_alv"){
    //Serial.print(topic);
    bit_vlAlv = atoi(incoming.c_str());
    //Serial.println(" Mensaje ->" + incoming);
  }
}