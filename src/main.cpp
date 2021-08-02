#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <dimmable_light.h>

//variables para establecer conexion
const char* ssid = "PSmart 2019"; //"Depar";
const char* password = "12345678";//"+-+/adgjl--++";
const char* mqttServer = "192.168.0.101"; //192.168.0.100
const int mqttPort = 1883;

WiFiClient espClient;
PubSubClient client(espClient);
TaskHandle_t Task0;

//Variables globales 
volatile int Speed_mt=0; //velocidad del motor 
volatile int pot_res=0;  //valor para variar la pontencia de la resistencia de la tolva
volatile int pot_res2=0; //valor para varia la potencia de la resistencia del reactor
volatile int prc_vlv=0;  //valor para variar la apertura de la vlv proporcional
volatile int bit_vlAlv=0; // bit para abrir o cerrar el vlv de alivio

//define los datos para los dimmers
const int syncPin = 23;
const int dimmerPin = 22;
const int dimmerPin2 = 21;

DimmableLight rst_tolva(dimmerPin);
DimmableLight rst_reactor(dimmerPin2);

//variables motor y ajuste PWM
const uint8_t PinPWM_left = 19;
const uint8_t PinPWM_rigth = 18;
const double  freq = 10000;
const uint8_t Channel_left = 0;
const uint8_t Channel_rigth = 5;
const uint8_t resolution = 8;
//lectura de velocidad del motor
const uint8_t channelPinA = 5;
int count=0;
double rpm=0,aux=0;
//parametros filttro motor
float Y_d = 0;  
float alpha_d=0.1;
float S_d=0.0;
volatile int S_d_int=0;
//variables para control PID
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

//****FUNCIONES *********//
void on_message(char* topic, byte* payload, unsigned int length);
void reconnect();
void setup_wifi();
void encoder();
int PID(int SP, int PV, uint8_t sp_ms);


void loop2(void *parameter){
for (;;){
  current_time = millis();
  if(current_time-prev_time > dt_ms){ 
    aux=(count*1000)/(dt_ms);
    rpm=((aux*60)/224.4);

    Y_d = (float)rpm;
    S_d=(alpha_d*Y_d)+((1-alpha_d)*S_d);
    S_d_int=S_d;//velocidad en rpm filtrada

    //ledcWrite(Channel_rigth,Speed_mt); //sin control
    ledcWrite(Channel_rigth,PID(Speed_mt,S_d_int,dt_ms)); //Con control
    ledcWrite(Channel_left,0);
    prev_time = current_time;
    count = 0;
  }
  rst_tolva.setBrightness(pot_res);
  rst_reactor.setBrightness(pot_res2);
  }
  vTaskDelay(10);
}

void setup() {
  Serial.begin(115200);
  setup_wifi(); 
  //Para establecer con el servidor
  client.setServer(mqttServer, mqttPort);
  client.setCallback(on_message);  
  DimmableLight::setSyncPin(syncPin);
  DimmableLight::begin();

  ledcSetup(Channel_left,freq,resolution);
  ledcAttachPin(PinPWM_left,Channel_left);
  ledcSetup(Channel_rigth,freq,resolution);
  ledcAttachPin(PinPWM_rigth,Channel_rigth);

  pinMode(channelPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelPinA),encoder, FALLING);
  xTaskCreatePinnedToCore(loop2,"Actuators",1000,NULL,1,&Task0,0);
  prev_time = millis();
}

void loop() {
  if(!client.connected()){
     reconnect();
    }
  if(client.connected()){
    //Aqui siempre va ha estar entrando si es que estamos conectados al servidor 
    //Aqui se deben de enviar los mensajes hacia el servidor 
    
    }
  client.loop(); 
}

void encoder(){
  count++;
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
    Serial.println("Connecting to MQTT...");
    //Crea un cliente ID
    String clientId = "Micro_ESP32";
    clientId += String(random(0xffff),HEX);
    //Intentamos conectar
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      //Sucribirse a los temas 
      client.subscribe("planta/tolva/val_Pro");
      client.subscribe("planta/tolva/res_elec");
      client.subscribe("planta/reactor/resi_elec");
      client.subscribe("planta/reactor/motor");
      client.subscribe("planta/reactor/val_alv");

    } else {
      Serial.print("failed with state");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void on_message(char* topic, byte* payload, unsigned int leng){
  //Serial.println("In core -> "+ String(xPortGetCoreID()));
  String incoming = "";
  if (String(topic) == "planta/tolva/val_Pro"){
      Serial.print(topic);
      for(int i=0; i< leng; i++){
        incoming += (char)payload[i];
      }
      incoming.trim();
      prc_vlv = atoi(incoming.c_str());
      Serial.println("Mensaje ->" + incoming);
    }
  
  if(String(topic) == "planta/tolva/res_elec"){
      Serial.print(topic);
      for(int i=0; i< leng; i++){
        incoming += (char)payload[i];
      }
      incoming.trim();
      pot_res = atoi(incoming.c_str());
      Serial.println(" Mensaje ->" + incoming);
    }

  if (String(topic) == "planta/reactor/resi_elec"){
      Serial.print(topic);
      for(int i=0; i< leng; i++){
        incoming += (char)payload[i];
      }
      incoming.trim();
      pot_res2 = atoi(incoming.c_str());
      Serial.println("Mensaje ->" + incoming);
   }
  
  if(String(topic) == "planta/reactor/motor"){
      Serial.print(topic);
      for(int i=0; i< leng; i++){
          incoming += (char)payload[i];
      }
      incoming.trim();
      Speed_mt = atoi(incoming.c_str());
      Serial.println(" Mensaje ->" + incoming);
   }
   
  if(String(topic) == "planta/reactor/val_alv"){
      Serial.print(topic);
      for(int i=0; i< leng; i++){
          incoming += (char)payload[i];
      }
      incoming.trim();
      bit_vlAlv = atoi(incoming.c_str());
      Serial.println(" Mensaje ->" + incoming);
   }
}