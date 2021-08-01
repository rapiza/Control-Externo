#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "PSmart 2019"; //"Depar";
const char* password = "12345678";//"+-+/adgjl--++";
const char* mqttServer = "192.168.0.101"; //192.168.0.100
const int mqttPort = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

//Variables globales 
volatile int Speed_mt=0;
volatile int pot_res=0;
volatile int pot_res2=0;
volatile int prc_vlv=0;
volatile int bit_vlAlv=0;

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setup_wifi(); 
  //Para establecer con el servidor
  client.setServer(mqttServer, mqttPort);
  client.setCallback(on_message);  
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
