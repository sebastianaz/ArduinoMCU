#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#define openClose 14 // Gpio14 == D5 (Swiche)
#define blanco    13 // Gpio13 == D7
#define amarillo  12 // Gpio12 == D6 
#include "Adafruit_SHT4x.h"
/************************Hardware Related MQ135************************************/                    //when the calibration start , LED pin 13 will light up , off when finish calibrating
const int sensorgas = 17; // salida analogica GPIO17 == A0
float RL_VALUE = 1.0;                                     //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR =3.574;                     //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,which is derived from the chart in datasheet
/***********************Software Related MQ135************************************/
int CALIBARAION_SAMPLE_TIMES=50;                    //define how many samples you are going to take in the calibration phase
int CALIBRATION_SAMPLE_INTERVAL=500;                //define the time interal(in milisecond) between each samples in the cablibration phase
int READ_SAMPLE_INTERVAL=50;                        //define how many samples you are going to take in normal operation
int READ_SAMPLE_TIMES=5;                            //define the time interal(in milisecond) between each samples in normal operation
/**********************Application Related MQ135**********************************/
#define         GAS_NH4             0   
#define         GAS_CO2             1 
#define         GAS_LPG             2 
/*****************************Globals***********************************************/
float           NH4Curve[2]  =  {101.92,-2.476};  //two points are taken from the curve.  
float           CO2Curve[2]  =  {111.94,-2.87};   //two points are taken from the curve.  
float           LPGCurve[2]  =  {44.458,-3.477};    //two points are taken from the curve.                                          
float           Ro           =  10.0;             //Ro is initialized to 10 kilo ohms.
//-------------------VARIABLES GLOBALES--------------------------
int contconexion = 0;
int analog;
float temp;
float humi;
float lume;
float gass;
unsigned long previousMillis = 0;
String strNivel_h2o;
String strNivel_h2oUltimo;
String strHTLG = "";
String strLUMI;
char charNivel_h2o [15];
char PLACA[50];
char valueStr[15];
char HTLG[50]; //Humedad-Temperatura-Luminosidad-gases
char NIVEL_H2O[50];//sensor infrarojo de nivel de agua
char SALIDAANALOGICA[50];
char LUZAB[50];

//actualizacion 27/01/2021
const char *ssid     = "Altavista";
const char *password = "29702587";
char   SERVER[50]    = "3.81.179.172";
int    SERVERPORT    = 16594;
String USERNAME      = "engorde";   
char   PASSWORD[50]  = "engorde"; 
//-------------------------------------------------------------------------
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

//------------------------MQ135--------------------------------
float MQResistanceCalculation(int raw_adc){
  float aux =  RL_VALUE*(1023-raw_adc)/raw_adc;
  return aux;
  }
//======================================================
float MQCalibration(int sensorgas){
  int i;
  float val=0;
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(sensorgas));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
  return val;                                                      //according to the chart in the datasheet 
}
//======================================================
float MQRead(int sensorgas){
  int i;
  float rs=0;
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(sensorgas));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}
//======================================================
long MQGetGasPercentage(float rs_ro_ratio, int gas_id){
  if ( gas_id == GAS_NH4 ) {
     return MQGetPercentage(rs_ro_ratio,NH4Curve);
  } else if ( gas_id == GAS_CO2 ) {
     return MQGetPercentage(rs_ro_ratio,CO2Curve);
  } else if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  }    
  return 0;
}
//======================================================
long  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{return (pcurve[0]*(pow(rs_ro_ratio, pcurve[1])));}

//------------------------CALLBACK-----------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  char PAYLOAD[5] = "    ";
  Serial.print("Mensaje Recibido: [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    PAYLOAD[i] = (char)payload[i];
  }
  Serial.println(PAYLOAD);
  if (String(topic) ==  String(LUZAB)) {
    if (payload[0] == 'A'){//AMARILLO
      digitalWrite(amarillo, HIGH);delay(300);
      digitalWrite(blanco, LOW);
    }
    if (payload[0] == 'B'){//BLANCO
      digitalWrite(blanco, HIGH);delay(300);
      digitalWrite(amarillo,LOW);
    }
    if (payload[0] == 'F'){//TODO APAGADO
      digitalWrite(blanco, LOW);
      digitalWrite(amarillo,LOW);
    }
  }
}

//------------------------RECONNECT-----------------------------
void reconnect() {
  uint8_t retries = 3;
  // Loop hasta que estamos conectados
  while ( !client.connected()) {
    Serial.print("Intentando conexion MQTT...");
    // Crea un ID de cliente al azar
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    USERNAME.toCharArray(PLACA, 50);
    if (client.connect("", PLACA, PASSWORD)) {
      Serial.println("conectado");
      client.subscribe(LUZAB);
    } else {
      Serial.print("fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intenta nuevamente en 5 segundos");
      // espera 5 segundos antes de reintentar
      delay(5000);
    }
    retries--;
    if (retries == 0) {
      // esperar a que el WDT lo reinicie
      while (1);
}}}

//------------------------SETUP-----------------------------
void setup() {
// Inicia Serial
Serial.begin(115200);
Serial.println("");
//prepara GPI13 y 12 como outputs <=> subscribe topics
pinMode(blanco, OUTPUT); // D7 salida digital RELE
digitalWrite(blanco, LOW); // 
pinMode(amarillo, OUTPUT); // D6 salida digital RELE
digitalWrite(amarillo, LOW);
// inputs
pinMode(openClose, INPUT); // D4
// Conexión WIFI
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, password);
while (WiFi.status() != WL_CONNECTED and contconexion <50) { 
  //Cuenta hasta 50 si no se puede conectar lo cancela
  ++contconexion;
  delay(500);
  Serial.print(".");
}
client.setServer(SERVER, SERVERPORT);
client.setCallback(callback);

String htlg = "/" + USERNAME + "/" + "HTLG"; 
htlg.toCharArray(HTLG, 50);
String nivel_h2o = "/" + USERNAME + "/" + "NIVEL_H2O"; 
nivel_h2o.toCharArray(NIVEL_H2O, 50);
String iluminacion = "/" + USERNAME + "/" + "LUZAB"; 
iluminacion.toCharArray(LUZAB, 50);
String salidaAnalogica = "/" + USERNAME + "/" + "SALIDAANALOGICA"; 
salidaAnalogica.toCharArray(SALIDAANALOGICA, 50);
//-------- sensor HT Adafruit -----------
if (! sht4.begin()){
  Serial.println("Couldn't find SHT4x");
  while (1) delay(1);
}
sht4.setPrecision(SHT4X_HIGH_PRECISION);
sht4.setHeater(SHT4X_LOW_HEATER_1S);
Ro = MQCalibration(sensorgas);
}

//--------------------------LOOP--------------------------------
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
    
  if (currentMillis - previousMillis >= 60000) { //envia la htlg cada 60 segundos
    previousMillis = currentMillis;
    //----------------------------------------------sensor Humedad Temp Adafruit
    sensors_event_t humidity, tempi;
    sht4.getEvent(&humidity, &tempi);// populate temp and humidity objects with fresh data
    
    //----------------------------------------------sensor GAS MQ135
    humi = humidity.relative_humidity;
    temp = tempi.temperature;
    lume = 100.0;
    gass = MQGetGasPercentage(MQRead(sensorgas)/Ro,GAS_NH4);
    //----------------------------------------------Control Bombillo vs Temp
    if(temp >= 35.0 ){
       digitalWrite(amarillo, LOW);delay(300);
       digitalWrite(blanco, HIGH);
      strLUMI = 'B';
      strLUMI.toCharArray(valueStr,5);
      client.publish(LUZAB, valueStr);
    }else{
       digitalWrite(amarillo, HIGH);delay(300);
      digitalWrite(blanco, LOW);
      strLUMI = 'A';
      strLUMI.toCharArray(valueStr,5);
      client.publish(LUZAB, valueStr);
    }
    delay(300);
    //---------------------------------------------- Envio de Datos HTLG MQTT
    
    strHTLG= String(humi, 2)+"/"+String(temp, 2)+"/"+String(lume, 2)+"/"+String(gass, 2); //2 decimales
    strHTLG.toCharArray(valueStr, 50);
    Serial.println("Enviando: [" +  String(HTLG) + "] " + strHTLG);
    client.publish(HTLG, valueStr);
  }
  //----------------------------------------------lectura de boton
  if (digitalRead(openClose) == 0) {
    strNivel_h2o = "presionado";
  } else {
    strNivel_h2o = "NO presionado";
  }
//envia el estado del nivel_h2o solamente cuando cambia.
  if (strNivel_h2o != strNivel_h2oUltimo) { 
    strNivel_h2oUltimo = strNivel_h2o;
    strNivel_h2o.toCharArray(valueStr, 15);
    Serial.println("Enviando: [" +  String(NIVEL_H2O) + "] " + strNivel_h2o);
    client.publish(NIVEL_H2O, valueStr);
  }
}
  
