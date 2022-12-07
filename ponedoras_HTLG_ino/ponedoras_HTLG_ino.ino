#include <ESP8266WiFi.h>
#include <PubSubClient.h>
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

char charNivel_h2o [15];
char PLACA[50];
char valueStr[50];
char HTLG[50]; //Humedad-Temperatura-Luminosidad-gases
char NIVEL_H2O[50];//sensor infrarojo de nivel de agua
char BOMBA_H2O[50];
char SALIDAANALOGICA[50];

const char *ssid     = "Altavista";
const char *password = "29702587";
char   SERVER[50]    = "108.129.17.116";
int    SERVERPORT    = 17403;
String USERNAME      = "ponedoras";   
char   PASSWORD[50]  = "piqira21"; 
//-------------------------------------------------------------------------
WiFiClient espClient;
PubSubClient client(espClient);
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
  if (String(topic) ==  String(BOMBA_H2O)) {
    if (payload[1] == 'N'){
      digitalWrite(12, HIGH);
    }
    if (payload[1] == 'F'){
      digitalWrite(12, LOW);
    }
  }
  if (String(topic) ==  String(SALIDAANALOGICA)) {
    analogWrite(13, String(PAYLOAD).toInt());
}}

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
      client.subscribe(BOMBA_H2O);
      client.subscribe(SALIDAANALOGICA);
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
pinMode(13, OUTPUT); // D7 salida analogic
analogWrite(13, 0); // analogWrite(pin, value);
pinMode(12, OUTPUT); // D6 salida digital/Bomba H2O
digitalWrite(12, LOW);
// inputs
pinMode(4, INPUT); // D2
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
String bomba_h2o = "/" + USERNAME + "/" + "BOMBA_H2O"; 
bomba_h2o.toCharArray(BOMBA_H2O, 50);
String salidaAnalogica = "/" + USERNAME + "/" + "SALIDAANALOGICA"; 
salidaAnalogica.toCharArray(SALIDAANALOGICA, 50);
}
//--------------------------LOOP--------------------------------
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
    
  if (currentMillis - previousMillis >= 5000) { //envia la htlg cada 5 segundos
    previousMillis = currentMillis;
    analog = analogRead(17);
    temp = 66.66*analog/(1023-analog);
    humi = 4.75*(1023-analog)/(1+analog);
    lume = pow(3, 0.5 * log(analog));
    gass = analog*(5.0/1023.5);


    strHTLG= String(humi, 2)+"/"+String(temp, 2)+"/"+String(lume, 2)+"/"+String(gass, 2); //2 decimales

    strHTLG.toCharArray(valueStr, 50);
    Serial.println("Enviando: [" +  String(HTLG) + "] " + strHTLG);
    client.publish(HTLG, valueStr);
  }
  
  if (digitalRead(4) == 0) {
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
  
