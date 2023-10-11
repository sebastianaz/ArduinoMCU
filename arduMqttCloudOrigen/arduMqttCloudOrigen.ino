#include <ESP8266WiFi.h>
#include <PubSubClient.h>
//-------------------VARIABLES GLOBALES--------------------------
int contconexion = 0;
int analog;
float temp;
unsigned long previousMillis = 0;
String strPulsador;
String strPulsadorUltimo;
String strtemp = "";
char charPulsador [15];
char PLACA[50];
char valueStr[15];
char TEMPERATURA[50];
char PULSADOR[50];
char SALIDADIGITAL[50];
char SALIDAANALOGICA[50];

const char *ssid     = "Altavista";
const char *password = "29702587";
char   SERVER[50]    = "3.81.179.172";
int    SERVERPORT    = 16594;
String USERNAME      = "jamaica";   
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
  if (String(topic) ==  String(SALIDADIGITAL)) {
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
      client.subscribe(SALIDADIGITAL);
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
pinMode(12, OUTPUT); // D6 salida digital
digitalWrite(12, LOW);
// inputs
pinMode(4, INPUT); // D2


// Conexión WIFI
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, password);
while (WiFi.status() != WL_CONNECTED and contconexion <50) { //Cuenta hasta 50 si no se puede conectar lo cancela
++contconexion;
delay(500);
Serial.print(".");
}

client.setServer(SERVER, SERVERPORT);
client.setCallback(callback);

String temperatura = "/" + USERNAME + "/" + "TEMPERATURA"; 
temperatura.toCharArray(TEMPERATURA, 50);
String pulsador = "/" + USERNAME + "/" + "PULSADOR"; 
pulsador.toCharArray(PULSADOR, 50);
String salidaDigital = "/" + USERNAME + "/" + "SALIDADIGITAL"; 
salidaDigital.toCharArray(SALIDADIGITAL, 50);
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
    
  if (currentMillis - previousMillis >= 5000) { //envia la temperatura cada 5 segundos
    previousMillis = currentMillis;
    analog = analogRead(17);
    temp = analog;
    strtemp = String(temp, 1); //1 decimal
    strtemp.toCharArray(valueStr, 15);
    Serial.println("Enviando: [" +  String(TEMPERATURA) + "] " + analog);
    client.publish(TEMPERATURA, valueStr);
  }
  
  if (digitalRead(4) == 0) {
    strPulsador = "presionado";
  } else {
    strPulsador = "NO presionado";
  }
//envia el estado del pulsador solamente cuando cambia.
  if (strPulsador != strPulsadorUltimo) { 
    strPulsadorUltimo = strPulsador;
    strPulsador.toCharArray(valueStr, 15);
    Serial.println("Enviando: [" +  String(PULSADOR) + "] " + strPulsador);
    client.publish(PULSADOR, valueStr);
  }
}
  
