/*
 * Materia actuadores y sensores
 * Grupo 6
 * Alumno : Mario Gonzalez
 * Ejercicio 1-F
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

const char *ssid = "TP-LINK_B33E";                     // red de wifi a la que me conecto
const char *password = "50868155";                     // password de la red de wifi
const char *mqtt_server = "mgalarmasserver1.ddns.net"; // dns del server mosquitto (MQTT)
unsigned int mqtt_port = 1883;                         // socket port del server MQTT Mosquitto
const char *Topico = "/grupo6/gps/";                   // topico para publicar los datos en el server
const unsigned int writeInterval = 25000;              // tiempod e demora para publicar
static const int RXPin = 16, TXPin = 17;               // pines del serial por soft

WiFiClient esp32_Client;           // creacion de objeto wifi client
PubSubClient client(esp32_Client); // creacion de objeto pubsunclient
TinyGPSPlus gps;                   // creacion de objeto del gps
SoftwareSerial ss(RXPin, TXPin);   // declaracion de los pines del software serial

void setup()
{
    Serial.begin(9600);         // puerto serial nativo 9600
    ss.begin(9600);             // puerto virtual serial 9600
    WiFi.begin(ssid, password); // conecto al wifi del lugar (micasa)

    while (WiFi.status() != WL_CONNECTED)
    {                // inicio conexion
        delay(1500); // demora para volver a intentar la conexion
    }
    client.setServer(mqtt_server, mqtt_port); // estableco conexion al server mwtt
    client.setCallback(callback);             // inicio el callback de server mqtt y espero datos
}

void loop()
{
    if (!client.connected()) // si la conexion esta negada reconecto
        reconnect();
    client.loop();
    while (ss.available() > 0)     // verifico si llego datos por el serial por soft
        if (gps.encode(ss.read())) // si llego leo el dato
            displayInfo();
}

void displayInfo()
{ // rutina de captura del dato por GPS
    if (gps.location.isValid())
    {                                                                  // verifico que el dato sea valido del objeto gps
        double latitude = (gps.location.lat());                        // desdoblo la info a cada variable y desprecio velocidad
        double longitude = (gps.location.lng());                       // desdoblo la info a cada variable y desprecio velocidad
        char mqtt_payload[50] = "";                                    // genero el arreglo para alojar los datos
        snprintf(mqtt_payload, 50, "m1=%lf;%lf", latitude, longitude); // armo el payload a enviar
        client.publish(Topico, mqtt_payload);                          // publico en el broker el dato del gps
        delay(writeInterval);                                          // espero este tiempo para hacer otra publicacion
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    // aca va el codigo del callback para hacer operaciones remotas
    // en este caso no se usa
}

void reconnect()
{
    while (!client.connected())
    {
        delay(5000); // demora para reintentar conectarse
    }
}
}