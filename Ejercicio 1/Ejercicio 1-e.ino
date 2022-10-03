/*
 * Materia sensores y actuadores
 * Grupo 6
 * Alumno : Mario Gonzalez
 * Ejercicio 1-e
 */
#include <SFE_BMP180.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

SFE_BMP180 bmp180; // creo el objeto

double PresionNivelMar = 1013.25; // presion a nivel de mar en milibar

const char *ssid = "TP-LINK_B33E";                     // red de wifi a la que me conecto
const char *password = "50868155";                     // password de la red de wifi
const char *mqtt_server = "mgalarmasserver1.ddns.net"; // dns del server mosquitto (MQTT)
unsigned int mqtt_port = 1883;                         // socket port del server MQTT Mosquitto
const char *Topico = "/grupo6/altura/";                // topico para publicar los datos en el server

WiFiClient esp32_Client;           // creacion de objeto wifi client
PubSubClient client(esp32_Client); // creacion de objeto pubsunclient

void setup()
{
    Serial.begin(9600);         // inicializo serial a 9600
    Wire.begin();               // inicializo i2c
    WiFi.begin(ssid, password); // conecto al wifi del lugar (micasa)
    bmp180.begin();             // inicializo sensor bmp180

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
        reconnect();         // si entra al condicional no hay conexion = reconectar
    client.loop();           // conexion al server ok
    Medir_altura();          // Mido altura
    delay(5000);             // demoro para volver a medir
}

void Medir_altura()
{
    char status;
    double T, P, A;
    status = bmp180.startTemperature(); // Inicio de lectura de temperatura
    if (status != 0)
    {
        delay(status);                     // Pausa para que finalice la lectura
        status = bmp180.getTemperature(T); // cargo temperatura en la variable
        if (status != 0)
        {
            status = bmp180.startPressure(3); // Inicio lectura de presión
            if (status != 0)
            {
                delay(status);                     // Pausa para que finalice la lectura
                status = bmp180.getPressure(P, T); // cargo la presion en la variable
                if (status != 0)                   // calculo la altitud segun datasheet del sensor
                {
                    A = bmp180.altitude(P, PresionNivelMar);
                    char mqtt_payload[10] = "";    // genero el arreglo para alojar los datos
                    snprintf(mqtt_payload, 10, A); // armo el payload a enviar
                    client.publish(Topico, mqtt_payload);
                }
            }
        }
    }
}

void reconnect()
{
    while (!client.connected())
    {
        delay(5000); // demora para reintentar conectarse
    }
}
void callback(char *topic, byte *payload, unsigned int length)
{
    // aca va el codigo del callback para hacer operaciones remotas
    // en este caso no se usa
}
