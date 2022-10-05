/*
 * Materia sensores y actuadores
 * Grupo 6
 * Alumno : Mario Gonzalez
 * Ejercicio 1-e
//  */
// //#include <>
// #include <BMP280_DEV.h>
// #include <Wire.h>
// #include <WiFi.h>
// #include <PubSubClient.h>

// BMP280_DEV bmp280; // creo el objeto

// // double PresionNivelMar = 1013.25; // presion a nivel de mar en milibar

// // const char *ssid = "TP-LINK_B33E";                     // red de wifi a la que me conecto
// // const char *password = "50868155";                     // password de la red de wifi
// // const char *mqtt_server = "mgalarmasserver1.ddns.net"; // dns del server mosquitto (MQTT)
// // unsigned int mqtt_port = 1883;                         // socket port del server MQTT Mosquitto
// // const char *Topico = "/grupo6/altura/";                // topico para publicar los datos en el server
// float temperature, pressure, altitude;

// // WiFiClient esp32_Client;           // creacion de objeto wifi client
// // PubSubClient client(esp32_Client); // creacion de objeto pubsunclient



// void setup()
// {
//     pinMode(LED_BUILTIN, OUTPUT);
//     Serial.begin(9600);         // inicializo serial a 9600
//     //Serial.print("Hola Mundo");
// //    Wire.begin();               // inicializo i2c
//     // WiFi.begin(ssid, password); // conecto al wifi del lugar (micasa)
//     bmp280.begin(BMP280_I2C_ALT_ADDR);
//     bmp280.setTimeStandby(TIME_STANDBY_2000MS);  

//     // while (WiFi.status() != WL_CONNECTED)
//     // {                // inicio conexion
//     //     delay(1500); // demora para volver a intentar la conexion
//     // }
//     // client.setServer(mqtt_server, mqtt_port); // estableco conexion al server mwtt
//     // client.setCallback(callback);             // inicio el callback de server mqtt y espero datos
// }

// void loop()
// {
//         digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//         delay(1000); 
//             Serial.print("Hola Mundo");                      // wait for a second
//         digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//         delay(1000);                       // wait for a second
//         // if (!client.connected()) // si la conexion esta negada reconecto
//         //     reconnect();         // si entra al condicional no hay conexion = reconectar
//         // client.loop();           // conexion al server ok
//         if (bmp280.getMeasurements(temperature, pressure, altitude))// recopilo datos del sensor
//         {   Serial.print(temperature);
//             Serial.print(F("*C   "));
//             Serial.print(pressure);
//             Serial.print(F("hPa   "));
//             Serial.print(altitude);
//             Serial.println(F("m"));
//         }           
//         //delay(5000);             // demoro para volver a medir
// }

// // void Medir_altura()
// // {
// //     char status;
// //     double T, P, A;
// //     status = bmp180.startTemperature(); // Inicio de lectura de temperatura
// //     if (status != 0)
// //     {
// //         delay(status);                     // Pausa para que finalice la lectura
// //         status = bmp180.getTemperature(T); // cargo temperatura en la variable
// //         if (status != 0)
// //         {
// //             status = bmp180.startPressure(3); // Inicio lectura de presión
// //             if (status != 0)
// //             {
// //                 delay(status);                     // Pausa para que finalice la lectura
// //                 status = bmp180.getPressure(P, T); // cargo la presion en la variable
// //                 if (status != 0)                   // calculo la altitud segun datasheet del sensor
// //                 {
// //                     A = bmp180.altitude(P, PresionNivelMar);
// //                     char mqtt_payload[10] = "";    // genero el arreglo para alojar los datos
// //                     snprintf(mqtt_payload, 10, A); // armo el payload a enviar
// //                     client.publish(Topico, mqtt_payload);
// //                 }
// //             }
// //         }
// //     }
// // }

// // void reconnect()
// // {
// //     while (!client.connected())
// //     {
// //         delay(5000); // demora para reintentar conectarse
// //     }
// // }
// // void callback(char *topic, byte *payload, unsigned int length)
// // {
// //     // aca va el codigo del callback para hacer operaciones remotas
// //     // en este caso no se usa
// // }

#include <Wire.h>
// #include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // usa el i2c por defecto del device


void setup() {
  Serial.begin(9600);
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

  /* configurar de fabrica segun datasheet ADAFRUIT Library */
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                   Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                   Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                   Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                   Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

   pinMode(LED_BUILTIN, OUTPUT);               

   //bmp_temp->printSensorDetails();
}

void loop() {
    Serial.print("Temperatura = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    Serial.print("Presion = ");
    Serial.print(bmp.readPressure()/100);
    Serial.println(" hPa");
    Serial.print("Altitud = ");
    Serial.print(bmp.readAltitude(1013.25));
    Serial.println(" m");
    Serial.println();
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(100);  
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(3000);

}