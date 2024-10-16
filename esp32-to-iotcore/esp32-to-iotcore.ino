#define MQTT_MAX_PACKET_SIZE 23552  // Ajusta este valor según sea necesario
#include <PubSubClient.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include "WiFi.h"
#include "esp_camera.h"

#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  21
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    19
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    5
#define Y2_GPIO_NUM    4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

const int bufferSize = 1024 * 23; // 23552 bytes
#define TRIG_PIN  15  // TRIG
#define ECHO_PIN  14  // ECHO
const int ledRojo = 12; // Cambia por el pin real usado para el LED rojo
const int ledVerde = 13; // Cambia por el pin real usado para el LED verde
const int ledAmarillo = 2; // Cambia por el pin real usado para el LED amarillo
#define MAX_DISTANCE 20  // Distancia máxima en cm para considerar un objeto "cerca"

// Configuración del MQTT
#define ESP32WROVER_PUBLISH_TOPIC "esp32/wrover_0"
#define LED_SUBSCRIBE_TOPIC "esp32/led_status"

// Instancia del cliente WiFi y MQTT
WiFiClientSecure net;
PubSubClient client(net);

// Prototipos de funciones
void messageReceived(char* topic, byte* payload, unsigned int length);

void cameraInit(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA; // 640x480
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
    return;
  }
}
void mqttInit() {
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setBufferSize(bufferSize);
  client.setCallback(messageReceived);
  
  // Conectar al servidor MQTT
  connectAWS();
}
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';  // Asegura que el mensaje sea una cadena terminada en nulo
  String message = String((char*) payload);
  int ledStatus = message.toInt();  // Convierte el mensaje en un entero
  setLED(ledStatus);
}

void grabImage(){
  camera_fb_t * fb = esp_camera_fb_get();
  if(fb != NULL && fb->format == PIXFORMAT_JPEG && fb->len < bufferSize){
    Serial.print("Image Length: ");
    Serial.print(fb->len);
    Serial.print("\t Publish Image: ");
    bool result = client.publish(ESP32WROVER_PUBLISH_TOPIC, (const char*)fb->buf, fb->len);
    Serial.println(result);

    if(!result){
      ESP.restart();
    }
  }
  esp_camera_fb_return(fb);
  delay(1);
}


float getDistance() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Tiempo en el que el pulso está en HIGH. Esto es el tiempo que tarda en ir y volver
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calcula la distancia en centímetros
  float distance = (float)duration / 29.1 / 2;

  return distance;
}

void connectAWS() {
  Serial.print("Conectando a AWS IoT...");
  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("AWS IoT conectado!");
  client.subscribe(LED_SUBSCRIBE_TOPIC);
}

void reconnect() {
  // Ciclo hasta que estemos reconectados
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect(THINGNAME)) {
      Serial.println("conectado");
      client.subscribe(LED_SUBSCRIBE_TOPIC);
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void messageReceived(char* topic, byte* payload, unsigned int length) {
  // Asegúrate de que el mensaje es para el topic de LED
  if (String(topic) == LED_SUBSCRIBE_TOPIC) {
    payload[length] = '\0'; // Asegura que el mensaje sea una cadena terminada en nulo
    int ledStatus = atoi((char*)payload); // Convierte el mensaje en un entero
    setLED(ledStatus);
  }
}

void setLED(int status) {
  // Apaga todos los LEDs primero
  digitalWrite(ledRojo, LOW);
  digitalWrite(ledVerde, LOW);
  digitalWrite(ledAmarillo, LOW);

  // Enciende el LED basado en el valor de 'status'
  switch(status) {
    case 0:
      digitalWrite(ledRojo, HIGH);
      break;
    case 1:
      digitalWrite(ledVerde, HIGH);
      break;
    case 2:
      digitalWrite(ledAmarillo, HIGH);
      break;
  }
}

void setup() {
  // Inicializar Serial, WiFi, cámara y MQTT
  Serial.begin(115200);
  // Inicialización para el sensor HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  // Configurar pines de LED como salidas
  pinMode(ledRojo, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAmarillo, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  cameraInit();
  mqttInit();
  
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  float distance = getDistance();
  if (distance < MAX_DISTANCE) {
    Serial.println("DISTANCE");
    Serial.println(distance);
    if(client.connected()) grabImage();
    delay(500000);  // Para evitar enviar imágenes constantemente, espera 5 segundos antes de la próxima medición
  }
  
}