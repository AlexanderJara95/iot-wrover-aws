#define MQTT_MAX_PACKET_SIZE 102400  // Aumentamos el tamaño máximo del paquete a 100 KB
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

const int bufferSize = 1024 * 100;  // Aumentamos el tamaño del buffer a 100 KB
#define TRIG_PIN  12  // TRIG
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

bool sensorActive = false; // Variable para controlar el estado del sensor

// Prototipos de funciones
void messageReceived(char* topic, byte* payload, unsigned int length);

void cameraInit() {
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

  config.frame_size = FRAMESIZE_VGA; // Cambia esto si necesitas una resolución más alta
  config.pixel_format = PIXFORMAT_JPEG; // Asegúrate de que esté en JPEG
  config.jpeg_quality = 12;  // Puedes ajustar este valor si necesitas reducir el tamaño del archivo
  config.fb_count = 2;

  // Inicialización de la cámara
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error al inicializar la cámara 0x%x", err);
    ESP.restart();
    return;
  }
}

void mqttInit() {
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);
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

void grabImage() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (fb == NULL) {
    Serial.println("Error al obtener la imagen de la cámara.");
    return;
  }

  if (fb->format != PIXFORMAT_JPEG) {
    Serial.println("Formato de imagen no soportado.");
    esp_camera_fb_return(fb);
    return;
  }

  // Mostrar el tamaño de la imagen capturada
  Serial.print("Tamaño de imagen capturada: ");
  Serial.println(fb->len);

  if (fb->len < bufferSize) {
    Serial.print("Publicando imagen: ");
    bool result = client.publish(ESP32WROVER_PUBLISH_TOPIC, (const char*)fb->buf, fb->len);
    Serial.println(result ? "Éxito" : "Error en publicación");
  } else {
    Serial.println("El tamaño de la imagen excede el buffer.");
  }

  esp_camera_fb_return(fb);
  delay(1);
}

float getDistance() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (float)duration / 29.1 / 2;

  return distance;
}

void connectAWS() {
  Serial.print("Conectando a AWS IoT...");
  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("¡AWS IoT conectado!");
  client.subscribe(LED_SUBSCRIBE_TOPIC);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect(THINGNAME)) {
      Serial.println("Conectado");
      client.subscribe(LED_SUBSCRIBE_TOPIC);
    } else {
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void messageReceived(char* topic, byte* payload, unsigned int length) {
  if (String(topic) == LED_SUBSCRIBE_TOPIC) {
    payload[length] = '\0'; // Asegura que el mensaje sea una cadena terminada en nulo
    int ledStatus = atoi((char*)payload);
    setLED(ledStatus);
  }
}

void setLED(int status) {
  digitalWrite(ledRojo, LOW);
  digitalWrite(ledVerde, LOW);
  digitalWrite(ledAmarillo, LOW);

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
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ledRojo, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAmarillo, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  cameraInit();
  mqttInit();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("sensor")) {
      sensorActive = true;
      Serial.println("Sensor activado.");
      grabImage();
      sensorActive = false;
    }
  }

  if (sensorActive) {
    float distance = getDistance();
    Serial.print("Distancia: ");
    Serial.println(distance);
    if (distance < MAX_DISTANCE) {
      Serial.println("Objeto cercano, tomando imagen...");
      grabImage();
      sensorActive = false;
    }
    delay(1000);
  }
}