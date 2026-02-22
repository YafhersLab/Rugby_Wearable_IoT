//===================== LIBRERIAS =========================//

// Conexion GPRS y MQTT
#define TINY_GSM_MODEM_A7670
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// Lectura BNO055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <esp_log.h>

// Lectura GPS
#include <TinyGPSPlus.h>

// Visualización SSD1306
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <utility/imumaths.h>

//===================== DEFINICIONES =========================//

// Pines AT670 
#define MODEM_RESET_PIN   5
#define MODEM_PWKEY       4
#define MODEM_POWER_ON    12
#define MODEM_TX          26
#define MODEM_RX          27
#define MODEM_BAUD        115200
#define SerialGsm         Serial1

// Definiciones SSD1306
#define SCREEN_WIDTH      128 
#define SCREEN_HEIGHT     64  
#define SCREEN_ADDRESS    0x3C
#define OLED_RESET        -1

// Definiciones GPS NEO M8N
#define NEO_M8N_TX        23
#define NEO_M8N_RX        19

//===================== VARIABLES ===========================//

// Datos APN de Bitel
const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Parametros del broker MQTT
const char* broker = "broker.hivemq.com"; 
const int   port   = 1883;                
const char* topic_sub = "test_SIM/sub";
const char* topic_pub = "test_SIM/pub";

// Estado de conexión MQTT
bool mqttConnected = false;
unsigned long lastMQTTSent = 0;

// Variables de lectura BNO055
double bno055_euler_x = 0.0;
double bno055_euler_y = 0.0;
double bno055_euler_z = 0.0;

// Variables de lectura GPS NEO-M8N
double neom8n_lat = 0.0;
double neom8n_lng = 0.0;

//======================= OBJETOS ===========================//

// Objeto TinyGSM
TinyGsm modem(SerialGsm);
TinyGsmClient gsmClient(modem);

// Objeto cliente MQTT
PubSubClient mqtt(gsmClient);

// Objeto SSD1306
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Objeto BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

// Objeto GPS NEO M8N
TinyGPSPlus gps;

//===================== PROTOTIPOS =========================//

// Prototipos de Tasks
static void readAngles(void* pvParameters);
static void readGPS(void* pvParameters);
static void refreshSDD1306(void* pvParameters);
static void sendMQTT(void* pvParameters);

// Prototipos de Funciones
static void MQTT_Callback(char* topic, byte* payload, unsigned int length);
static bool MQTT_Connect(void);

//===================== MAIN CODE ===========================//

// Void Setup
void setup() 
{
  // Inicialización serial
  Serial.begin(115200);
  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  while(!Serial);

  // Inicialización BNO055
  Wire.begin();
  if(!bno.begin())
  {
    printf("[ERROR] No se ha encontrado el BNO055\r\n"); 
    while(1);
  }
  bno.setExtCrystalUse(true);

  // Inicialización SSD1306 
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    printf("[ERROR] No se ha encontrado el SSD1306\r\n"); while(1);
  } 
  display.clearDisplay();

  // Título grande
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(15, 5);
  display.println("MGC-Sport");
  display.display();

  display.clearDisplay();
  // Barra de carga animada
  for (int i = 0; i <= 20; i += 5) 
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(13, 5);
    display.println("MGC-Sport");
    display.setTextSize(1);
    display.setCursor(10, 30);
    display.println("Iniciando Sistema..");
    display.fillRect(10, 50, i, 10, WHITE);
    display.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Inicialización GPS NEO M8N
  Serial2.begin(9600, SERIAL_8N1, NEO_M8N_RX, NEO_M8N_TX);

  // Alimentar modem
  pinMode(MODEM_POWER_ON, OUTPUT);  
  digitalWrite(MODEM_POWER_ON, 1);

  // Reiniciar modem
  pinMode(MODEM_RESET_PIN, OUTPUT); 
  digitalWrite(MODEM_RESET_PIN, 0); delay(100);
  digitalWrite(MODEM_RESET_PIN, 1); delay(1000);
  digitalWrite(MODEM_RESET_PIN, 0);

  // Pulso de reinicio en PWRKEY
  pinMode(MODEM_PWKEY, OUTPUT);
  digitalWrite(MODEM_PWKEY, 0); delay(100);
  digitalWrite(MODEM_PWKEY, 1); delay(1000);
  digitalWrite(MODEM_PWKEY, 0);

  // Inicializar modem
  Serial.println("Inicializando modem ... 👀");
  SerialGsm.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Obtener información del módem
  String name = modem.getModemName(); delay(500);
  String modemInfo = modem.getModemInfo();

  // Imprimir información del modem
  Serial.println("================================");
  Serial.println("Información básica del módem 🌟");
  Serial.print("   Name: "); Serial.println(name);
  Serial.print("   Info: "); Serial.println(modemInfo);
  Serial.println("================================");

  display.clearDisplay();
  // Barra de carga animada
  for (int i = 20; i <= 40; i += 5) 
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(15, 5);
    display.println("MGC-Sport");
    display.setTextSize(1);
    display.setCursor(10, 30);
    display.println("Modem configurado..");
    display.fillRect(10, 50, i, 10, WHITE);
    display.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
  // Conectarse a la red
  Serial.println("Esperando a la red...");
  if (!modem.waitForNetwork())
  {
    Serial.println("Error: no se pudo conectar a la red ❌");
    delay(10000);
    return;
  }

  // Verificar si esta conectado a la red
  if (modem.isNetworkConnected())
  {
    Serial.println("Red disponible, conectado correctamente ✔");
  }

  // Obtener mas informacion del modem
  String ccid = modem.getSimCCID();           delay(500);
  String imei = modem.getIMEI();              delay(500);
  String operatorName = modem.getOperator();  delay(500);
  int csq = modem.getSignalQuality();         delay(500);

  // Imprimir información del modem
  Serial.println("Información del módem: 📡");
  Serial.print("   CCID (SIM): "); Serial.println(ccid);
  Serial.print("   IMEI: "); Serial.println(imei);
  Serial.print("   Operador: "); Serial.println(operatorName);
  Serial.print("   Calidad de señal (0-31): "); Serial.println(csq);
  Serial.println("================================");

  // Barra de carga animada
  display.clearDisplay();
  for (int i = 40; i <= 60; i += 5) 
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(15, 5);
    display.println("MGC-Sport");
    display.setTextSize(1);
    display.setCursor(10, 30);
    display.println("Red SIM obtenida..");
    display.fillRect(10, 50, i, 10, WHITE);
    display.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Realizar conexión GPRS
  Serial.print("Esperando conexión al APN (");
  Serial.print(apn);
  Serial.println(")...");
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) 
  {
    Serial.println("Error: no se pudo conectar al APN ❌");
  }
  delay(500);

  // Verificar si esta conectado al APN
  if (modem.isGprsConnected()) 
  {
    Serial.println("APN disponible, conectado correctamente ✔");
  }

  // Obtener IP local
  IPAddress local = modem.localIP();
  Serial.print("  IP obtenida: ");
  Serial.println(local);
  Serial.println("================================");
  delay(500);

  // Barra de carga animada
  display.clearDisplay();
  for (int i = 60; i <= 80; i += 5) 
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(15, 5);
    display.println("MGC-Sport");
    display.setTextSize(1);
    display.setCursor(10, 30);
    display.println("Conexion GPRS OK..");
    display.fillRect(10, 50, i, 10, WHITE);
    display.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Configuración conexión MQTT
  mqtt.setServer(broker, port);
  mqtt.setCallback(MQTT_Callback);

  // Realizar conexión MQTT
  if (!MQTT_Connect()) 
  {
    Serial.println("No se pudo conectar al broker MQTT 😢");
  }

  // Barra de carga animada
  display.clearDisplay();
  for (int i = 80; i <= 100; i += 5) 
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(15, 5);
    display.println("MGC-Sport");
    display.setTextSize(1);
    display.setCursor(10, 30);
    display.println("Conexion MQTT OK..");
    display.fillRect(10, 50, i, 10, WHITE);
    display.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Indicar que todo salio bien
  printf("Sistema iniciado correctamente ✔\r\n");
  vTaskDelay(pdMS_TO_TICKS(2000));

  // Configuración de Tasks
  xTaskCreate(readAngles, "Read Angles", 4096, NULL, 3, NULL);
  xTaskCreate(readGPS, "Read GPS", 4096, NULL, 3, NULL);
  xTaskCreate(refreshSDD1306, "Refresh SSD1306", 4096, NULL, 2, NULL);
  xTaskCreate(sendMQTT, "Send MQTT", 4096, NULL, 2, NULL);

  // Elimino el loop
  vTaskDelete(NULL);
}

//===================== FUNCIONES =========================//

// Función: Conexión MQTT
static bool MQTT_Connect(void) 
{
  Serial.print("Conectando al broker MQTT...");

  // Conectarse con el ID A7670Client
  if (mqtt.connect("A7670Client")) 
  {
    Serial.println(" conectado ✔");

    // Suscribirse a un topic
    mqtt.subscribe(topic_sub);
    Serial.print("Suscrito a: "); Serial.println(topic_sub);

    return true;
  } 
  else 
  {
    Serial.print(" fallo, rc=");
    Serial.println(mqtt.state());
    return false;
  }
}

// Función: Callback MQTT
static void MQTT_Callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("📩 Mensaje en [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) 
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

//====================== TASKS ============================//

// Task: Lectura de angulos del BNO055
void readAngles(void* pvParameters)
{
  // Variables locales
  static int failCount = 0;

  while(1)
  {
    // Obtener datos del BNO055
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    bno055_euler_x = euler.x();
    bno055_euler_y = euler.y();
    bno055_euler_z = euler.z();

    // Verificar si todo está en cero
    if (bno055_euler_x == 0.0 && bno055_euler_y == 0.0 && bno055_euler_z == 0.0)
    {
      failCount++;

      if (failCount > 5) 
      {
        printf("[CRITICO] BNO055 colgado, forzando reinicio I2C...\r\n");
        vTaskDelay(pdMS_TO_TICKS(50));

        if (bno.begin()) 
        {
          bno.setExtCrystalUse(true);
          printf("[OK] BNO055 recuperado tras reset de I2C\r\n");
          failCount = 0;
        } 
        else 
        {
          printf("[ERROR] No responde tras reset I2C\r\n");
        }
      }
    }
    else 
    {
      failCount = 0; 
    }

    // Impresion de datos
    //printf("[BNO055] X: %.2f, Y: %.2f, Z: %.2f\r\n", bno055_euler_x, bno055_euler_y, bno055_euler_z);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task: Lectura de posición del GPS NEO M8N
void readGPS(void* pvParameters)
{
  // Bucle
  while(1)
  {
    while(Serial2.available() > 0)
    {
      gps.encode(Serial2.read());
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (gps.location.isUpdated() && gps.location.isValid())
    {
      neom8n_lat = gps.location.lat();
      neom8n_lng = gps.location.lng();

      // Imprimir por Serial
      // printf("[GPS] Lat: %.6f, Lng: %.6f\r\n", neom8n_lat, neom8n_lng);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task: Actualización de pantalla OLED
void refreshSDD1306(void* pvParameters)
{
  while(1)
  {
    display.clearDisplay();

    // Encabezado
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(30, 0);
    display.println(" MGC-Sport ");
    display.drawLine(0, 10, SCREEN_WIDTH, 10, WHITE);

    // Datos GPS (Lat/Lng)
    display.setTextSize(1);
    display.setCursor(30, 15);
    display.print("Lat:");
    display.setCursor(60, 15);
    display.print(neom8n_lat, 4);

    display.setCursor(30, 25);
    display.print("Lng:");
    display.setCursor(60, 25);
    display.print(neom8n_lng, 4);

    display.drawLine(0, 35, SCREEN_WIDTH, 35, WHITE);

    // Datos BNO055 (X,Y,Z en fila)
    display.setCursor(0, 40);
    display.print("X:");
    display.setCursor(15, 40);
    display.print(bno055_euler_x, 0);

    display.setCursor(45, 40);
    display.print("Y:");
    display.setCursor(60, 40);
    display.print(bno055_euler_y, 0);

    display.setCursor(90, 40);
    display.print("Z:");
    display.setCursor(105, 40);
    display.print(bno055_euler_z, 0);

    // Línea inferior: mostrar estado MQTT
    display.drawLine(0, 55, SCREEN_WIDTH, 55, WHITE);
    display.setCursor(1, 57);
    display.print("MQTT: ");
    display.print(mqttConnected ? "OK" : "FAIL");

    // Mostrar último envío en segundos
    if (lastMQTTSent > 0)
    {
        display.setCursor(78, 57);
        display.print("PUB: ");
        if (lastMQTTSent <= 9999) display.print(lastMQTTSent);
        else lastMQTTSent = 0;
    }

    // Refrescar pantalla
    display.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task: Envio de datos a servidor MQTT
static void sendMQTT(void* pvParameters)
{
  while(1)
  {
    // Reconexión MQTT en caso de desconexión
    if (!mqtt.connected()) 
    {
      MQTT_Connect();
    }
    else
    {
      mqttConnected = true;
    }

    // Loop MQTT
    mqtt.loop();

    // Publicar datos al TOPIC cada 5 segundos
    static unsigned long lastSend = 0;
    unsigned long now = millis();
    if (now - lastSend > 5000) 
    {
      lastSend = now;
      lastMQTTSent++;

      // Armar trama de datos
      char payload[128];
      snprintf(payload, sizeof(payload), "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"lat\":%.6f,\"lng\":%.6f}", bno055_euler_x, bno055_euler_y, bno055_euler_z, neom8n_lat, neom8n_lng);

      // Publicar datos MQTT
      mqtt.publish(topic_pub, payload);
      Serial.println("Datos enviados al TOPIC 👍");
    }

    vTaskDelay(100);
  }
}

// Void Loop (sin usar)
void loop()
{
}