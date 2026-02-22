// Librerias
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_SSD1306.h>
#include <utility/imumaths.h>
#include <TinyGPSPlus.h>
#include "esp_log.h"

// Definiciones SSD1306
#define SCREEN_WIDTH    128 
#define SCREEN_HEIGHT   64  
#define SCREEN_ADDRESS  0x3C
#define OLED_RESET      -1

// Definiciones GPS NEO M8N
#define NEO_M8N_TX      23
#define NEO_M8N_RX      19

// Lectura de bateria 18650
#define PIN_BATTERY     35 

// Variables Globales
double bno055_euler_x = 0.0;
double bno055_euler_y = 0.0;
double bno055_euler_z = 0.0;
double neom8n_lat = 0.0;
double neom8n_lng = 0.0;

// Objeto SSD1306
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Objeto BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

// Objeto GPS NEO M8N
TinyGPSPlus gps;

// Prototipos de Tasks
void readAngles(void* pvParameters);
void readGPS(void* pvParameters);
void refreshSDD1306(void* pvParameters);

// Void Setup
void setup() 
{
  // Inicialización Serial
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
  display.println("MGC-");
  display.setCursor(25, 25);
  display.println("Sport");
  display.display();

  // Barra de carga animada
  for (int i = 0; i <= 100; i += 5) {
    display.fillRect(10, 50, i, 10, WHITE);  // Rectángulo que se va llenando
    display.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Pausa un momento con pantalla completa
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Inicialización GPS NEO M8N
  Serial2.begin(9600, SERIAL_8N1, NEO_M8N_RX, NEO_M8N_TX);

  // Indicar que todo salio bien
  printf("Sistema iniciado correctamente ✔\r\n");
  vTaskDelay(pdMS_TO_TICKS(2000));

  // Configuración de Tasks
  xTaskCreate(readAngles, "Read Angles", 4096, NULL, 3, NULL);
  xTaskCreate(readGPS, "Read GPS", 4096, NULL, 3, NULL);
  xTaskCreate(refreshSDD1306, "Refresh SSD1306", 4096, NULL, 2, NULL);
}

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
    vTaskDelay(pdMS_TO_TICKS(100));
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
      //printf("[GPS] Lat: %.6f, Lng: %.6f\r\n", neom8n_lat, neom8n_lng);
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

    // 🔹 Encabezado
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(30, 0);
    display.println(" MGC-Sport ");
    display.drawLine(0, 10, SCREEN_WIDTH, 10, WHITE);

    // 🔹 Datos GPS (Lat/Lng)
    display.setTextSize(1);
    display.setCursor(0, 15);
    display.print("Lat:");
    display.setCursor(30, 15);
    display.print(neom8n_lat, 4);

    display.setCursor(0, 25);
    display.print("Lng:");
    display.setCursor(30, 25);
    display.print(neom8n_lng, 4);

    display.drawLine(0, 35, SCREEN_WIDTH, 35, WHITE);

    // 🔹 Datos BNO055 (X,Y,Z en fila)
    display.setCursor(0, 40);
    display.print("X:");
    display.setCursor(15, 40);
    display.print(bno055_euler_x, 1);

    display.setCursor(45, 40);
    display.print("Y:");
    display.setCursor(60, 40);
    display.print(bno055_euler_y, 1);

    display.setCursor(90, 40);
    display.print("Z:");
    display.setCursor(105, 40);
    display.print(bno055_euler_z, 1);

    // 🔹 Línea inferior
    display.drawLine(0, 55, SCREEN_WIDTH, 55, WHITE);
    display.setCursor(35, 57);
    display.print("Running...");

    // Refrescar pantalla
    display.display();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// Void Loop (sin usar)
void loop()
{
  // Leer el valor del ADC
  int raw = analogRead(PIN_BATTERY);

  // Calcular voltaje de la batería
  float v_battery = raw / 377.0;

  // Mostrar resultados
  Serial.print("V_bat: ");
  Serial.print(v_battery, 3);
  Serial.println(" V");

  vTaskDelay(pdMS_TO_TICKS(1000)); 
}