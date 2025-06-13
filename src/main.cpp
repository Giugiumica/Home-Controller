#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// Definire pini pentru Camera 1
#define CAMERA1_SCL_PIN 26
#define CAMERA1_SDA_PIN 27
#define CAMERA1_CLAPETA 32
#define CAMERA1_HEATER 33
#define CAMERA1_ATOMIZOR 25

// Definire pini pentru Camera 2
#define CAMERA2_SCL_PIN 22
#define CAMERA2_SDA_PIN 21
#define CAMERA2_CLAPETA 14
#define CAMERA2_HEATER 13
#define CAMERA2_ATOMIZOR 23


// Definire pini pentru Camera 3
#define CAMERA3_SCL_PIN 16
#define CAMERA3_SDA_PIN 4
#define CAMERA3_CLAPETA 19
#define CAMERA3_HEATER 18
#define CAMERA3_ATOMIZOR 17

// Definire pini pentru Camera Tehnică
#define CAM_TEHNICA_SCL_PIN 2
#define CAM_TEHNICA_SDA_PIN 15
#define VENTILATOR 5

// Inițializare obiecte pentru senzori
Adafruit_AHTX0 aht1, aht2, aht3, ahtTech;
Adafruit_BMP280 bmp1, bmp2, bmp3, bmpTech;

void setup() {
  Serial.begin(96000);

  // Inițializare I2C pentru fiecare cameră
  Wire.begin(CAMERA1_SDA_PIN, CAMERA1_SCL_PIN);
  if (!aht1.begin()) Serial.println("Eroare la senzorul AHT20 Camera 1!");
  if (!bmp1.begin(0x76)) Serial.println("Eroare la senzorul BMP280 Camera 1!");

  Wire.begin(CAMERA2_SDA_PIN, CAMERA2_SCL_PIN);
  if (!aht2.begin()) Serial.println("Eroare la senzorul AHT20 Camera 2!");
  if (!bmp2.begin(0x76)) Serial.println("Eroare la senzorul BMP280 Camera 2!");

  Wire.begin(CAMERA3_SDA_PIN, CAMERA3_SCL_PIN);
  if (!aht3.begin()) Serial.println("Eroare la senzorul AHT20 Camera 3!");
  if (!bmp3.begin(0x76)) Serial.println("Eroare la senzorul BMP280 Camera 3!");

  Wire.begin(CAM_TEHNICA_SDA_PIN, CAM_TEHNICA_SCL_PIN);
  if (!ahtTech.begin()) Serial.println("Eroare la senzorul AHT20 Camera Tehnică!");
  if (!bmpTech.begin(0x76)) Serial.println("Eroare la senzorul BMP280 Camera Tehnică!");
}

void loop() {
  // Citire temperatură și umiditate de la AHT20
  sensors_event_t temp1, hum1, temp2, hum2, temp3, hum3, tempTech, humTech;
  aht1.getEvent(&hum1, &temp1);
  aht2.getEvent(&hum2, &temp2);
  aht3.getEvent(&hum3, &temp3);
  ahtTech.getEvent(&humTech, &tempTech);

  // Citire temperatură și presiune de la BMP280
  float bmpTemp1 = bmp1.readTemperature();
  float bmpTemp2 = bmp2.readTemperature();
  float bmpTemp3 = bmp3.readTemperature();

  float bmpPressure1 = bmp1.readPressure();
  float bmpPressure2 = bmp2.readPressure();
  float bmpPressure3 = bmp3.readPressure();

  // Afișare valori
  Serial.print("\nCamera 1 - Temp(AHT20): ");
  Serial.print(temp1.temperature);
  Serial.print(" °C, Umiditate: ");
  Serial.print(hum1.relative_humidity);
  Serial.println(" %, Temp(BMP280): ");

  Serial.print("Camera 2 - Temp(AHT20): ");
  Serial.print(temp2.temperature);
  Serial.print(" °C, Umiditate: ");
  Serial.print(hum2.relative_humidity);
  Serial.println(" %, Temp(BMP280): ");

  Serial.print("Camera 3 - Temp(AHT20): ");
  Serial.print(temp3.temperature);
  Serial.print(" °C, Umiditate: ");
  Serial.print(hum3.relative_humidity);
  Serial.print(" %, Temp(BMP280): ");
  Serial.println(bmpTemp3);

  Serial.print("Camera Tehnică - Temp(AHT20): ");
  Serial.print(tempTech.temperature);
  Serial.print(" °C, Umiditate: ");
  Serial.print(humTech.relative_humidity);
  Serial.println(" %, Temp(BMP280): ");
  Serial.println("-------------------------------------------");

  delay(5000); // Pauză de 2 secunde între măsurători
}