#include <Wire.h>
#include <SoftwareWire.h>
#include <Adafruit_AHTX0.h>
#include <DHT20.h>

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

// Obiecte Adafruit (hardware I2C)
Adafruit_AHTX0 aht1;
Adafruit_AHTX0 aht2;

SoftwareWire sw3(CAMERA3_SDA_PIN, CAMERA3_SCL_PIN); // SDA, SCL pentru Camera 3
SoftwareWire sw4(CAM_TEHNICA_SDA_PIN, CAM_TEHNICA_SCL_PIN);   // SDA, SCL pentru Camera Tehnică

DHT20 aht3(&sw3);  //  2nd I2C interface
DHT20 ahtTech(&sw4); //  2nd I2C interface pentru Camera Tehnică

void setup() {
    Serial.begin(115200);

    // Inițializare Camera 1
    Wire.begin(CAMERA1_SDA_PIN, CAMERA1_SCL_PIN);
    if (!aht1.begin(&Wire)) Serial.println("Eroare la AHT20 Camera 1!");

    // Inițializare Camera 2
    Wire.begin(CAMERA2_SDA_PIN, CAMERA2_SCL_PIN);
    if (!aht2.begin(&Wire)) Serial.println("Eroare la AHT20 Camera 2!");

    // Inițializare Camera 3 (SoftwareWire)
    sw3.begin();
    if (!aht3.begin()) Serial.println("Eroare la AHT20 Camera 3!");

    // Inițializare Camera Tehnică (SoftwareWire)
    sw4.begin();
    if (!ahtTech.begin()) Serial.println("Eroare la AHT20 Camera Tehnică!");
}

void loop() {
    sensors_event_t temp, humidity;

    // Camera 1
    aht1.getEvent(&humidity, &temp);
    Serial.print("Camera 1 - Temp: "); Serial.print(temp.temperature);
    Serial.print(" C, Umiditate: "); Serial.println(humidity.relative_humidity);

    // Camera 2
    aht2.getEvent(&humidity, &temp);
    Serial.print("Camera 2 - Temp: "); Serial.print(temp.temperature);
    Serial.print(" C, Umiditate: "); Serial.println(humidity.relative_humidity);

    // Camera 3 (SoftwareWire)
    float t3 = aht3.getTemperature();
    float h3 = aht3.getHumidity();
    Serial.print("Camera 3 - Temp: "); Serial.print(t3);
    Serial.print(" C, Umiditate: "); Serial.println(h3);

    // Camera Tehnică (SoftwareWire)
    float t4 = ahtTech.getTemperature();
    float h4 = ahtTech.getHumidity();
    Serial.print("Camera Tehnică - Temp: "); Serial.print(t4);
    Serial.print(" C, Umiditate: "); Serial.println(h4);

    Serial.println("------------------------------------------");
    delay(5000);
}
