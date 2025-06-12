#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// Crearea obiectelor pentru senzori
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
#define camera1close 85 // unghiul la care clapeta de la camera 1 este închisă

void setare_unghi_clapeta(int clapetaCamera,int unghi) {
  switch (clapetaCamera) {
    case 0:
      Serial.print("Setare clapeta 1 la unghi: ");
      break;
    case 1:
      Serial.print("Setare clapeta 1 la unghi: ");
      break;
    case 2:
      Serial.print("Setare clapeta 2 la unghi: ");
      break;
    case 3:
      Serial.print("Setare clapeta 3 la unghi: ");
      break;
    default:
      Serial.println("Numarul clapetei introduse nu este valid!");
      return;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Inițializarea senzorilor
  if (!aht.begin()) {
    Serial.println("Eroare la inițializarea AHT20!");
    while (1);
  }
  Serial.println("Senzor AHT20 detectat!");

  if (!bmp.begin(0x76)) { // Adresa I2C poate fi 0x76 sau 0x77, verifică specificațiile modulului tău
    Serial.println("Eroare la inițializarea BMP280!");
    while (1);
  }
  Serial.println("Senzor BMP280 detectat!");
}

void loop() {
  // Citirea datelor de la AHT20
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  // Citirea temperaturii de la BMP280
  float bmpTemp = bmp.readTemperature();

  // Afișarea valorilor în terminal
  Serial.print("Temperatura AHT20: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.print("Umiditate AHT20: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %");

  Serial.print("Temperatura BMP280: ");
  Serial.print(bmpTemp);
  Serial.println(" °C");

  Serial.println("---------------------------");

  delay(2000); // Pauză de 2 secunde înainte de următoarea citire
}