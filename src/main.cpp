#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>

Servo Clapeta_camera_1,Clapeta_camera_2, Clapeta_camera_3; // Obiecte pentru clapetele camerelor

constexpr uint8_t TCA_ADDR   = 0x70;   // adresa multiplexorului
constexpr uint8_t NUM_NODES  = 4;

Adafruit_AHTX0 aht[NUM_NODES];

#define HEAT_PWM_FREQ 10 // Frecvența de citire a senzorilor în milisecunde
#define HUMI_PWM_FREQ 10 // Rezoluția PWM pentru controlul încălzirii

#define FAN_PWM_FREQ 50
#define FAN_MAX_DUTY 100 // Valoarea maximă a PWM pentru ventilator
#define FAN_MIN_DUTY 50   // Valoarea minimă a PWM pentru ventilator
#define FAN_STOP 0

#define CLAPETA_CAMERA_1_CLOSED 85 //Grade la care clapeta este închisă
#define CLAPETA_CAMERA_1_OPEN 0 // Grade la care clapeta este deschisă
#define CLAPETA_CAMERA_2_CLOSED 90 // Grade la care clapeta este închisă
#define CLAPETA_CAMERA_2_OPEN 0 // Grade la care clapeta este deschisă
#define CLAPETA_CAMERA_3_CLOSED 0 // Grade la care clapeta este închisă
#define CLAPETA_CAMERA_3_OPEN 95 // Grade la care clapeta este deschisă
#define CLAPETE_INCHISE 255
#define CLAPETE_DESCHISE 100

// Definire pini pentru Camera 1
#define CAMERA1_CLAPETA 32
#define CAMERA1_HEATER 33
#define CAMERA1_ATOMIZOR 25
// Definire pini pentru Camera 2
#define CAMERA2_CLAPETA 14
#define CAMERA2_HEATER 13
#define CAMERA2_ATOMIZOR 23
// Definire pini pentru Camera 3
#define CAMERA3_CLAPETA 19
#define CAMERA3_HEATER 18
#define CAMERA3_ATOMIZOR 17
// Definire pini pentru Camera Tehnică
#define VENTILATOR_PIN 5


bool room_heating_state[] = {false, false, false}; // Starea încălzirii pentru fiecare cameră
bool room_humi_state[] = {false, false, false}; // Starea umidificării pentru fiecare cameră
float actual_temp_set[] = {20.0, 20.0, 20.0}; // Temperatura setată pentru fiecare cameră
int actual_rh_set[] = {50, 50, 50}; // Umiditatea setată pentru fiecare cameră
int default_clapeta_state[] = {CLAPETA_CAMERA_1_CLOSED, CLAPETA_CAMERA_2_CLOSED, CLAPETA_CAMERA_3_CLOSED}; // Starea clapetei pentru fiecare cameră
char message_length[16];
uint8_t temp_actual[4]; // Temperatura actuală pentru fiecare cameră
uint8_t humi_actual[4]; // Umiditatea actuală pentru fiecare cameră
bool aht_is_present[NUM_NODES] = {false, false, false, false};

//adresa mac a acestui esp 14:33:5C:02:88:20
//adresa mac a esp ului pentru UI D4:8A:FC:A4:89:90
uint8_t receiverMac[] = {0xD4, 0x8A, 0xFC, 0xA4, 0x89, 0x90}; // Adresa MAC a receptorului

void tcaSelect(uint8_t ch){
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);                 // activează exact canalul „ch”
  Wire.endTransmission();
}

void trimite_mesaj_la_ecran(uint8_t cameraNR){
    int n = snprintf(message_length, sizeof(message_length),"cam%u-%.1f-%u",cameraNR,actual_temp_set[cameraNR - 1],actual_rh_set[cameraNR - 1]);
    esp_err_t err = esp_now_send(receiverMac, reinterpret_cast<const uint8_t*>(message_length),n);
    if (err != ESP_OK) Serial.printf("❌ Eroare ESP-NOW (%d)\n", err);
}

/*
void get_temp_humi(uint8_t cameraNR, float &temp, uint8_t &humi) {
    switch (cameraNR) {
        case 1:
            break;

        case 2:
            break;

        case 3:
            break;

        case 4:
            break;

        default:
            Serial.println("Numărul camerei nu este valid.");
    }
}
*/

void unghi_clapeta(uint8_t numarul_camerei, uint8_t unghi) {
    switch (numarul_camerei) {
        case 0:
            if(unghi==CLAPETE_DESCHISE) {
                Clapeta_camera_1.write(CLAPETA_CAMERA_1_OPEN);
                Clapeta_camera_2.write(CLAPETA_CAMERA_2_OPEN);
                Clapeta_camera_3.write(CLAPETA_CAMERA_3_OPEN);
            } else if(unghi==CLAPETE_INCHISE) {
                Clapeta_camera_1.write(CLAPETA_CAMERA_1_CLOSED);
                Clapeta_camera_2.write(CLAPETA_CAMERA_2_CLOSED);
                Clapeta_camera_3.write(CLAPETA_CAMERA_3_CLOSED);
            } else {
                Serial.println("Unghiul specificat nu este valid pentru toate camerele.");
            }
            break;
        case 1:
            if (unghi> CLAPETA_CAMERA_1_OPEN && unghi < CLAPETA_CAMERA_1_CLOSED) {
                Clapeta_camera_1.write(unghi);
            } else if (unghi< CLAPETA_CAMERA_1_OPEN) {
                Clapeta_camera_1.write(CLAPETA_CAMERA_1_OPEN);
            } else if (unghi > CLAPETA_CAMERA_1_CLOSED) {
                Clapeta_camera_1.write(CLAPETA_CAMERA_1_CLOSED);
            } else{Serial.println("Eroare necuoscuta la setarea unghiuli pentru camera 1");
            }
            
            break;
        case 2:
            if (unghi> CLAPETA_CAMERA_2_OPEN && unghi < CLAPETA_CAMERA_2_CLOSED) {
                Clapeta_camera_2.write(unghi);
            } else if (unghi< CLAPETA_CAMERA_2_OPEN) {
                Clapeta_camera_2.write(CLAPETA_CAMERA_2_OPEN);
            } else if (unghi > CLAPETA_CAMERA_2_CLOSED) {
                Clapeta_camera_2.write(CLAPETA_CAMERA_2_CLOSED);
            } else{Serial.println("Eroare necuoscuta la setarea unghiuli pentru camera 2");
            }
            break;
        case 3:
            if (unghi> CLAPETA_CAMERA_3_CLOSED && unghi < CLAPETA_CAMERA_3_OPEN) {
                Clapeta_camera_3.write(unghi);
            } else if (unghi< CLAPETA_CAMERA_3_CLOSED) {
                Clapeta_camera_3.write(CLAPETA_CAMERA_3_CLOSED);
            } else if (unghi > CLAPETA_CAMERA_3_OPEN) {
                Clapeta_camera_3.write(CLAPETA_CAMERA_3_OPEN);
            } else{Serial.println("Eroare necuoscuta la setarea unghiuli pentru camera 3");
            }
            break;
        default:
            Serial.println("Numărul camerei nu este valid.");
    }
}

void setare_viteza_ventilator(uint8_t dutyCycle) {
}

void get_temp_humi(uint8_t cameraNR, float &temp, uint8_t &humi) {
  if (cameraNR < 1 || cameraNR > NUM_NODES) {
    Serial.println("❌ Numărul camerei nu este valid.");
    return;
  }

  uint8_t index = cameraNR - 1; // indexul în array-ul de senzori
  tcaSelect(cameraNR); // canalul TCA corespunde direct numărului camerei

  sensors_event_t hum_event, temp_event;
  if (aht[index].getEvent(&hum_event, &temp_event)) {
    temp = temp_event.temperature;
    humi = (uint8_t)(hum_event.relative_humidity + 0.5); // rotunjire simplă

    Serial.printf("✅ Camera %u: %.1f °C, %u %% RH\n", cameraNR, temp, humi);
  } else {
    Serial.printf("⚠️  Eroare la citirea senzorului pentru camera %u\n", cameraNR);
    temp = -100.0;
    humi = 0;
  }
}

void scanChannel(uint8_t ch) {
  tcaSelect(ch);
  delay(100);
  Serial.printf("📡 Scan pe canalul %u: ", ch);
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);

    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", addr);
    }
  }
  //Serial.println();
}

void handle_temp(){
    for (uint8_t ch = 0; ch < NUM_NODES; ch++){
        if (!aht_is_present[ch])continue;
        tcaSelect(ch + 1);
        sensors_event_t hum, temp;
        if (aht[ch].getEvent(&hum, &temp)){
            Serial.printf("cam%u  |  AHT20 %.1f°C %.0f%%  \n", ch + 1, temp.temperature, hum.relative_humidity);
        }
        else{
            Serial.printf("cam%u  |  Eroare la citire senzor\n", ch + 1);
        }
    }
    Serial.println("-----------------------------");
}

void setup() {
    Serial.begin(115200);
    Clapeta_camera_1.attach(CAMERA1_CLAPETA);
    Clapeta_camera_2.attach(CAMERA2_CLAPETA);
    Clapeta_camera_3.attach(CAMERA3_CLAPETA);
    Wire.begin(22, 23); // SDA, SCL
    Serial.begin(115200);
    for (uint8_t ch = 0; ch < NUM_NODES; ch++){
        tcaSelect(ch + 1);
        if (aht[ch].begin()){
            aht_is_present[ch] = true;
            Serial.printf("cam%u  AHT20 OK\n", ch + 1);
        }
        else{
            Serial.printf("cam%u  AHT20 missing ❌\n", ch + 1);
        }
    }
}

void loop() {
    handle_temp();
    delay(5000);
}
