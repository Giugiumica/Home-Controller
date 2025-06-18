#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
//adresa mac a acestui esp 14:33:5C:02:88:20
//adresa mac a esp ului pentru UI D4:8A:FC:A4:89:90
uint8_t receiverMac[] = {0xD4, 0x8A, 0xFC, 0xA4, 0x89, 0x90}; // Adresa MAC a receptorului

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

//pini pentru clapete
#define CAMERA1_CLAPETA 32 // Pinul pentru clapeta camerei 1
#define CAMERA2_CLAPETA 33 // Pinul pentru clapeta camerei 2
#define CAMERA3_CLAPETA 25 // Pinul pentru clapeta camerei 3
//pini pentru atomizoare
#define CAMERA1_ATOMIZOR 26 // Pinul pentru atomizorul camerei 1
#define CAMERA2_ATOMIZOR 27 // Pinul pentru atomizorul camerei 2
#define CAMERA3_ATOMIZOR 14 // Pinul pentru atomizorul camerei 3
// Definire pini pentru heatere
#define CAMERA1_HEATER 21 // Pinul pentru încălzitorul camerei 1
#define CAMERA2_HEATER 19 // Pinul pentru încălzitorul camerei 2
#define CAMERA3_HEATER 18 // Pinul pentru încălzitorul camerei 3
// Definire pin pwm ventilator
#define VENTILATOR_PIN 13

uint8_t default_clapeta_state[] = {CLAPETA_CAMERA_1_CLOSED, CLAPETA_CAMERA_2_CLOSED, CLAPETA_CAMERA_3_CLOSED}; // Starea clapetei pentru fiecare cameră
bool aht_is_present[NUM_NODES] = {false, false, false, false};

struct CameraData {
  bool incalzire_activ;
  bool umidificare_activ;
  float temperatura_setata;
  uint8_t umiditate_setata;
};
CameraData camere[3];

struct CameraReadout {
  float temperatura;
  uint8_t umiditate;
};
CameraReadout camere_actuale[4]; // index 0 = cam1, etc.

char message_length[20];

void tcaSelect(uint8_t ch){
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);                 // activează exact canalul „ch”
  Wire.endTransmission();
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

void trimite_mesaj_la_ecran(uint8_t cameraNR, float temp_actual, uint8_t humi_actual) {
    int n = snprintf(message_length, sizeof(message_length),"cam%u-%.1f-%u",cameraNR+1,temp_actual,humi_actual);
    //Serial.print("Mesaj generat: ");
    //Serial.println(message_length); // să vezi exact ce se trimite
    esp_err_t err = esp_now_send(receiverMac, reinterpret_cast<const uint8_t*>(message_length),n);
    if (err != ESP_OK) Serial.printf("❌ Eroare ESP-NOW (%d)\n", err);
}

void handle_temp(){
for (uint8_t ch = 0; ch < NUM_NODES; ch++) {
  if (!aht_is_present[ch]) continue;
  tcaSelect(ch + 1);
  sensors_event_t humi, temp;
  if (aht[ch].getEvent(&humi, &temp)) {
    camere_actuale[ch].temperatura = temp.temperature;
    camere_actuale[ch].umiditate = humi.relative_humidity;
    //Serial.printf("cam%u  |  AHT20 %.1f°C %.0f%%  \n", ch + 1, camere_actuale[ch].temperatura, camere_actuale[ch].umiditate);
    trimite_mesaj_la_ecran(ch,camere_actuale[ch].temperatura,camere_actuale[ch].umiditate); // 👈 aici îl trimiți
  } else {
    Serial.printf("Eroare la citire senzor cam%u\n", ch + 1);
  }
}
}

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

void setare_viteza_ventilator(uint8_t dutyCycleProcent) {
  const int pwmChannel = 0;
  static bool pwm_initializat = false;
  if (!pwm_initializat) {
    ledcSetup(pwmChannel, FAN_PWM_FREQ, 8);
    ledcAttachPin(VENTILATOR_PIN, pwmChannel);
    pwm_initializat = true;
  }
  if (dutyCycleProcent > FAN_MAX_DUTY) dutyCycleProcent = 100;
  else if (dutyCycleProcent < FAN_MIN_DUTY) dutyCycleProcent = FAN_STOP;
  uint8_t dutyPWM = map(dutyCycleProcent, 0, 100, 0, 255);
  ledcWrite(pwmChannel, dutyPWM);
}

void setup() {
  Serial.begin(115200);
  Clapeta_camera_1.attach(CAMERA1_CLAPETA);
  Clapeta_camera_2.attach(CAMERA2_CLAPETA);
  Clapeta_camera_3.attach(CAMERA3_CLAPETA);
  setare_viteza_ventilator(FAN_STOP); // Inițializare ventilator cu PWM 0
  Clapeta_camera_1.write(CLAPETA_CAMERA_1_CLOSED);
  Clapeta_camera_3.write(CLAPETA_CAMERA_3_CLOSED);
  Clapeta_camera_2.write(CLAPETA_CAMERA_2_CLOSED);
  Wire.begin(22, 23); // SDA, SCL
  for (uint8_t ch = 0; ch < NUM_NODES; ch++) {
    tcaSelect(ch + 1);
    if (aht[ch].begin()) {
      aht_is_present[ch] = true;
    } else {
      Serial.printf("cam%u  AHT20 missing ❌\n", ch + 1);
    }
  }
  WiFi.mode(WIFI_STA);
  int canal_wifi = 6;
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(canal_wifi, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Eroare la inițializarea ESP-NOW");
    return;
  }
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(receiverMac)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("❌ Eroare la adăugarea peer-ului");
      return;
    }
  }
}

void loop() {
    handle_temp();
    delay(1000);
}
