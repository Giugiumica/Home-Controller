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
constexpr uint8_t TCA_ADDR = 0x70;   // adresa multiplexorului
constexpr uint8_t NUM_NODES = 4;
Adafruit_AHTX0 aht[NUM_NODES];
bool aht_is_present[NUM_NODES] = {false, false, false, false};

#define NR_CAMERE         3
#define TEMP_TOLERANTA    1.0     // °C
#define UMID_TOLERANTA    3.0     // %

#define HEAT_AND_HUMI_PWM_FREQ 10 // Frecvența PWM pentru încălzire și umidificare
#define PWM_RESOLUTION 8 // Rezoluția PWM pentru atomizoare

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

struct CameraData {
  bool incalzire_activ;
  bool umidificare_activ;
  float temperatura_setata;
  uint8_t umiditate_setata;
};
CameraData camere[3];

struct CameraReadout {
  float temperatura;
  float temperatura_eroare_anterioara;
  float temperatura_integrala;
  uint8_t umiditate;
  uint8_t umiditate_eroare_anterioara;
  uint8_t umiditate_integrala;
};
CameraReadout camere_actuale[4];

char message_length[20];
char ultimul_mesaj[20] = "";
bool mesaj_nou = false;

constexpr uint8_t CLAPETA_PINS[3]    = {32, 33, 25};
constexpr uint8_t CLAPETA_CHANNELS[3] = {4, 5, 6};  // LEDC channels
constexpr uint8_t PWM_RESOLUTION_clapeta     = 16;          // 2^16 = 65536
constexpr uint32_t PWM_FREQUENCY     = 50;          // 50 Hz = 20 ms period

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
    esp_err_t err = esp_now_send(receiverMac, reinterpret_cast<const uint8_t*>(message_length),n);
    if (err != ESP_OK) Serial.printf("❌ Eroare ESP-NOW (%d)\n", err);
}

void decodare_date_primite() {
    uint8_t cam = 0;
    bool incalzire_activ = false;
    bool umidificare_activ = false;
    float temp_setp = 0.0;
    uint8_t rh_setp = 0;
    sscanf(ultimul_mesaj, "cam%hhu-%d-%d-%f-%hhu", &cam,&incalzire_activ,&umidificare_activ, &temp_setp, &rh_setp);
    camere[cam-1].incalzire_activ = incalzire_activ;
    camere[cam-1].umidificare_activ = umidificare_activ;
    camere[cam-1].temperatura_setata = temp_setp;
    camere[cam-1].umiditate_setata = rh_setp;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memset(ultimul_mesaj, 0, sizeof(ultimul_mesaj));
  memcpy(ultimul_mesaj, incomingData, std::min((size_t)len, sizeof(ultimul_mesaj) - 1));
  mesaj_nou = true;
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

uint32_t pulseToDuty(uint32_t pulse_us) {
  // duty = pulse * freq * 2^res / 1_000_000
  return (pulse_us * PWM_FREQUENCY * (1UL << PWM_RESOLUTION)) / 1000000UL;
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
            if (unghi>= CLAPETA_CAMERA_1_OPEN && unghi <= CLAPETA_CAMERA_1_CLOSED) {
                Clapeta_camera_1.write(unghi);
            } else if (unghi< CLAPETA_CAMERA_1_OPEN) {
                Clapeta_camera_1.write(CLAPETA_CAMERA_1_OPEN);
            } else if (unghi > CLAPETA_CAMERA_1_CLOSED) {
                Clapeta_camera_1.write(CLAPETA_CAMERA_1_CLOSED);
            } else{Serial.println("Eroare necuoscuta la setarea unghiuli pentru camera 1");
            }
            
            break;
        case 2:
            if (unghi>= CLAPETA_CAMERA_2_OPEN && unghi <= CLAPETA_CAMERA_2_CLOSED) {
                Clapeta_camera_2.write(unghi);
            } else if (unghi< CLAPETA_CAMERA_2_OPEN) {
                Clapeta_camera_2.write(CLAPETA_CAMERA_2_OPEN);
            } else if (unghi > CLAPETA_CAMERA_2_CLOSED) {
                Clapeta_camera_2.write(CLAPETA_CAMERA_2_CLOSED);
            } else{Serial.println("Eroare necuoscuta la setarea unghiuli pentru camera 2");
            }
            break;
        case 3:
            if (unghi>= CLAPETA_CAMERA_3_CLOSED && unghi <= CLAPETA_CAMERA_3_OPEN) {
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

void setare_atomizor(uint8_t cameraNR, bool pornit_oprit) {
    if (cameraNR==1){
        digitalWrite(CAMERA1_ATOMIZOR, pornit_oprit ? HIGH : LOW);
    } else if (cameraNR==2){
        digitalWrite(CAMERA2_ATOMIZOR, pornit_oprit ? HIGH : LOW);
    } else if (cameraNR==3){
        digitalWrite(CAMERA3_ATOMIZOR, pornit_oprit ? HIGH : LOW);
    } else {
        Serial.println("Numărul camerei nu este valid pentru atomizor.");
    }
}

void setare_heater(uint8_t cameraNR, uint8_t procentPWM) {
  static const uint8_t pwmChannel[3] = { 4, 5, 6 };
  static const uint8_t heaterPin[3]  = { CAMERA1_HEATER, CAMERA2_HEATER, CAMERA3_HEATER };
  static bool initDone[3] = { false, false, false };

  uint8_t idx = cameraNR - 1;
  uint8_t ch  = pwmChannel[idx];
  uint8_t pin = heaterPin[idx];

  if (!initDone[idx]) {
    ledcSetup(ch, HEAT_AND_HUMI_PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(pin, ch);
    initDone[idx] = true;
  }
  procentPWM = constrain(procentPWM, 0, 100);
  uint32_t duty = map(procentPWM, 0, 100, 0, (1 << PWM_RESOLUTION) - 1);
  ledcWrite(ch, duty);
}

void setare_viteza_ventilator(uint8_t dutyCycleProcent) {
  const int pwmChannel = 0;
  static bool pwm_initializat = false;
  if (!pwm_initializat) {
    ledcSetup(pwmChannel, FAN_PWM_FREQ, 8);
    ledcAttachPin(VENTILATOR_PIN, pwmChannel);
    pwm_initializat = true;
  }
  uint8_t dutyPWM = map(dutyCycleProcent, 0, 100, 0, 255);
  ledcWrite(pwmChannel, dutyPWM); // Setează viteza ventilatorului
}

void handle_heating_state(){
    for (uint8_t i = 0; i < NR_CAMERE; i++) {
        if (camere[i].incalzire_activ) {
        if (camere_actuale[i+1].temperatura < camere[i].temperatura_setata- TEMP_TOLERANTA) {
            setare_heater(i + 1, 100); // Setează încălzitorul la 100%
        } else if (camere_actuale[i+1].temperatura > camere[i].temperatura_setata + TEMP_TOLERANTA) {
            setare_heater(i + 1, 0); // Oprește încălzitorul
        }
        } else {
        setare_heater(i + 1, 0); // Oprește încălzitorul dacă nu este activ
        }
    }
}

void handle_humi_state(){
    for (uint8_t i = 0; i < NR_CAMERE; i++) {
        if (camere[i].umidificare_activ) {
        if (camere_actuale[i+1].umiditate < camere[i].umiditate_setata - UMID_TOLERANTA) {
            setare_atomizor(i + 1, true); // Setează atomizorul la 100%
        } else if (camere_actuale[i+1].umiditate > camere[i].umiditate_setata + UMID_TOLERANTA) {
            setare_atomizor(i + 1, false); // Oprește atomizorul
        }
        } else {
        setare_atomizor(i + 1, false); // Oprește atomizorul dacă nu este activ
        }
    }
}

void handle_clapeta_and_fan_state(){
    uint8_t contIncalzire = 0,contUmidificare = 0,contGeneral=0;
    for (uint8_t i = 0; i < NR_CAMERE; i++) {
        if (camere[i].incalzire_activ) {
            contIncalzire+=1;
            if (camere_actuale[i+1].umiditate < camere[i].umiditate_setata || camere_actuale[i+1].temperatura < camere[i].temperatura_setata) {
                contGeneral+=1;
            }else contGeneral=0;    
        }else if (camere[i].umidificare_activ) {
            contUmidificare+=1;
        }    
    }
    if(contGeneral > 0) {
    if (contIncalzire == 3) {
        unghi_clapeta(0, CLAPETE_DESCHISE); // Deschide toate clapetele
        setare_viteza_ventilator(FAN_MAX_DUTY); // Setează ventilatorul la viteză maximă
    } else if (contIncalzire == 2) {
        //codul pentru a deschide clapetele corespunzătoare
        if(camere[0].incalzire_activ) {
            unghi_clapeta(1, CLAPETA_CAMERA_1_OPEN);
        } else if(camere[1].incalzire_activ) {
            unghi_clapeta(2, CLAPETA_CAMERA_2_OPEN);
        } else unghi_clapeta(3, CLAPETA_CAMERA_3_OPEN);
        if (contUmidificare==3){
        setare_viteza_ventilator(90); // Setează ventilatorul la viteză mică
        }else setare_viteza_ventilator(80); // Setează ventilatorul la viteză mică

    } else if (contIncalzire == 1) {
        //codul pentru a deschide clapeta corespunzătoare
        if(camere[0].incalzire_activ) {
            unghi_clapeta(1, CLAPETA_CAMERA_1_OPEN);
        } else if(camere[1].incalzire_activ) {
            unghi_clapeta(2, CLAPETA_CAMERA_2_OPEN);
        } else unghi_clapeta(3, CLAPETA_CAMERA_3_OPEN);
        if (contUmidificare==2){
        setare_viteza_ventilator(70); // Setează ventilatorul la viteză mică
        }else setare_viteza_ventilator(60); // Setează ventilatorul la viteză mică

    } else if(contIncalzire == 0 && contUmidificare == 0) {
        unghi_clapeta(0, CLAPETE_INCHISE); // Închide toate clapetele
        setare_viteza_ventilator(FAN_STOP); // Oprește ventilatorul
    }else if (contIncalzire == 0 && contUmidificare != 0) {
        //codul pentru a deschide clapeta corespunzătoare
        if(contUmidificare == 3) {
            unghi_clapeta(0, CLAPETE_DESCHISE); // Deschide toate clapetele
            setare_viteza_ventilator(90); // Setează ventilatorul la viteză maximă
        } else if (contUmidificare == 2) {
            if(camere[0].umidificare_activ) {
                unghi_clapeta(1, CLAPETA_CAMERA_1_OPEN);
            } else if(camere[1].umidificare_activ) {
                unghi_clapeta(2, CLAPETA_CAMERA_2_OPEN);
            } else unghi_clapeta(3, CLAPETA_CAMERA_3_OPEN);
            setare_viteza_ventilator(70); // Setează ventilatorul la viteză medie
        } else if (contUmidificare == 1) {
            if(camere[0].umidificare_activ) {
                unghi_clapeta(1, CLAPETA_CAMERA_1_OPEN);
            } else if(camere[1].umidificare_activ) {
                unghi_clapeta(2, CLAPETA_CAMERA_2_OPEN);
            } else unghi_clapeta(3, CLAPETA_CAMERA_3_OPEN);
            setare_viteza_ventilator(50); // Setează ventilatorul la viteză mică
        }
    }
}else {
    unghi_clapeta(0, CLAPETE_INCHISE); // Închide toate clapetele
    setare_viteza_ventilator(FAN_STOP); // Oprește ventilatorul
  }
}

void control_sistem() {
    static unsigned long lastRun = 0;
    unsigned long now = millis();
    if (now - lastRun < 1000UL)
        return;
    lastRun = now;

    handle_clapeta_and_fan_state();//done
    handle_heating_state();
    handle_humi_state();
}

void setup() {
  Serial.begin(115200);
  Clapeta_camera_1.attach(CAMERA1_CLAPETA);
  Clapeta_camera_2.attach(CAMERA2_CLAPETA);
  Clapeta_camera_3.attach(CAMERA3_CLAPETA);
  setare_viteza_ventilator(FAN_STOP); // Inițializare ventilator cu PWM 0
  unghi_clapeta(0, CLAPETE_DESCHISE); // Inițializare clapete închise
  pinMode(CAMERA1_ATOMIZOR, OUTPUT);
  pinMode(CAMERA2_ATOMIZOR, OUTPUT);
  pinMode(CAMERA3_ATOMIZOR, OUTPUT);
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
  if (esp_now_init() != ESP_OK){
    return;
}
  esp_now_register_recv_cb(OnDataRecv);
  if (!esp_now_is_peer_exist(receiverMac)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("❌ Eroare la adăugarea peer-ului");
      return;
    }
  }
}

void loop() {
    handle_temp();

    if (mesaj_nou) {
    mesaj_nou = false;
    decodare_date_primite();
    }
    control_sistem();
    delay(10);
}
