#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <ESP32Servo.h>

Servo Clapeta_camera_1,Clapeta_camera_2, Clapeta_camera_3; // Obiecte pentru clapetele camerelor

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

void setup() { 
    Serial.begin((96000));
    Clapeta_camera_1.attach(CAMERA1_CLAPETA);
    Clapeta_camera_2.attach(CAMERA2_CLAPETA);
    Clapeta_camera_3.attach(CAMERA3_CLAPETA);
}

void loop() {
}
