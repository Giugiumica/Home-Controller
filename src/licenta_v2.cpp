#include <Wire.h>
#include <Servo.h>

// Definirea pinilor
// Camera 1
#define SERVO1 4
#define camera1close85 85 // unghiul la care clapeta de la camera 1 este închisă
#define camera1open0 0 // unghiul la care clapeta de la camera 1 este deschisă
#define HEATER1 5
#define ATOMIZER1 18
#define SCL1 22
#define SDA1 21
// Camera 2
#define SERVO2 19
#define camera2close90 90 // unghiul la care clapeta de la camera 2 și 3 este închisă
#define camera2open0 0 // unghiul la care clapeta de la camera 2 și 3 este deschisă 
#define HEATER2 23
#define ATOMIZER2 13
#define SCL2 25
#define SDA2 26
// Camera 3
#define SERVO3 27
#define camera3close0 0 // unghiul la care clapeta de la camera 2 și 3 este închisă
#define camera3open95 95 // unghiul la care clapeta de la camera 2 și 3 este deschisă
#define HEATER3 14
#define ATOMIZER3 12
#define SCL3 32
#define SDA3 33



// Obiecte Servo
Servo ClapetaCamera1, ClapetaCamera2, ClapetaCamera3;

void setup() {
    Serial.begin(115200);
    // Inițializare I2C
    Wire.begin(SDA1, SCL1);
    Wire.begin(SDA2, SCL2);
    Wire.begin(SDA3, SCL3);

    // Inițializare PWM
    pinMode(HEATER1, OUTPUT);
    pinMode(ATOMIZER1, OUTPUT);
    pinMode(HEATER2, OUTPUT);
    pinMode(ATOMIZER2, OUTPUT);
    pinMode(HEATER3, OUTPUT);
    pinMode(ATOMIZER3, OUTPUT);

    // Atașare servomotoare
    ClapetaCamera1.attach(SERVO1);
    ClapetaCamera2.attach(SERVO2);
    ClapetaCamera3.attach(SERVO3);
}
void setare_unghi_clapeta(String INPUT) {
    if (INPUT.substring(0,7) == "clapeta") {
        switch (INPUT.substring(7,8).toInt()) { // Extrage numărul clapetei
            case 1:
                if (INPUT.substring(8,9) == "-") {
                    if(isdigit(INPUT[10])) {
                        int unghi = INPUT.substring(10,12).toInt(); // Extrage unghiul
                        if (unghi <= camera1close85 && unghi >= camera1open0) {
                            ClapetaCamera1.write(unghi); // Setează unghiul clapetei
                            Serial.print("Clapeta de la camera");
                            Serial.print(INPUT[10]);
                            Serial.print(" este setată la unghiul: ");
                            Serial.println(unghi);
                        } else {
                            Serial.println("Unghi invalid pentru clapeta 1!");
                        }
                    }else if (INPUT.substring(10) == "open") {
                        ClapetaCamera1.write(camera1open0); // Deschide clapeta
                        Serial.println("Clapeta de la camera ");
                        Serial.print(INPUT[10]);
                        Serial.println(" este complet deschisă!");
                    } else if (INPUT.substring(10) == "close") {
                        ClapetaCamera1.write(camera1close85); // Închide clapeta
                        Serial.print("Clapeta de la camera ");
                        Serial.print(INPUT[10]);
                        Serial.println(" este cumplet închisă!");
                    }else{
                        Serial.println("Unghiul pentru clapeta 1 nu este valid!");
                    }
                }else {
                Serial.println("Comandă necunoscută pentru clapeta 1!");
            }
            break;

            case 2:
                if (INPUT.substring(8,9) == "-") {
                    if(isdigit(INPUT[10])) {
                        int unghi = INPUT.substring(10,12).toInt(); // Extrage unghiul
                        if (unghi <= camera2close90 && unghi >= camera2open0) {
                            ClapetaCamera2.write(unghi); // Setează unghiul clapetei
                            Serial.print("Clapeta de la camera");
                            Serial.print(INPUT[10]);
                            Serial.print(" este setată la unghiul: ");
                            Serial.println(unghi);
                        } else {
                            Serial.println("Unghi invalid pentru clapeta 2!");
                        }
                    }else if (INPUT.substring(10) == "open") {
                        ClapetaCamera2.write(camera2open0); // Deschide clapeta
                        Serial.println("Clapeta de la camera ");
                        Serial.print(INPUT[10]);
                        Serial.println(" este complet deschisă!");
                    } else if (INPUT.substring(10) == "close") {
                        ClapetaCamera2.write(camera2close90); // Închide clapeta
                        Serial.print("Clapeta de la camera ");
                        Serial.print(INPUT[10]);
                        Serial.println(" este cumplet închisă!");
                    }else{
                        Serial.println("Unghiul pentru clapeta 2 nu este valid!");
                    }
            break;
            
            case 3:
                if (INPUT.substring(8,9) == "-") {
                    if(isdigit(INPUT[10])) {
                        int unghi = INPUT.substring(10,12).toInt(); // Extrage unghiul
                        if (unghi >= camera3close0 && unghi <= camera3open95) {
                            ClapetaCamera3.write(unghi); // Setează unghiul clapetei
                            Serial.print("Clapeta de la camera");
                            Serial.print(INPUT[10]);
                            Serial.print(" este setată la unghiul: ");
                            Serial.println(unghi);
                        } else {
                            Serial.println("Unghi invalid pentru clapeta 3!");
                        }
                    }else if (INPUT.substring(10) == "open") {
                        ClapetaCamera3.write(camera3open95); // Deschide clapeta
                        Serial.println("Clapeta de la camera ");
                        Serial.print(INPUT[10]);
                        Serial.println(" este complet deschisă!");
                    } else if (INPUT.substring(10) == "close") {
                        ClapetaCamera3.write(camera3close0); // Închide clapeta
                        Serial.print("Clapeta de la camera ");
                        Serial.print(INPUT[10]);
                        Serial.println(" este cumplet închisă!");
                    }else{
                        Serial.println("Unghiul pentru clapeta 3 nu este valid!");
                    }
                }else {
                    Serial.println("Comandă necunoscută pentru clapeta 3!");
                }

            default: 
            Serial.println("Comandă gresita, camera introdusa nu exista !!!!!!");
            break;
        }
    }
}

void loop() {
    if (Serial.available() > 0) { // Verifică dacă există date disponibile
        String input = Serial.readString(); // Citește datele ca un string

    }

    delay(1000);
}