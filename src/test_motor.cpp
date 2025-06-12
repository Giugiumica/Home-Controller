#include <Servo.h>

// Definirea pinilor
// Camera 1
#define SERVO1 4 //pinul D32/GPIO32
#define camera1close85 85 // unghiul la care clapeta de la camera 1 este închisă
#define camera1open0 0 // unghiul la care clapeta de la camera 1 este deschisă

Servo ClapetaCamera1

void setare_unghi_clapeta(String INPUT){
    if (INPUT.substring(0,7) == "clapeta") {
        if (INPUT.substring(7,8).toInt() == 1) { // Verifică dacă este clapeta 1
            if (INPUT.substring(8,9) == "-") {
                if (isdigit(INPUT[10])) {
                    int unghi = INPUT.substring(10,12).toInt(); // Extrage unghiul
                    if (unghi <= camera1close85 && unghi >= camera1open0) {
                        ClapetaCamera1.write(unghi); // Setează unghiul clapetei
                        Serial.print("Clapeta de la camera 1 este setată la unghiul: ");
                        Serial.println(unghi);
                    } else {
                        Serial.println("Unghi invalid pentru clapeta 1!");
                    }
                } else if (INPUT.substring(10) == "open") {
                    ClapetaCamera1.write(camera1open0); // Deschide clapeta
                    Serial.println("Clapeta de la camera 1 este complet deschisă!");
                } else if (INPUT.substring(10) == "close") {
                    ClapetaCamera1.write(camera1close85); // Închide clapeta
                    Serial.println("Clapeta de la camera 1 este complet închisă!");
                } else {
                    Serial.println("Unghiul pentru clapeta 1 nu este valid!");
                }
            } else {
                Serial.println("Comandă necunoscută pentru clapeta 1!");
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    ClapetaCamera1.attach(SERVO1);
}

void loop() {
    if (Serial.available() > 0) { // Verifică dacă există date disponibile
        String input = Serial.readString(); // Citește datele ca un string
        setare_unghi_clapeta(input); // Apelează funcția pentru a seta unghiul clapetei
    }
    delay(500);
}