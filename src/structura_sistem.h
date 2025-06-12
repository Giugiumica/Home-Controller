#include <Wire.h>
#include <Adafruit_AHTX0.h>
using namespace std;
Adafruit_AHTX0 senzor_camera1,senzor_camera2,senzor_camera3,senzor_camera_tehnica;

class StructuraGeneralaSistem {
public:
    StructuraGeneralaSistem() {
    }

    // Funcție de citire temperatură pentru senzorul specificat
    float get_temperature(Adafruit_AHTX0& sensor) const {
        sensors_event_t humidity, temp;
        sensor.getEvent(&humidity, &temp);
        return temp.temperature;
    }

    // Funcție de citire umiditate pentru senzorul specificat
    float get_humidity(Adafruit_AHTX0& sensor) const {
        sensors_event_t humidity, temp;
        sensor.getEvent(&humidity, &temp);
        return humidity.relative_humidity;
    }
};

class camera_tehnica : public StructuraGeneralaSistem {
    int vitezaVentilatorPrincipal;
public:
    camera_tehnica(int vitezaVentilatorPrincipal) : StructuraGeneralaSistem(), vitezaVentilatorPrincipal(vitezaVentilatorPrincipal) {} 
};