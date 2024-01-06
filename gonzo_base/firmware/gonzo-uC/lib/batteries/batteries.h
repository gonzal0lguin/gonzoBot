#ifndef BATTERIES_H
#define BATTERIES_H
#include <Arduino.h>


class Battery {
    public:
        Battery(int pins[3]);
        void update_cell_voltages();
        void update_cell_voltages_avrg();
        void print_statuses();
        

        static float mapf(float val, float min, float max, float a, float b);
        static float sumarr(float arr[10]);
        static float average(float arr[10]);

        const int N_CELLS = 3;
        float battery_voltage;
        float battery_percentage;
        bool present = false;
        float analog_val[3];
        float cell_voltage[3];
        float cell_percentage[3];
        int _pins[3];

        float update_rate = 0.5; 

        bool check_battery_present();

    private:
        float RC1 = 430.0 / 330.0;
        float RC2 = 770.0 / 300.0; 
        float RC3 = 205.0 / 55.0;

        float cell_voltage_av[3][10];

        uint32_t t_sample = 0;
        uint32_t sample_interval = 100; // ms
        int sample_count = 0;
};

Battery::Battery(int pins[3])
{
    for (int i= 0; i< N_CELLS; i++) {
        _pins[i] = pins[i];
        pinMode(_pins[i], INPUT);
    }
}



float Battery::mapf(float val, float min, float max, float a, float b)
{
  float scale = (b - a) * (val - min) / (max - min) + a;
  return scale;
}

void Battery::update_cell_voltages() {
    for (int i= 0; i< N_CELLS; i++) 
        analog_val[i] = (float )analogRead(_pins[i]);
    
    cell_voltage[0] = (analog_val[0] / 1023.0 * 3.3) * RC1;
    cell_voltage[1] = (analog_val[1] / 1023.0 * 3.3) * RC2 - cell_voltage[0];
    cell_voltage[2] = (analog_val[2] / 1023.0 * 3.3) * RC3 - cell_voltage[0] - cell_voltage[1];

    for (int i= 0; i< N_CELLS; i++) 
        cell_percentage[i] = mapf(cell_voltage[i], 3.0, 4.2, 0.0, 1.0) * 100.;

    battery_voltage = (analog_val[2] / 1023.0 * 3.3) * RC3;  //mapf(cell_voltage[2], 0.0, 4.2, 0.0, 12.6);
    battery_percentage = mapf(battery_voltage, 9.0, 12.3, 0.0, 1.0);

}



void Battery::update_cell_voltages_avrg() {
    if (millis() - t_sample > sample_interval) {
        //Serial.println(sample_count);
        for (int i= 0; i< N_CELLS; i++) 
            analog_val[i] = (float )analogRead(_pins[i]);
    
        cell_voltage_av[0][sample_count] = (analog_val[0] / 1023.0 * 3.3) * RC1;
        cell_voltage_av[1][sample_count] = (analog_val[1] / 1023.0 * 3.3) * RC2 - cell_voltage_av[0][sample_count];
        cell_voltage_av[2][sample_count] = (analog_val[2] / 1023.0 * 3.3) * RC3 - cell_voltage_av[0][sample_count]
                                            - cell_voltage_av[1][sample_count];

        sample_count++;
        t_sample = millis();
    }

    else {
        return;
    }

    if (sample_count == 9) {
        cell_voltage[0] = average(cell_voltage_av[0]);
        cell_voltage[1] = average(cell_voltage_av[1]);
        cell_voltage[2] = average(cell_voltage_av[2]);

        for (int i= 0; i< N_CELLS; i++) 
            cell_percentage[i] = mapf(cell_voltage[i], 3.0, 4.2, 0.0, 1.0);

        battery_voltage = mapf(cell_voltage[2], 0.0, 4.2, 0.0, 12.6);
        battery_percentage = mapf(battery_voltage, 9.0, 12.6, 0.0, 1.0);
        sample_count = 0;
    }
}

float Battery::sumarr(float arr[10]) {
    float sum = 0;
    for (int i= 0; i< 10 ; i++) 
        sum += arr[i];
    
    return sum;
}

float Battery::average(float arr[10]) {
    float av = sumarr(arr) / 10.0;
    return av;
}


#endif // BATTERIES_H