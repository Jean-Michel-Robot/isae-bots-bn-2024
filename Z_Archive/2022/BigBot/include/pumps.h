#ifndef PUMP_H
#define PUMP_H

class Pump {
    
    private:

    public:
        void setup();
        void loop();

        int state_of_pump;
        int pin_command;

    Pump(int pin_command);
    void switch_pump(int futur_state);
};

#endif