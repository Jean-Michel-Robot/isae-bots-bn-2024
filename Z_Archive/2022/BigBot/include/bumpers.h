#ifndef BUMPER_H
#define BUMPER_H

class Bumper {

    private:

    public:

        void setup();
        void loop();

        int pin_read;
        int state;
    
    Bumper(int pin_read);
    void update_state();

};

#endif