

#include "Motor.h"
#include "mbed.h"

Motor::Motor(PinName pwm, PinName fwd):
        _pwm(pwm), _fwd(fwd) {

    // Set initial condition of PWM
    _pwm.period(0.00001);
    _pwm = 0;

    // Initial condition of output enables
    _fwd = 0;
}

void Motor::speed(float speed) {
    if(speed < 0){
        // printf("Going forward");
        _fwd = true;
        }
    else{
        // printf("Going backward");
        _fwd = false;
        }
    _pwm.write(abs(speed));



}



