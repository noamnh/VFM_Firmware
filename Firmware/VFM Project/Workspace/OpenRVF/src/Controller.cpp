#include "Controller.hpp"
#include "CAN_Communication.hpp"


Controller::Controller(
    PinName motor_pwm,
    PinName motor_direction,
    PinName vibrator_pwm,
    PinName vibrator_direction,
    PinName EncoderChannelA,
    PinName EncoderChannelB,
    PinName EncoderIndex,
    PinName IntteruptPin){


is_calibrated = false;
status = CONTROLLER_STATUS::IDLE;



pid = std::make_shared<PID>(&control_parameters_.desired_position_pulses,
                            &control_parameters_.actual_position_pulses,
                            &control_parameters_.pidout,
                            control_parameters_.output_lower,
                            control_parameters_.output_upper,
                            control_parameters_.kp,
                            control_parameters_.ki,
                            control_parameters_.kd,
                            control_parameters_.Ts);



encoder = std::make_shared<QEI>(EncoderChannelA,
                                EncoderChannelB,
                                EncoderIndex,
                                1);                              



limit_interupt = std::make_shared<InterruptIn>(IntteruptPin);



motor = std::make_shared<Motor>(motor_pwm,motor_direction);



vibrator = std::make_shared<Motor>(vibrator_pwm,vibrator_direction);

Pulses_to_speed = std::make_shared<Map>(-360,360,-1,1);

Angle_to_pulses = std::make_shared<Map>(0,360,0,2800);;


}



void Controller::update(){


    if(debug_){
                printf("(Controller) Updating\n");

    }


    update_actual_position(encoder->getPulses());
    pid->sample();
        // printf("(Controller) pid out: %f \n", control_parameters_.pidout);
                // printf("(Controller) error: %f \n", pid->getError());


    motor->speed(Pulses_to_speed->Calculate(control_parameters_.pidout));
    

}
void Controller::start(){
status = CONTROLLER_STATUS::RUN;

}

void Controller::stop_calibration(){
    printf("(Controller) Reached the limit switch\n");

    control_parameters_.desired_position_pulses =  encoder->getPulses(); // get the optic switch cut off position and make it desired
    motor->speed(0); // stop the motor
    limit_interupt->rise(NULL);
    limit_interupt->fall(NULL);
    // Now go to the desired position and do it until reached the switch cut off
    while(abs(control_parameters_.desired_position_pulses-control_parameters_.actual_position_pulses) < 7){ // TODO ADD TIMEOUT
        update();
    }
    printf("(Controller) Done calibrating\n");
    motor->speed(0);
    control_parameters_.desired_position_pulses = 0;
    encoder->reset();
    is_calibrated = true;
    status = CONTROLLER_STATUS::IDLE;

}

void Controller::calibrate(){

    // Rotate the motor till it reach the limit switch
    printf("(Controller) Start Calibrating\n");
    motor->speed(0.5);

    limit_interupt->rise(callback(this, &Controller::stop_calibration));
    limit_interupt->fall(callback(this, &Controller::stop_calibration));

    status = CONTROLLER_STATUS::CALIBRATE;
}


void Controller::stop(){
printf("(Controller) STOPPING THE CONTROLLER\n");
    motor->speed(0);
    status = CONTROLLER_STATUS::IDLE;


}

void Controller::update_desired_position(float desired_position_radians){
    control_parameters_.desired_position = desired_position_radians;
    control_parameters_.desired_position_pulses = radians_to_pulses(desired_position_radians);
}

void Controller::update_actual_position(float actual_position_pulses){
    control_parameters_.actual_position_pulses = actual_position_pulses;
    control_parameters_.actual_position = pulses_to_radians(actual_position_pulses);

}

float Controller::pulses_to_radians(float pulses){
    float degree = pulses/degree_to_pulses;
                //   printf("(Controller)degree: %f \n", degree);

    float radians = degree*PI/180;
    return radians;
}

float Controller::radians_to_pulses(float radians){

    if(debug_)
    {
        printf("(Controller) Convert radians to pulses\n");

    }

    float degree = radians*180/PI;
    float pulses = degree_to_pulses*degree;


    return pulses;
    
}

void Controller::set_controller_parameters(ControlStruct controller_parameters){
    
    if(debug_){
        printf("(Controller) Setting control parameters\n");
    }

    control_parameters_ = controller_parameters;
    pid->set_parameters(control_parameters_.kp, control_parameters_.ki, control_parameters_.kd, control_parameters_.Ts);
    pid->_output_upper = control_parameters_.output_upper;
    pid->_output_lower = control_parameters_.output_lower;

}

void Controller::set_motor_parameters(MotorParameters motor_parameters){
    if(debug_){
        printf("(Controller) Setting motor parameters\n");
    }

    motor_parameters_ = motor_parameters;
    degree_to_pulses = (motor_parameters_.gear_ratio*motor_parameters_.lines_per_rev)/360.0;
    id = motor_parameters_.id;
}

void Controller::set_debug_mode(bool mode){
    debug_ = mode;
}