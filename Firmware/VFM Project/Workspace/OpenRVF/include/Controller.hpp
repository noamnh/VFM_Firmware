#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "mbed.h"
#include "PID.h"
#include "QEI.h"
#include "Motor.h"
#include "Map.hpp"

#define PI 3.14159265359


  typedef enum CONTROLLER_STATUS
  {
    IDLE = 0,
    RUN = 1,
    CALIBRATE = 2

  } controlelr_status_t;

    struct ControlStruct {
        float desired_position = 0;
        float desired_position_pulses = 0;
        float actual_position = 0;
        float actual_position_pulses = 0;
        float output_lower = 0;
        float output_upper = 0;
        float pidout = 0;
        float kp = 1;
        float ki = 0;
        float kd = 0;
        float Ts;
        float vibration = 0;
        int sign = 1;
    };

    struct MotorParameters {
        int gear_ratio = 1;
        int lines_per_rev = 1;
        int id = 0x201;
    };

    struct GPIOStruct {
        PinName motor_pwm;
        PinName motor_direction;
        PinName vibrator_pwm;
        PinName vibrator_direction;
        PinName EncoderChannelA;
        PinName EncoderChannelB;
        PinName EncoderIndex;
        PinName IntteruptPin;
        int PulsesPerRev;
    };



class Controller{

    public:
        Controller(PinName motor_pwm,
          PinName motor_direction,
           PinName vibrator_pwm,
            PinName vibrator_direction,
             PinName EncoderChannelA,
              PinName EncoderChannelB,
              PinName EncoderIndex,
              PinName IntteruptPin
              );
        void start();
        void stop();
        void update();
        void calibrate();
        void stop_calibration();
        void update_desired_position(float desired_position);
        void update_actual_position(float actual_position_pulses);
        void set_motor_gear_ratio(int gear_ratio);
        void set_pulses_per_revolution(int number);
        float radians_to_pulses(float radians);
        float pulses_to_radians(float pulses);
        void set_controller_parameters(ControlStruct controller_parameters);
        void set_motor_parameters(MotorParameters motor_parameters);
        void set_debug_mode(bool mode);

        ControlStruct control_parameters_;
        MotorParameters motor_parameters_; 
        std::shared_ptr<PID> pid;
        std::shared_ptr<QEI> encoder;
        std::shared_ptr<Motor> motor;
        std::shared_ptr<Motor> vibrator;
        std::shared_ptr<InterruptIn> limit_interupt;
        std::shared_ptr<Map> Pulses_to_speed;
        std::shared_ptr<Map> Angle_to_pulses;
        

        Ticker controller_ticker;
        bool debug_;
        bool is_calibrated;
        float degree_to_pulses;
        int id;
        controlelr_status_t status;





};



#endif


