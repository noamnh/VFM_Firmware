/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include <memory>
#define MOTOR_PWN_PIN PB_1
#define MOTOR_DIRECTION_PIN PB_0
#define VIBRATOR_PWM_PIN PA_4
#define VIBRATOR_DIRECTION_PIN PA_5
#define ENCODER_CHANNEL_A_PIN PF_0
#define ENCODER_CHANNEL_B_PIN PF_1
#define OPTIC_SWITCH_PIN PB_4

#include <unordered_map>
#include <vector>
#include "mbed.h"
#include "Controller.hpp"
#include "CAN_Communication.hpp"

#define WAIT_TIME_MS 500 
DigitalOut led1(LED1);
CAN can(PA_11,PA_12);
std::shared_ptr<Controller> VFM;
ControlStruct controller_struct;
MotorParameters motor_struct;

std::vector<std::shared_ptr<Controller>> controller_vector;
std::unordered_map<int, std::shared_ptr<Controller>> id_map;

CANMessage request_msg;
CANMessage response_msg;
unsigned char recognizer;

int main()
{

    can.frequency(1000000);

    VFM = std::make_shared<Controller>(MOTOR_PWN_PIN,MOTOR_DIRECTION_PIN,VIBRATOR_PWM_PIN,VIBRATOR_DIRECTION_PIN,ENCODER_CHANNEL_A_PIN,ENCODER_CHANNEL_B_PIN, NC,OPTIC_SWITCH_PIN);

    VFM->set_debug_mode(false);
    controller_struct.kp = 5;
    controller_struct.ki = 1.0;
    controller_struct.kd = 0.0;
    controller_struct.Ts = 0.00001;
    controller_struct.output_upper = 360;
    controller_struct.output_lower = -360;

    VFM->set_controller_parameters(controller_struct);

    motor_struct.gear_ratio = 200;
    motor_struct.lines_per_rev = 14;
    motor_struct.id = 0x201;


    VFM->set_motor_parameters(motor_struct);
    controller_vector.emplace_back(VFM);


    for(auto &controller: controller_vector){
        id_map[controller->id] = controller;
    }
    


    while (true)
    {   

        if(can.read(request_msg)){
            led1 = !led1;
            auto controller = id_map[request_msg.id];
            response_msg = Command_recognizer(request_msg, controller);
            can.write(response_msg);
            
        }

        for(auto &controller: controller_vector){
            if(controller->status == CONTROLLER_STATUS::RUN){
                controller->update();

            }
        }


    }
}
