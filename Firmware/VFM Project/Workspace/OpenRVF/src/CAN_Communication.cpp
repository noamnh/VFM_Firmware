#include "CAN_Communication.hpp"



void ConvertBytesToFloat(float &x,const unsigned char *data){
    memcpy(&x, data, sizeof(float));
}

void ConvertFloatToBytes(float x, unsigned char *data){
    memcpy(data + 4, &x, sizeof(float));
}

void ConvertIntToBytes(int x, unsigned char *data){
    memcpy(data, &x, sizeof(int));
}

void ConvertBytesToInt(int &x,const unsigned char *data){
    memcpy(&x, data, sizeof(int));
}

void ExtractXBytes(const unsigned char *input, unsigned char *output, int index, int x) {
    const unsigned char *p = input + index;
    memcpy(output, p, x);
}

CANMessage Start(CANMessage msg, std::shared_ptr<Controller> controller){
    CANMessage response_msg;
    if(controller->is_calibrated){
    printf("(Controller) Motor is calibrated starting the controller \n");
    controller->start();
    response_msg.id = controller->id;
    response_msg.data[0] = 0x17;
    response_msg.data[1] = 0x01;
    }
    else{
    printf("(Controller) Motor is NOT calibrated !! CANNOT START THE CONTROLLER \n");
    controller->stop();
    response_msg.id = controller->id;
    response_msg.data[0] = 0x17;
    response_msg.data[1] = 0x00;
    }

    return response_msg;
}

CANMessage Stop(CANMessage msg, std::shared_ptr<Controller> controller){
    controller->stop();
    CANMessage response_msg;
    response_msg.id = controller->id;
    response_msg.data[0] = 0x18;
    return response_msg;
}

CANMessage Calibrate(CANMessage msg, std::shared_ptr<Controller> controller){
    controller->calibrate();
    CANMessage response_msg;
    response_msg.id = controller->id;
    response_msg.data[0] = 0x19;
    return response_msg;
}

CANMessage Absolute_position(CANMessage msg, std::shared_ptr<Controller> controller){
  unsigned char position[4];
  unsigned char vibration[2];
  ExtractXBytes(msg.data, position, 4, 4);
  ExtractXBytes(msg.data, vibration, 2, 2);
  ConvertBytesToFloat(controller->control_parameters_.desired_position, position);
  ConvertBytesToFloat(controller->control_parameters_.vibration, vibration);
              printf("(Controller)desired pos: %f \n", controller->control_parameters_.desired_position);

controller->update_desired_position(controller->control_parameters_.desired_position);

  return return_feedback(controller,0x20);

}

CANMessage Incremental_position(CANMessage msg, std::shared_ptr<Controller> controller){
  unsigned char position[4];
  unsigned char vibration[2];
  float desired_position;
  ExtractXBytes(msg.data, position, 4, 4);
  ExtractXBytes(msg.data, vibration, 2, 2);
  ConvertBytesToFloat(desired_position, position);
  ConvertBytesToFloat(controller->control_parameters_.vibration, vibration);
  controller->control_parameters_.desired_position = controller->control_parameters_.actual_position+desired_position;
  controller->update_desired_position(controller->control_parameters_.desired_position);

  return return_feedback(controller,0x21);
}

CANMessage return_feedback(std::shared_ptr<Controller> controller, int recognizer){
  CANMessage response_msg;
  ConvertFloatToBytes(controller->control_parameters_.actual_position, response_msg.data);
  response_msg.id = controller->id;
  response_msg.data[0] = recognizer;
  return response_msg;
}




CANMessage Command_recognizer(CANMessage msg, std::shared_ptr<Controller> controller){
    switch (msg.data[0]){
        
        case 0x17:
            return Start(msg,controller);
        case 0x18:
            return Stop(msg,controller);
        case 0x19:
            printf("(Controller) Got msg going for calibrating \n");
            return Calibrate(msg,controller);
        case 0x20:
            return Absolute_position(msg,controller);
        case 0x21:
            return Incremental_position(msg,controller);
        case 0x22:
            break;
        case 0x23:
            break;
        

        
    }
    CANMessage empty_can_message;
    empty_can_message.id = controller->id;
    empty_can_message.data[0] = 0xFF;
    return empty_can_message;

}


