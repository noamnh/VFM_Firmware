#include "mbed.h"
#include "Controller.hpp"


void ConvertFloatToBytes(float x, unsigned char *data);

void ConvertIntToBytes(int x, unsigned char *data);

void ConvertBytesToInt(int &x,const unsigned char *data);

void ConvertBytesToFloat(float &x,const unsigned char *data);

void ExtractXBytes(const unsigned char *input, unsigned char *output, int index, int x);

CANMessage Start_Controller(CANMessage msg, std::shared_ptr<Controller> controller);

CANMessage Stop(CANMessage msg, std::shared_ptr<Controller> controller);

CANMessage Calibrate(CANMessage msg, std::shared_ptr<Controller> controller);

CANMessage Absolute_position(CANMessage msg,  std::shared_ptr<Controller> controller);

CANMessage Incremental_position(CANMessage msg, std::shared_ptr<Controller> controller);

CANMessage return_feedback(std::shared_ptr<Controller> controller,int recognizer);

CANMessage Command_recognizer(CANMessage msg, std::shared_ptr<Controller> controller);