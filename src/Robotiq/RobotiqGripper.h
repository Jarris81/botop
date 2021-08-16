#pragma once

//#include <Core/util.h>
#include <Core/thread.h>
#include <Franka/controlEmulator.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

using namespace boost::asio;

namespace robotiq{
    class Gripper;
}

//The control message send to the robot
struct GripperCmdMsg {
    enum Command { _open, _close, _home, _done };
    Command cmd=_done;
    double force=20;  //which is 1kg
    double width=.05; //which is 5cm
    double speed=.1;
};

struct RobotiqGripper : rai::GripperAbstraction, Thread{
    Var<GripperCmdMsg> cmd;

//    serial_port port;
    double maxWidth;

    RobotiqGripper(uint whichRobot=0);
    ~RobotiqGripper(){ threadClose(); }

    void homing();

    void open(double width=.075, //which is 7.5cm
              double speed=.2);

    void close(double force=20,  //which is 1kg
               double width=.05, //which is 5cm
               double speed=.1);

    double pos();

    bool isGrasped();

    void step();

    void waitForIdle();

private:
    std::shared_ptr<serial_port> serialPort;
};
