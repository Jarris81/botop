#include "RobotiqGripper.h"
#include <boost/asio.hpp>

//const char *gripperIpAddresses[2] = {"172.16.0.2", "172.17.0.2"};

RobotiqGripper::RobotiqGripper(uint whichRobot)
: Thread(STRING("RobotiqGripper_"<<whichRobot))
, cmd(this, false){
  //-- choose robot/ipAddress
  CHECK_LE(whichRobot, 1, "");
//  robotiqGripper = make_shared<robotiq::Gripper>(gripperIpAddresses[whichRobot]);

  threadOpen();

  io_service io;
  serialPort = std::make_shared<serial_port>(io);

//  serialPort->open("Somename");
  serialPort->set_option(serial_port_base::baud_rate(115200));
  serialPort->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
  serialPort->set_option(serial_port_base::character_size(8));
  serialPort->set_option(serial_port_base::parity(serial_port_base::parity::none));

  // send initialization command


}


void RobotiqGripper::homing(){
  {
    auto cmdSet = cmd.set();
    cmdSet->cmd = GripperCmdMsg::Command::_home;
  }
  threadStep();
}

void RobotiqGripper::open(double width, double speed){
  {
    auto cmdSet = cmd.set();
    cmdSet->cmd = GripperCmdMsg::Command::_open;
    cmdSet->width = width;
    cmdSet->speed = speed;
  }
  threadStep();
}


void RobotiqGripper::close(double force, double width, double speed){
  {
    auto cmdSet = cmd.set();
    cmdSet->cmd = GripperCmdMsg::Command::_close;
    cmdSet->force = force;
    cmdSet->width = width;
    cmdSet->speed = speed;
  }
  threadStep();
}

double RobotiqGripper::pos(){
  //TODO
  std::cout<<"Hello"<<endl;
}

bool RobotiqGripper::isGrasped(){
  //TODO
  return true;
}

void RobotiqGripper::step() {

  GripperCmdMsg msg = cmd.set();

  if(msg.cmd==msg._done) return;

  if(msg.width>maxWidth){
    LOG(-1) <<"width " <<msg.width <<" is too large (max:" <<maxWidth <<')';
    msg.width = maxWidth - .001;
  }

  bool ret=true;

  switch(msg.cmd){
    case msg._close:
      //ret = robotiqGripper->grasp(msg.width, msg.speed, msg.force, 0.08, 0.08);
      break;
    case msg._open:
      //ret = frankaGripper->move(msg.width, msg.speed);
      break;
    case msg._home:
      //ret = frankaGripper->homing();
      break;
    case msg._done:
      break;
  }

  //return value means something else
  //if(!ret) LOG(-1) <<"gripper command " <<msg.cmd <<" failed (" <<msg.width <<' ' <<msg.speed <<')' <<endl;
  //LOG(0) <<"gripper command " <<msg.cmd <<" SEND (" <<msg.width <<' ' <<msg.speed <<' ' <<msg.force  <<')' <<endl;

}

void RobotiqGripper::waitForIdle() {
  return Thread::waitForIdle();
}