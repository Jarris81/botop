#include <franka/gripper.h>
#include "gripper.h"

const char *gripperIpAddresses[2] = {"172.16.0.2", "172.17.0.2"};

FrankaGripper::FrankaGripper(uint whichRobot)
  : Thread(STRING("FrankaGripper_"<<whichRobot))
  , cmd(this, true){
  //-- choose robot/ipAddress
  CHECK_LE(whichRobot, 1, "");
  gripper = make_shared<franka::Gripper>(gripperIpAddresses[whichRobot]);
  franka::GripperState gripper_state = gripper->readOnce();
  maxWidth = gripper_state.max_width;
  LOG(0) <<"gripper max width:" <<maxWidth;
  if(maxWidth<.05){
    LOG(0) <<" -- no reasonable value -> homing gripper";
    homing();
  }

  threadOpen();
}

void FrankaGripper::homing(){
    auto cmdSet = cmd.set();
    cmdSet->cmd = GripperCmdMsg::Command::_home;
}

void FrankaGripper::open(double width, double speed){
  auto cmdSet = cmd.set();
  cmdSet->cmd = GripperCmdMsg::Command::_open;
  cmdSet->width = width;
  cmdSet->speed = speed;
}


void FrankaGripper::close(double force, double width, double speed){
  auto cmdSet = cmd.set();
  cmdSet->cmd = GripperCmdMsg::Command::_close;
  cmdSet->force = force;
  cmdSet->width = width;
  cmdSet->speed = speed;
}

double FrankaGripper::pos(){
  franka::GripperState gripper_state = gripper->readOnce();
  return gripper_state.width;
}

bool FrankaGripper::isGrasped(){
  franka::GripperState gripper_state = gripper->readOnce();
  return gripper_state.is_grasped;
}

void FrankaGripper::step() {
  GripperCmdMsg msg = cmd.get();

  if(msg.width>maxWidth){
      LOG(-1) <<"width " <<msg.width <<" is too large (max:" <<maxWidth <<')';
      msg.width = maxWidth - .001;
  }

  bool ret=true;

  switch(msg.cmd){
      case msg._close:
          ret = gripper->grasp(msg.width, msg.speed, msg.force, 0.08, 0.08);
          break;
      case msg._open:
          ret = gripper->move(msg.width, msg.speed);
          break;
      case msg._home:
          ret = gripper->homing();
          break;
  }

  if(!ret) LOG(-1) <<"gripper command " <<msg.cmd <<" failed (" <<msg.width <<' ' <<msg.speed <<')' <<endl;
}

void FrankaGripper::waitForIdle() {
  return Thread::waitForIdle();
}
