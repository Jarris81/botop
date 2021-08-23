
#include <KOMO/komo.h>
#include <KOMO/pathTools.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

#if 1
arr getStartGoalPath2(rai::Configuration& C, const arr& target_q, const arr& qHome, const FrameL& collisionPairs) {

  arr q0 = C.getJointState();

  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(1., 32, 5., 2);
  komo.add_qControlObjective({}, 2, 1.);

  //constrain target - either endeff target or qTarget
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, target_q);

  //final still
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, {}, 1);

  //homing
  if(qHome.N) komo.addObjective({.4,.6}, FS_qItself, {}, OT_sos, {1.}, qHome);

  // collision avoidances
  if(collisionPairs.N){
    komo.addObjective({}, FS_distance, framesToNames(collisionPairs), OT_ineq, {1e2}, {-.001});
  }
//  for(const Avoid& a:avoids){
//    komo.addObjective(a.times, FS_distance, a.frames, OT_ineq, {1e1}, {-a.dist});
//  }
//  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e2});

//  komo.initWithWaypoints({target_q});
//  komo.initWithConstant(target_q);
  komo.optimize(.1, OptOptions().set_stopTolerance(1e-3));

  //  cout <<komo.getReport(true) <<endl;
  cout <<"  path -- time:" <<komo.timeTotal <<"\t sos:" <<komo.sos <<"\t ineq:" <<komo.ineq <<"\t eq:" <<komo.eq <<endl;

  arr path = komo.getPath_qOrg();

  if(komo.sos>50. || komo.ineq>.1 || komo.eq>.1){
    FILE("z.path") <<path <<endl;
    cout <<komo.getReport(true) <<endl;
    LOG(-2) <<"WARNING!";
    while(komo.view_play(true));
#if 0
    //repeat
    komo.initWithConstant(q0);
    komo.optimize(.1, OptOptions().set_stopTolerance(1e-3));
#endif

    return {};
  }

  return path;
}
#endif

//===========================================================================

void rndPoses(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  arr qHome = C.getJointState();
  arr limits = C.getLimits();
  FrameL collisionPairs = C.getCollisionAllPairs();
  for(uint i=0;i<collisionPairs.d0;i++){
    cout <<"PAIR: " <<collisionPairs(i,0)->name <<' ' <<collisionPairs(i,1)->name <<endl;
  }

  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);
  arr bounds = ~C.getLimits();

  uint N=100;
  for(uint i=0;i<N;i++){
    cout <<" ====== POSE " <<i <<" ====== " <<endl;
    arr x = bounds[0] + (bounds[1]-bounds[0]) % rand(qHome.N);

    C.setJointState(x);
    //C.watch(false, STRING("conf " <<i));
    bool succ = checkCollisionsAndLimits(C, collisionPairs, limits, true);
    if(succ){
      cout <<" === pose made feasible === " <<endl;
      x = C.getJointState();
      //C.watch(true, STRING("conf " <<i));

      C.setJointState(bot.get_q());
      arr path = getStartGoalPath2(C, x, bot.qHome, collisionPairs);
      if(!path.N){
        cout <<" === path infeasible === " <<endl;
        continue;
      }
      cout <<" === path feasible -> executing === " <<endl;
      bot.moveAutoTimed(path, .01);
      while(bot.step(C));
      if(bot.keypressed=='q') break;
    }else{
      cout <<" === pose infeasible === " <<endl;
    }
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();

  //  driveToPoses();
  rndPoses();

  return 0;
}
