// package team492;

// import TrcCommonLib.trclib.TrcEvent;
// import TrcCommonLib.trclib.TrcPidController;
// import TrcCommonLib.trclib.TrcPose2D;
// import TrcCommonLib.trclib.TrcRobot;
// import TrcCommonLib.trclib.TrcStateMachine;
// import TrcCommonLib.trclib.TrcTaskMgr;
// import TrcCommonLib.trclib.TrcUtil;
// import TrcCommonLib.trclib.TrcNotifier.Receiver;
// import TrcCommonLib.trclib.TrcTaskMgr.Task;
// //objective: task runs on its own thread, you can call it to start running code that shoots the ball 
// import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
// import TrcFrcLib.frclib.FrcRemoteVisionProcessor.RelativePose;

// public class TaskAimShooter
// {

//     private enum State{
//         SHIFT_BALL_IN_CONVEYOR, PREP_FOR_AIM, AIM, PREP_SHOOTER_BALL_2, SHOOT, DONE

//     }



//     private final String instanceName = "AutoShooter";

//     private Robot robot;
//     public TrcTaskMgr.TaskObject shooterTaskObject;
//     private TrcEvent event;
//     private TrcEvent event2;
//     private final TrcStateMachine<State> sm; 
//     private int numBallsShot = 0;
//     private int totalBallsToShoot = 1; 
//     private TrcEvent onFinishEvent;
//     private double targetShooterVelocity = 0.0;

//     private RelativePose targetPos;

//     private double tx;

//     private double tiltAngle;

//     private double distanceToTarget; 

//     public Receiver conveyorExitSensorReceiver; 
    

//     public TaskAimShooter(Robot robot)
//     {
//         this.robot = robot;
//         shooterTaskObject = TrcTaskMgr.createTask(instanceName + ".shooterTask", this::doShooterTask);
        
//         sm = new TrcStateMachine<>(instanceName);
//         event = new TrcEvent(instanceName + ".event");
//         event2 = new TrcEvent(instanceName+".event2");

//     }



//     private void cancel()
//     {
        
//         shooterTaskObject.unregisterTask();  
//         onFinishEvent.notify(); 
//         robot.shooter.releaseExclusiveAccess(instanceName);
//         robot.conveyor.releaseExclusiveAccess(instanceName);
//         sm.stop();
        
//     }
//     //assume the target is in view of limelight before this is called 
//     public void aimAndShoot(TrcEvent event, boolean shoot2Balls){
//         this.onFinishEvent = event; 
//         if(robot.conveyor.isEntranceSensorActive()){
//             totalBallsToShoot++;
//         }
//         if(robot.conveyor.isExitSensorActive()){
//             totalBallsToShoot++;
//         }
//         if(!shoot2Balls){
//             totalBallsToShoot --; 
//         }
//         if(totalBallsToShoot>0){
//             shooterTaskObject.registerTask(TaskType.POSTPERIODIC_TASK);
//             sm.start(State.AIM);
//         }
//     }
//     //state when you only want to shoot one ball 
//     public void shootOneBall(TrcEvent event ){

//     }
//     //shooter task, at least 1 ball in the entrance or exit sensor of conveyor 
//     public void doShooterTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
//     {
//         State state = sm.checkReadyAndGetState();
//         switch(state){
//             case PREP_FOR_AIM:
//                 if(!robot.shooter.acquireExclusiveAccess(instanceName)||robot.conveyor.acquireExclusiveAccess(instanceName)){
//                     sm.setState(State.DONE);
//                 }
//                 else{ 
//                     sm.setState(State.SHIFT_BALL_IN_CONVEYOR);
//                 }
//             break;
//             case SHIFT_BALL_IN_CONVEYOR:
//                 // if there is no ball at the exit sensor advance the ball to there 
//                 if(robot.conveyor.isExitSensorActive()){
//                     robot.conveyor.advance(instanceName, event);
//                     sm.waitForSingleEvent(event, State.AIM );
//                 }
//                 //otherwise go to the aim state
//                 else{
//                     sm.setState(State.AIM);
//                 }
                
//             case AIM:
//                 //rotate robot tx degrees based on vision
//                 //use vision.getLastPose() to find distance to target 
//                 //prepare shooter to be ready to shoot(tilt and flywheel velocity )
//                 if(robot.vision.vision.targetDetected()){
//                     targetPos = robot.vision.getLastPose();
//                     tx = robot.vision.vision.get("tx");
//                     tiltAngle = robot.vision.vision.get("ty");
//                     distanceToTarget = TrcUtil.magnitude(targetPos.x, targetPos.y);
//                     //turn the robot tx degrees
//                     robot.robotDrive.purePursuitDrive.start(event, robot.robotDrive.driveBase.getFieldPosition(), true, new TrcPose2D(0, 0, tx));
//                     //prep flywheels and tilter for shooting based on distancce to target 
//                     robot.shooter.prepForShoot(instanceName, tiltAngle, distanceToTarget, event2);
//                     sm.addEvent(event);
//                     sm.addEvent(event2);
//                     sm.waitForEvents(State.SHOOT, 0.0, true);
//                 }
//                 else{
//                     sm.setState(State.DONE);
                    
//                 }
//             break; 
//             //this state is for after preparing to shoot the second ball
//             //need to 
//             case PREP_SHOOTER_BALL_2: 
//                 //because we just shot a ball from the exit sensor, need to advance the other ball to the exit sensor 
//                 robot.conveyor.advance(instanceName, event);
//                 //prepare for shooting(spin flywheels up to speed)
//                 robot.shooter.setFlywheelVelocity(targetShooterVelocity, event2);
//                 sm.addEvent(event);
//                 sm.addEvent(event2);
//                 sm.waitForEvents(State.SHOOT, 0.0, true);
//             break; 
//             case SHOOT:
//                 robot.shooter.shoot(instanceName, event);
//                 totalBallsToShoot--; 
//                 if(totalBallsToShoot==0){
//                     sm.waitForSingleEvent(event, State.DONE);
//                 }
//                 else{
//                     sm.waitForSingleEvent(event, State.PREP_SHOOTER_BALL_2);
//                 }
//             break; 
//             case DONE:
//                 cancel();
//             break; 

//         }

//     }
// }

       