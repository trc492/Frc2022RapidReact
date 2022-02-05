package team492;

import TrcCommonLib.trclib.TrcDigitalInputTrigger;
import TrcCommonLib.trclib.TrcNotifier;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcNotifier.Receiver;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcDigitalInput;

public class Conveyor {
    TrcPidActuator conveyor ;
    FrcCANTalon conveyorMotor; 
    FrcDigitalInput entranceSensor; 
    FrcDigitalInput exitSensor;
    TrcDigitalInputTrigger entranceTrigger; 
    Receiver entranceReceiver; 
    int numBallsInConveyor = 0; 
    public Conveyor(String instanceName, int port, int numberPreloadBalls){
        numBallsInConveyor = numberPreloadBalls ; 
        conveyorMotor = new FrcCANTalon("conveyor", port);
        final TrcPidActuator.Parameters conveyorParams = new TrcPidActuator.Parameters()
        .setPidParams(new TrcPidController.PidParameters(
            RobotParams.CONVEYOR_KP, RobotParams.CONVEYOR_KI, RobotParams.CONVEYOR_KD, RobotParams.CONVEYOR_TOLERANCE))
        .setMotorParams(
            RobotParams.CONVEYOR_MOTOR_INVERTED,
            RobotParams.CONVEYOR_HAS_ENTRANCE_SENSOR, RobotParams.CONVEYOR_ENTRANCE_SENSOR_INVERTED,
            RobotParams.CONVEYOR_HAS_EXIT_SENSOR, RobotParams.CONVEYOR_EXIT_SENSOR_INVERTED,
            RobotParams.CONVEYOR_CAL_POWER)
        .setStallProtectionParams(
            RobotParams.CONVEYOR_STALL_MIN_POWER, RobotParams.CONVEYOR_STALL_TIMEOUT, RobotParams.CONVEYOR_RESET_TIMEOUT);

        //make exit and entrance sensors
        if (RobotParams.CONVEYOR_HAS_ENTRANCE_SENSOR)
        {
            entranceSensor =
                new FrcDigitalInput(instanceName + ".entranceSensor", RobotParams.CONVEYOR_ENTRANCE_SENSOR_PORT);
            entranceTrigger = new TrcDigitalInputTrigger(instanceName + ".entranceTrigger", entranceSensor, this::entranceTriggerEvent);
    
        }
        exitSensor =
        RobotParams.CONVEYOR_HAS_EXIT_SENSOR?
            new FrcDigitalInput(RobotParams.HWNAME_CONVEYOR + ".exitSensor", RobotParams.CONVEYOR_EXIT_SENSOR_PORT): null;

        conveyor = new TrcPidActuator(RobotParams.HWNAME_CONVEYOR, conveyorMotor, entranceSensor, exitSensor, conveyorParams);
            

        //write autochoices for number of preload ball 
    }
    public void advanceOneBall(){
        conveyor.setTarget(RobotParams.CONVEYOR_ADVANCE_BALL_DISTANCE);
    }
    public void advanceTwoBalls(){
        conveyor.setTarget(2 * RobotParams.CONVEYOR_ADVANCE_BALL_DISTANCE);

    }
    //when trigger happens call this method: adds a ball, if anybody wants to know about entrance receiver, call him
    //need state bc there is one state for beam broken and another when ball passes the beam 
    void entranceTriggerEvent(boolean state)
    {
        if(state){
            numBallsInConveyor++;
            if(entranceReceiver!=null){
                entranceReceiver.notify(null);
            }
        }
    }
    void exitTriggerEvent(boolean state)
    {
        if(state){
            numBallsInConveyor--;
            if(entranceReceiver!=null){
                entranceReceiver.notify(null);
            }
        }   
    }
    public int numBallsInConveyor(){
        return numBallsInConveyor;
    }
    
    //call this method if you want to receive notifications for entrance trigger. 
    public void registerEntranceTrigger(TrcNotifier.Receiver receiver){
        entranceReceiver = receiver; 

    }


    
    
}
