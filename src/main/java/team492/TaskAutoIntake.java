package team492;

import TrcCommonLib.trclib.TrcRobot;

public class TaskAutoIntake
{
    private Robot robot;
    private Intake intake;

    public void intake(Robot robot)
    {
        boolean entranceHasBall = robot.conveyor.isEntranceSensorActive();
        boolean exitHasBall = robot.conveyor.isExitSensorActive();
        double pickupDelay = 0.5; // NEED TO TUNE 

        if (!entranceHasBall || !exitHasBall) //if not full
        {
            if (!entranceHasBall) // if front is empty
            {
                intake.pickup();
            }
            else if (entranceHasBall) // if front is not empty
            {
                robot.conveyor.advance();
                intake.pickup(pickupDelay); 
            }
            
        }
        return;

    }



}
