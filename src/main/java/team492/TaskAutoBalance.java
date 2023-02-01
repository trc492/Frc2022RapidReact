package team492;

import TrcCommonLib.trclib.TrcAutoTask;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;

public class TaskAutoBalance extends TrcAutoTask<TaskAutoBalance.State>
{
    private static final String moduleName = "TaskAutoBalance";

    public enum State
    {
        MOVE_FORWARD,
        BALANCING,
        DONE
    }
    
    private final String owner;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent event;
    private String currOwner = null;

    public TaskAutoBalance(String owner, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, owner, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.owner = owner;
        this.robot = robot;
        this.msgTracer = msgTracer;
        event = new TrcEvent(moduleName);
    }   //TaskAutoBalance

    public void autoAssistBalance()
    {
        startAutoTask(State.MOVE_FORWARD, null, null);
    }   //autoAssistBalance

    public void autoAssistBalanceCancel()
    {
        stopAutoTask(false);
    }   //autoAssistBalanceCancel

    @Override
    protected boolean acquireSubsystemsOwnership()
    {
        boolean success = owner == null ||
            (robot.robotDrive.driveBase.acquireExclusiveAccess(owner));

        if (success)
        {
            currOwner = owner;
        }
        else
        {
            releaseSubsystemsOwnership();
        }

        return success;
    }   //acquireSubsystemsOwnership

    @Override
    protected void releaseSubsystemsOwnership()
    {
        if(owner != null)
        {
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }
    }   //releaseSubsystemsOwnership

    @Override
    protected void stopSubsystems()
    {
        robot.robotDrive.driveBase.stop(currOwner);
        
    }   //stopSubsystems

    @Override
    protected void runTaskState(
        Object params, State state, TaskType taskType, RunMode runMode, boolean slowPeriodicLoop)
    {
        switch (state)
        {
            case MOVE_FORWARD:
                // Move forward slowly until the robot is tipping backward.
                if (Math.abs(robot.robotDrive.getGyroXHeading()) < 2.0)
                {
                    robot.robotDrive.driveBase.holonomicDrive(currOwner, 0.0, 0.2, 0.0);
                }
                else
                {
                    robot.robotDrive.driveBase.stop(currOwner);
                    sm.setState(State.BALANCING);
                }
                break;

            case BALANCING:
                msgTracer.traceInfo(moduleName, "Balancing.");
                robot.robotDrive.balancePidDrive.setSensorTarget(
                    currOwner, 0.0, 0.0, 0.0, true, event, 0.0);
                sm.waitForSingleEvent(event, State.DONE);
                // float roll = ((FrcAHRSGyro)robot.robotDrive.gyro).ahrs.getRoll();
                // msgTracer.traceInfo(moduleName, "Moving Forward; Gyro:=%.1f", roll);
                // robot.robotDrive.pidDrive.driveMaintainHeading(0.0, 0.1, 0.0);
                // if(Math.abs(roll) < 2.0) {
                //     msgTracer.traceInfo(moduleName, ">>>>> Balanced <<<<<");
                //     sm.setState(State.IMPULSE_BACK);
                // }
                break;

            // case IMPULSE_BACK:
            //     msgTracer.traceInfo(moduleName, "Impulsing back");
            //     robot.robotDrive.driveBase.holonomicDrive(0.0, -0.4, 0.0);
            //     timer.set(0.05, new TrcEvent.Callback() {
            //         @Override
            //         public void notify(Object context) {
            //             msgTracer.traceInfo(moduleName, "Finished");
            //             sm.setState(State.DONE);
            //         }
            //     });
            //     break;

            case DONE:
            default:
                robot.robotDrive.setAntiDefenseEnabled(currOwner, true);
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoBalance
