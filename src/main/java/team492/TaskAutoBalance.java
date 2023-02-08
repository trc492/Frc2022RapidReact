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
        robot.robotDrive.balancePidDrive.cancel();
        
    }   //stopSubsystems

    @Override
    protected void runTaskState(
        Object params, State state, TaskType taskType, RunMode runMode, boolean slowPeriodicLoop)
    {
        switch (state)
        {
            case MOVE_FORWARD:
                // Move forward slowly until the robot is tipping backward.
                if (Math.abs(robot.robotDrive.getGyroYHeading()) < 2.0)
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
                    currOwner, robot.robotDrive.driveBase.getXPosition(), 0.0, 0.0, true, event, 0.0);
                //TODO: Event never fires
                sm.waitForSingleEvent(event, State.DONE);
                break;

            case DONE:
            default:
                robot.robotDrive.setAntiDefenseEnabled(currOwner, true);
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoBalance
