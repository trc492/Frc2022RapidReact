package team492;

import TrcCommonLib.trclib.TrcAutoTask;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcAHRSGyro;
import TrcCommonLib.trclib.TrcTimer;

public class TaskAutoBalance extends TrcAutoTask<TaskAutoBalance.State>
{
    private static final String moduleName = "TaskAutoBalance";

    public enum State
    {
        START_DELAY,
        MOVE_FORWARD,
        IMPULSE_BACK,
        DONE
    }
    
    private final String owner;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcTimer timer;
    private String currOwner = null;

    public TaskAutoBalance(String owner, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, owner, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.owner = owner;
        this.robot = robot;
        this.msgTracer = msgTracer;
        timer = new TrcTimer(moduleName);
    }

    public void autoAssistBalance()
    {
        startAutoTask(State.MOVE_FORWARD, null, null);
    }

    public void autoAssistBalanceCancel()
    {
        stopAutoTask(false);
    }

    @Override
    protected boolean acquireSubsystemsOwnership() {
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
    }

    @Override
    protected void releaseSubsystemsOwnership() {
        if(owner != null)
        {
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }
        
    }

    @Override
    protected void stopSubsystems() {
        robot.robotDrive.driveBase.stop(currOwner);
        
    }

    @Override
    protected void runTaskState(Object params, State state, TaskType taskType, RunMode runMode,
            boolean slowPeriodicLoop) {
        switch (state) {
            case START_DELAY:
                msgTracer.traceInfo(moduleName, "Start Delay");
                // if (autoChoices.getStartDelay() == 0.0) {
                    sm.setState(State.MOVE_FORWARD);
                // } else {
                //     timer.set(autoChoices.getStartDelay(), new TrcEvent.Callback() {
                //         @Override
                //         public void notify(Object context) {
                //             sm.setState(State.MOVE_FORWARD);
                //         }
                //     });
                // }
                break;

            case MOVE_FORWARD:
                float roll = ((FrcAHRSGyro)robot.robotDrive.gyro).ahrs.getRoll();
                msgTracer.traceInfo(moduleName, "Moving Forward; Gyro:=%.1f", roll);
                robot.robotDrive.pidDrive.driveMaintainHeading(0.0, 0.1, 0.0);
                if(Math.abs(roll) < 2.0) {
                    msgTracer.traceInfo(moduleName, ">>>>> Balanced <<<<<");
                    sm.setState(State.IMPULSE_BACK);
                }
                break;

            case IMPULSE_BACK:
                msgTracer.traceInfo(moduleName, "Impulsing back");
                robot.robotDrive.driveBase.holonomicDrive(0.0, -0.4, 0.0);
                timer.set(0.05, new TrcEvent.Callback() {
                    @Override
                    public void notify(Object context) {
                        msgTracer.traceInfo(moduleName, "Finished");
                        sm.setState(State.DONE);
                    }
                });
                break;

            case DONE:
            default:
                robot.robotDrive.setAntiDefenseEnabled("CmdAutoBalanec", true);
                stopAutoTask(true);
                break;
        }
    }
}
