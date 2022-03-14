/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcPidActuator.Parameters;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcDigitalInput;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climber
{
    private static final String moduleName = "Climber";
    private static final boolean debugEnabled = false;

    private final Robot robot;
    public final FrcCANFalcon climberMotor;
    public final FrcPneumatic climberPneumatic;
    public final FrcDigitalInput climberLowerLimitSwitch;
    public final TrcPidActuator climber;

    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject climberTaskObj;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for accessing other global objects.
     */
    public Climber(Robot robot)
    {
        this.robot = robot;
        climberMotor = createClimberMotor(moduleName + ".motor", RobotParams.CANID_CLIMBER);
        climberPneumatic = new FrcPneumatic(
            moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_CLIMBER_RETRACT, RobotParams.PNEUMATIC_CLIMBER_EXTEND);
        climberPneumatic.retract();

        climberLowerLimitSwitch = new FrcDigitalInput(
            moduleName + ".lowerLimitSwitch", RobotParams.DIO_CLIMBER_LOWER_LIMIT_SWITCH);
        Parameters params = new Parameters()
            .setPidParams(
                RobotParams.CLIMBER_KP, RobotParams.CLIMBER_KI, RobotParams.CLIMBER_KD, RobotParams.CLIMBER_TOLERANCE)
            .setPosRange(RobotParams.CLIMBER_MIN_POS, RobotParams.CLIMBER_MAX_POS)
            .setScaleOffset(RobotParams.CLIMBER_INCHES_PER_COUNT, RobotParams.CLIMBER_OFFSET)
            // .setStallProtectionParams(
            //     RobotParams.CLIMBER_STALL_MIN_POWER, RobotParams.CLIMBER_STALL_TOLERANCE,
            //     RobotParams.CLIMBER_STALL_TIMEOUT, RobotParams.CLIMBER_RESET_TIMEOUT)
            .setZeroCalibratePower(RobotParams.CLIMBER_CAL_POWER);
        climber = new TrcPidActuator(moduleName, climberMotor, climberLowerLimitSwitch, null, params);

        event = new TrcEvent(moduleName + ".event");
        timer = new TrcTimer(moduleName + ".timer");
        sm = new TrcStateMachine<>(moduleName);
        climberTaskObj = TrcTaskMgr.createTask(moduleName + ".climberTask", this::autoClimbTask);
    }   //Climber

    /**
     * This method creates and confiugres a climber motor.
     *
     * @param name specifies the name of the motor.
     * @param canID specifies the CAN ID of the motor.
     */
    private FrcCANFalcon createClimberMotor(String name, int canID)
    {
        FrcCANFalcon motor = new FrcCANFalcon(name, canID);

        motor.motor.configFactoryDefault();
        motor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);
        motor.motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyClosed, 10);
        motor.motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, 10);
        motor.motor.setSensorPhase(true);
        motor.setBrakeModeEnabled(true);
        motor.setInverted(RobotParams.CLIMBER_MOTOR_INVERTED);

        return motor;
    }   //createClimberMotor

    public boolean isLowerLimitSwitchActive()
    {
        return climberLowerLimitSwitch.isActive();
    }   //isLowerLimitSwitchActive

    //
    // Climber PID Actuator methods.
    //

    public void setPower(double power)
    {
        climber.setPower(power);
    }   //setPower

    public void setPosition(double position, boolean hold, TrcEvent event)
    {
        climber.setTarget(position, hold, event);
    }   //setPosition

    public void setPosition(double position)
    {
        climber.setTarget(position);
    }   //setPosition

    public void zeroCalibrateClimber()
    {
        final String funcName = "zeroClimber";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(
                funcName, "RetractClimber: currPos=%.1f", climber.getPosition());
        }

        climber.zeroCalibrate();
    }   //zeroCalibrateClimber

    //
    // Climber Pneumatic methods.
    //

    public void extendHookArm()
    {
        climberPneumatic.extend();
    }   //extendHookArm

    public void retractHookArm()
    {
        climberPneumatic.retract();
    }   //retractHookArm

    //
    // Climbing state machine.
    //

    private enum State
    {
        START,
        PULL_UP,
        DEPLOY_HOOK_ARM,
        UNHOOK_CLIMBER,
        REACH_FOR_NEXT_RUNG,
        RETRACT_HOOK_ARM,
        DONE
    }   //enum State

    State nextStage = State.START;
    int nextRung = 2;

    public void prepareToClimb()
    {
        sm.start(State.START);
        climberTaskObj.registerTask(TaskType.POSTPERIODIC_TASK);
    }   //prepareToClimb

    public void climbARung()
    {
        sm.start(State.PULL_UP);
        climberTaskObj.registerTask(TaskType.POSTPERIODIC_TASK);
    }   //climbARung

    public void reachForNextRung()
    {
        sm.start(State.REACH_FOR_NEXT_RUNG);
        climberTaskObj.registerTask(TaskType.POSTPERIODIC_TASK);
    }   //reachForNextRung

    /**
     * This method is called by TeleOp possibly tied to a button to execute the next stage of the climb.
     */
    public void executeNextState()
    {
        switch (nextStage)
        {
            case START:
                prepareToClimb();
                nextStage = State.PULL_UP;
                break;

            case PULL_UP:
                climbARung();
                nextStage = State.REACH_FOR_NEXT_RUNG;
                break;

            case REACH_FOR_NEXT_RUNG:
                reachForNextRung();
                nextRung++;
                nextStage = nextRung <= 4? State.PULL_UP: State.DONE;
                break;

            case DONE:
            default:
                // We are done, don't do anything.
                break;
        }
    }   //executeNextState

    /**
     * This method is called to cancel any pending operations and stop the subsystem. It is typically called before
     * exiting a competition mode.
     */
    public void cancel()
    {
        climber.setPower(0.0);
        sm.stop();
        climberTaskObj.unregisterTask();
    }   //cancel

    /**
     * This method is called periodically to execute the auto climb task.
     *
     * @param taskType specifies the task type.
     * @param runMode specifies the robot run mode.
     */
    private void autoClimbTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            switch (state)
            {
                case START:
                    // Extend climber to reach slightly beyond the 2nd rung.
                    // Then let the driver drive up to the rung.
                    retractHookArm();
                    setPosition(63.5, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case PULL_UP:
                    // Pull robot up.
                    setPosition(31.0, true, event);
                    sm.waitForSingleEvent(event, State.DEPLOY_HOOK_ARM);
                    break;

                case DEPLOY_HOOK_ARM:
                    // Deploy the hook arm to hook on the rung.
                    extendHookArm();
                    timer.set(0.1, event);
                    sm.waitForSingleEvent(event, State.UNHOOK_CLIMBER);
                    break;

                case UNHOOK_CLIMBER:
                    // Extend climber to release it from the rung.
                    // Then let the operator decide when to extend the climber to the next rung.
                    setPosition(45.0, true, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case REACH_FOR_NEXT_RUNG:
                    // Extend it fully to reach the next rung.
                    setPosition(63.5, false, event);
                    sm.waitForSingleEvent(event, State.RETRACT_HOOK_ARM);
                    break;

                case RETRACT_HOOK_ARM:
                    // Retract hook arm allowing it to be unhooked from the previous rung.
                    retractHookArm();
                    timer.set(0.1, event);
                    sm.waitForSingleEvent(event, State.UNHOOK_CLIMBER);
                    break;

                case DONE:
                default: 
                    cancel();
                    break; 
            }
        }

        if (debugEnabled)
        {
            robot.globalTracer.traceStateInfo(state);
        }
    }   //autoClimbTask

}   //class Climber