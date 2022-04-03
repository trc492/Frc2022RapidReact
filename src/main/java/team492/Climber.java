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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInputTrigger;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcPidActuator.Parameters;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcDigitalInput;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climber
{
    private static final String moduleName = "Climber";
    private static final boolean debugEnabled = true;

    private final Robot robot;
    public final FrcCANFalcon climberMotor;
    public final FrcPneumatic climberPneumatic;
    public final FrcDigitalInput climberLowerLimitSwitch;
    public final TrcPidActuator climber;

    private TrcDbgTrace msgTracer = null;
    private final TrcDigitalInputTrigger limitSwitchTrigger;
    private final TrcEvent limitSwitchEvent;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject climberTaskObj;

    private boolean traversalRung = false;

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

        limitSwitchTrigger = new TrcDigitalInputTrigger(
            moduleName + ".limitSWTrigger", climberLowerLimitSwitch, this::limitSwitchEvent);
        limitSwitchEvent = new TrcEvent(moduleName + "limitSWEvent");
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

    /**
     * This method enables/disables tracing for the shooter subsystem.
     *
     * @param tracer specifies the tracer to use for logging events.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        msgTracer = tracer;
    }   //setMsgTracer

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

    public void setPosition(double position, boolean hold, TrcEvent event, double timeout)
    {
        climber.setTarget(position, hold, event, null, timeout);
    }   //setPosition

    public void setPosition(double position, boolean hold, TrcEvent event)
    {
        climber.setTarget(position, hold, event, null, 0.0);
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
                funcName, "[%.3f] RetractClimber: currPos=%.1f", TrcUtil.getModeElapsedTime(), climber.getPosition());
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
        PULL_DOWN_PRIMARY_HOOK,
        DEPLOY_SECONDARY_HOOK,
        LOWER_ONTO_RUNG,
        EXTEND_PRIMARY_HOOK,
        ENGAGE_NEXT_RUNG,
        UNHOOK_PREVIOUS_RUNG,
        DAMPENED_SWING,
        DONE
    }   //enum State

    public void traverseOneRung(boolean traversalRung)
    {
        this.traversalRung = traversalRung;
        sm.start(State.PULL_DOWN_PRIMARY_HOOK);
        climberTaskObj.registerTask(TaskType.POSTPERIODIC_TASK);
    }   //traverseOneRung

    /**
     * This method is called to cancel any pending operations and stop the subsystem. It is typically called before
     * exiting a competition mode.
     */
    public void cancel()
    {
        climber.setManualOverride(false);
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
                case PULL_DOWN_PRIMARY_HOOK:
                    climber.setManualOverride(true);
                    // Pull robot up.
                    sm.waitForSingleEvent(limitSwitchEvent, State.DEPLOY_SECONDARY_HOOK);
                    limitSwitchTrigger.setEnabled(true);
                    setPosition(26.0, true, null);
                    break;

                case DEPLOY_SECONDARY_HOOK:
                    // Deploy the hook arm to hook on the rung.
                    limitSwitchTrigger.setEnabled(false);
                    extendHookArm();
                    sm.waitForSingleEvent(event, State.LOWER_ONTO_RUNG);
                    timer.set(0.3, event);
                    break;

                case LOWER_ONTO_RUNG: //TODO: Test if doing a slow power on climber works as intended
                    // Run the primary hook up slowly to lower itself onto the rung
                    // This is to avoid the violent release of the current rung
                    setPower(0.1);
                    sm.waitForSingleEvent(event, State.EXTEND_PRIMARY_HOOK);
                    timer.set(1.0, event);
                    break;

                case EXTEND_PRIMARY_HOOK:
                    // Extend primary hook to engage the next rung.
                    sm.waitForSingleEvent(event, State.ENGAGE_NEXT_RUNG);
                    setPosition(59.0, true, event);
                    break;

                case ENGAGE_NEXT_RUNG:
                    // Retract hook arm allowing it to engage the next rung.
                    retractHookArm();
                    sm.waitForSingleEvent(event, State.UNHOOK_PREVIOUS_RUNG);
                    timer.set(traversalRung? 1.75: 1.0, event); //TODO: Time engaging of rung tighter?
                    break;

                case UNHOOK_PREVIOUS_RUNG:
                    // Pull up to the next rung to unhook the previous rung.
                    sm.waitForSingleEvent(event, State.DONE);//DAMPENED_SWING);
                    setPosition(26.0/*40.0*/, true, event, 1.8);
                    break;

                case DAMPENED_SWING:
                    sm.waitForSingleEvent(event, State.DONE);
                    setPosition(60.0, true, event);
                    break;

                case DONE:
                default: 
                    cancel();
                    break; 
            }
        }

        if (msgTracer != null)
        {
            robot.globalTracer.traceStateInfo(sm.toString(), state);
        }
    }   //autoClimbTask

    /**
     * This method is called when the lower limit switch is triggered.
     *
     * @param active specifies true if the climber primary hook has retracted all the way, false otherwise.
     */
    private void limitSwitchEvent(boolean active)
    {
        final String funcName = "limitSwitchEvent";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "active=%s", active);
        }

        if (active)
        {
            limitSwitchEvent.signal();
        }
    }   //limitSwitchEvent

}   //class Climber