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

import java.util.Locale;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcValueSensorTrigger;
import TrcCommonLib.trclib.TrcPidActuator.Parameters;
import TrcCommonLib.trclib.TrcPidController.PidParameters;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcDigitalInput;

public class Shooter implements TrcExclusiveSubsystem
{
    private static final String moduleName  = "Shooter";
    private static final boolean debugEnabled = false;

    public class ShooterPreset
    {
        public final String name;
        public final double lowerFlywheelVelocity;
        public final double upperFlywheelVelocity;
        public final double tilterAngle;

        public ShooterPreset(
            String name, double lowerFlywheelVelocity, double upperFlywheelVelocity, double tilterAngle)
        {
            this.name = name;
            this.lowerFlywheelVelocity = lowerFlywheelVelocity;
            this.upperFlywheelVelocity = upperFlywheelVelocity;
            this.tilterAngle = tilterAngle;
        }   //ShooterPreset

        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "name=%s, lowerFwVel=%.0f, upperFwVel=%.0f, tilterAngle=%.2f",
                name, lowerFlywheelVelocity, upperFlywheelVelocity, tilterAngle);
        }   //toString

    }   //class ShooterPreset

    private enum State
    {
        START,
        SHOOT,
        DONE
    }   //enum State

    private final Robot robot;
    private final FrcCANFalcon lowerFlywheelMotor, upperFlywheelMotor;
    private final TrcValueSensorTrigger flywheelVelocityTrigger;
    private final FrcCANTalon tilterMotor;
    private final FrcDigitalInput tilterLowerLimitSwitch;
    private final TrcPidActuator tilter;
    private final TrcEvent flywheelEvent;
    private final TrcEvent tilterEvent;
    private final TrcEvent conveyorEvent;
    private final TrcEvent driveEvent;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject shooterTaskObj;

    private boolean flywheelInVelocityMode = false;
    private TrcEvent flywheelToSpeedEvent = null;
    private String currOwner = null;

    private boolean usingVision = false;
    private double lowerFlywheelSetVel = 0.0;
    private double upperFlywheelSetVel = 0.0;
    private double tilterSetAngle = 0.0;
    private TrcEvent onFinishShootingEvent = null;

    Double expireTime = null; 

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object to access all robot subsystems.
     */
    public Shooter(Robot robot)
    {
        this.robot = robot; 
        //
        // Create and configure Flywheel related objects.
        //
        lowerFlywheelMotor = createFlywheelMotor(
            moduleName + ".lowerFlywheelMotor", RobotParams.CANID_SHOOTER_LOWER_FLYWHEEL, true);
        upperFlywheelMotor = createFlywheelMotor(
            moduleName + ".upperFlywheelMotor", RobotParams.CANID_SHOOTER_UPPER_FLYWHEEL, true);
        flywheelVelocityTrigger = new TrcValueSensorTrigger(
            moduleName + ".flywheelVelTrigger", this::getUpperFlywheelVelocity, this::flywheelTriggerEvent);
        setFlywheelVelocityModeEnabled(true);
        //
        // Create and configure Tilter related objects.
        //
        tilterMotor = createTilterMotor(moduleName + ".tilterMotor", RobotParams.CANID_SHOOTER_TILTER);
        tilterMotor.setPositionSensorInverted(true);
        tilterLowerLimitSwitch = new FrcDigitalInput(
            moduleName + ".lowerLimitSwitch", RobotParams.DIO_TILTER_LOWER_LIMIT_SWITCH);
        Parameters tilterParams = new Parameters()
            .setPidParams(
                new PidParameters(
                    RobotParams.TILTER_KP, RobotParams.TILTER_KI, RobotParams.TILTER_KD, RobotParams.TILTER_TOLERANCE))
            .setPosRange(RobotParams.TILTER_MIN_POS, RobotParams.TILTER_MAX_POS)
            .setScaleOffset(RobotParams.TILTER_DEG_PER_COUNT, RobotParams.TILTER_OFFSET)
            .setZeroCalibratePower(RobotParams.TILTER_CAL_POWER);
            // .setStallProtectionParams(
            //     RobotParams.TILTER_STALL_MIN_POWER, RobotParams.TILTER_STALL_TOLERANCE,
            //     RobotParams.TILTER_STALL_TIMEOUT, RobotParams.TILTER_RESET_TIMEOUT);
        tilter = new TrcPidActuator(moduleName + ".tilter", tilterMotor, tilterLowerLimitSwitch, null, tilterParams);
        //
        // Create and initialize other objects.
        //
        flywheelEvent = new TrcEvent(moduleName + ".flywheelEvent");
        tilterEvent = new TrcEvent(moduleName + ".tilterEvent");
        conveyorEvent = new TrcEvent(moduleName + ".conveyorEvent");
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        sm = new TrcStateMachine<>(moduleName);
        shooterTaskObj = TrcTaskMgr.createTask(moduleName + ".shooterTask", this::autoShootTask);
    }   //Shooter

    /**
     * This method creates and confiugres a flywheel motor.
     *
     * @param name specifies the name of the motor.
     * @param canID specifies the CAN ID of the motor.
     * @param inverted specifies true if the motor is inverted, false otherwise.
     */
    private FrcCANFalcon createFlywheelMotor(String name, int canID, boolean inverted)
    {
        FrcCANFalcon motor = new FrcCANFalcon(name, canID);

        motor.motor.configFactoryDefault();
        // TODO: Make sure we have updated the PID coeff from Phoenix Tuner before enabling this or we would have overwritten them.
        // motor.motor.config_kP(0, RobotParams.FLYWHEEL_KP, 10);
        // motor.motor.config_kI(0, RobotParams.FLYWHEEL_KI, 10);
        // motor.motor.config_kD(0, RobotParams.FLYWHEEL_KD, 10);
        // motor.motor.config_kF(0, RobotParams.FLYWHEEL_KF, 10);
        // motor.motor.config_IntegralZone(0, RobotParams.FLYWHEEL_IZONE, 10);
        motor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);
        motor.setBrakeModeEnabled(false);
        motor.setInverted(inverted);

        return motor;
    }   //createFlywheelMotor

    /**
     * This method creates and confiugres a tilter motor.
     *
     * @param name specifies the name of the motor.
     * @param canID specifies the CAN ID of the motor.
     */
    private FrcCANTalon createTilterMotor(String name, int canID)
    {
        FrcCANTalon motor = new FrcCANTalon(name, canID);

        motor.motor.configFactoryDefault();
        // TODO: Decide whether to go with motor PID control or software PID control, disabled for now to use software PID control.
        // motor.motor.config_kP(0, RobotParams.TILTER_KP, 10);
        // motor.motor.config_kI(0, RobotParams.TILTER_KI, 10);
        // motor.motor.config_kD(0, RobotParams.TILTER_KD, 10);
        // motor.motor.config_kF(0, RobotParams.TILTER_KF, 10);
        // motor.motor.configAllowableClosedloopError(0, RobotParams.TILTER_TOLERANCE/RobotParams.TILTER_DEG_PER_COUNT, 10);
        motor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);
        motor.setBrakeModeEnabled(true);
        motor.setInverted(RobotParams.TILTER_MOTOR_INVERTED);

        motor.motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        motor.motor.setSensorPhase(false);
        SensorCollection sensorCollection = motor.motor.getSensorCollection();
        sensorCollection.setPulseWidthPosition(0, 10); // reset index
        TrcUtil.sleep(50); // guarantee reset
        ErrorCode error = sensorCollection.syncQuadratureWithPulseWidth(0, 0, true, RobotParams.TILTER_ZERO, 10);
        if (error != ErrorCode.OK)
        {
            robot.globalTracer.traceErr(moduleName, "Failed to configure encoder (error=%s).", error.name());
        }
        TrcUtil.sleep(50); // guarantee reset

        robot.globalTracer.traceInfo(
            moduleName, "Tilter: zero=%d, pwmPos=%d, quadPos=%d, selectedPos=%f",
            RobotParams.TILTER_ZERO, sensorCollection.getPulseWidthPosition(),
            sensorCollection.getQuadraturePosition(), motor.motor.getSelectedSensorPosition());

        return motor;
    }   //createTilterMotor

    /**
     * This method is called to cancel the autoShoot operation.
     */
    public void cancel()
    {
        final String funcName = "cancel";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "Canceling: currOwner=%s", currOwner);
        }

        setFlywheelValue(currOwner, 0.0, 0.0, null);
        tilter.cancel(currOwner);
        robot.conveyor.setPower(currOwner, 0.0, 0.0, 0.0, null);
        robot.robotDrive.driveBase.stop();
        if (currOwner != null)
        {
            this.releaseExclusiveAccess(currOwner);
            robot.conveyor.releaseExclusiveAccess(currOwner);
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }

        if (onFinishShootingEvent != null)
        {
            onFinishShootingEvent.signal();
            onFinishShootingEvent = null;
        }
        sm.stop();
        shooterTaskObj.unregisterTask();
    }   //cancel

    //
    // Flywheel methods.
    //

    /**
     * This method enables/disables flywheel velocity mode.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     * @param enabled specifies true to enable velocity mode, false to disable.
     */
    public void setFlywheelVelocityModeEnabled(String owner, boolean enabled)
    {
        final String funcName = "setFlywheelVelocityModeEnabled";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "enabled=%s, flywheelMaxVel=%d", enabled, RobotParams.FLYWHEEL_MAX_RPM);
        }

        if (validateOwnership(owner))
        {
            flywheelInVelocityMode = enabled;
            if (enabled)
            {
                lowerFlywheelMotor.enableVelocityMode(RobotParams.FLYWHEEL_MAX_VEL, RobotParams.SHOOTER_COEFFS);
                upperFlywheelMotor.enableVelocityMode(RobotParams.FLYWHEEL_MAX_VEL, RobotParams.SHOOTER_COEFFS);
            }
            else
            {
                lowerFlywheelMotor.disableVelocityMode();
                upperFlywheelMotor.disableVelocityMode();
            }
        }
    }   //setFlywheelVelocityModeEnabled

    /**
     * This method enables/disables flywheel velocity mode.
     *
     * @param enabled specifies true to enable velocity mode, false to disable.
     */
    public void setFlywheelVelocityModeEnabled(boolean enabled)
    {
        setFlywheelVelocityModeEnabled(null, enabled);
    }   //setFlywheelVelocityModeEnabled

    /**
     * This method checks if the flywheel is in velocity mode.
     *
     * @return true if flywheel is in velocity mode, false otherwise.
     */
    public boolean isFlywheelInVelocityMode()
    {
        return flywheelInVelocityMode;
    }   //isFlywheelInVelocityMode

    /**
     * This method returns the current power applied to the lower flywheel.
     *
     * @return lower flywheel motor power.
     */
    public double getLowerFlywheelPower()
    {
        return lowerFlywheelMotor.getMotorPower();
    }   //getLowerFlywheelPower

    /**
     * This method returns the current power applied to the upper flywheel.
     *
     * @return upper flywheel motor power.
     */
    public double getUpperFlywheelPower()
    {
        return upperFlywheelMotor.getMotorPower();
    }   //getUpperFlywheelPower

    /**
     * This method sets the flywheel power or velocity depending on if flywheel velocity mode is enabled.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     * @param lowerValue specifies the lower flywheel power in the range of -1.0 to 1.0 or velocity in RPM.
     * @param upperValue specifies the lower flywheel power in the range of -1.0 to 1.0 or velocity in RPM.
     * @param event specifies the event to signal when the upper flywheel is up to speed, can be null if not provided.
     *              This is only applicable if velocity mode is enabled.
     */
    public void setFlywheelValue(String owner, double lowerValue, double upperValue, TrcEvent event)
    {
        final String funcName = "setFlywheelValue";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(
                funcName, "owner=%s, lower=%.2f, upper=%.2f, event=%s", owner, lowerValue, upperValue, event);
        }

        if (validateOwnership(owner))
        {
            if (flywheelInVelocityMode)
            {
                // We only care about the upper flywheel velocity. This is the exit velocity of the ball.
                flywheelVelocityTrigger.setTrigger(
                    upperValue - RobotParams.FLYWHEEL_TOLERANCE, upperValue + RobotParams.FLYWHEEL_TOLERANCE,
                    RobotParams.FLYWHEEL_SETTLING_TIME);
                lowerValue *= RobotParams.FLYWHEEL_ENCODER_PPR / 60.0;
                upperValue *= RobotParams.FLYWHEEL_ENCODER_PPR / 60.0;
                flywheelToSpeedEvent = event;
            }

            if (lowerValue == 0.0)
            {
                // Stop the flywheels in a gentler way by using coast mode that is only applicable in PercentOutput
                // mode.
                lowerFlywheelMotor.stopMotor();
            }
            else
            {
                lowerFlywheelMotor.set(lowerValue);
            }

            if (upperValue == 0.0)
            {
                upperFlywheelMotor.stopMotor();
                if (flywheelInVelocityMode)
                {
                    flywheelVelocityTrigger.setEnabled(false);
                }
            }
            else
            {
                upperFlywheelMotor.set(upperValue);
                flywheelVelocityTrigger.setEnabled(true);
            }
        }
    }   //setFlywheelValue

    /**
     * This method sets the flywheel power or velocity depending on if flywheel velocity mode is enabled.
     *
     * @param lowerValue specifies the lower flywheel power in the range of -1.0 to 1.0 or velocity in RPM.
     * @param upperValue specifies the lower flywheel power in the range of -1.0 to 1.0 or velocity in RPM.
     * @param event specifies the event to signal when the upper flywheel is up to speed, can be null if not provided.
     *              This is only applicable if velocity mode is enabled.
     */
    public void setFlywheelValue(double lowerValue, double upperValue, TrcEvent event)
    {
        setFlywheelValue(null, lowerValue, upperValue, event);
    }   //setFlywheelValue

    /**
     * This method sets the flywheel power or velocity depending on if flywheel velocity mode is enabled.
     *
     * @param lowerValue specifies the lower flywheel power in the range of -1.0 to 1.0 or velocity in RPM.
     * @param upperValue specifies the lower flywheel power in the range of -1.0 to 1.0 or velocity in RPM.
     */
    public void setFlywheelValue(double lowerValue, double upperValue)
    {
        setFlywheelValue(null, lowerValue, upperValue, null);
    }   //setFlywheelValue

    /**
     * This method sets the flywheel power or velocity depending on if flywheel velocity mode is enabled.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     * @param value specifies the flywheel power in the range of -1.0 to 1.0 or velocity in RPM.
     * @param event specifies the event to signal when the flywheel is up to speed, can be null if not provided.
     *              This is only applicable if velocity mode is enabled.
     */
    public void setFlywheelValue(String owner, double value, TrcEvent event)
    {
        setFlywheelValue(owner, value, value*RobotParams.FLYWHEEL_UPPER2LOWER_VALUE_RATIO, event);
    }   //setFlywheelValue

    /**
     * This method sets the flywheel power or velocity depending on if flywheel velocity mode is enabled.
     *
     * @param value specifies the flywheel power in the range of -1.0 to 1.0 or velocity in RPM.
     * @param event specifies the event to signal when the flywheel is up to speed, can be null if not provided.
     *              This is only applicable if velocity mode is enabled.
     */
    public void setFlywheelValue(double value, TrcEvent event)
    {
        setFlywheelValue(null, value, event);
    }   //setFlywheelValue

    /**
     * This method sets the flywheel power or velocity depending on if flywheel velocity mode is enabled.
     *
     * @param value specifies the flywheel power in the range of -1.0 to 1.0 or velocity in RPM.
     */
    public void setFlywheelValue(double value)
    {
        setFlywheelValue(null, value, null);
    }   //setFlywheelValue

    /**
     * This method checks if the flywheel is spinning at target velocity.
     *
     * @return true if flywheel velocity is on target, false otherwise.
     */
    public boolean isFlywheelVelOnTarget()
    {
        return flywheelVelocityTrigger.getSensorState();
    }   //isFlywheelVelOnTarget

    /**
     * This method is called when the flywheel velocity trigger is activated (velocity reached) or deactivated.
     *
     * @param active  specifies true if the trigger is active, false if inactive.
     */
    private void flywheelTriggerEvent(boolean active)
    {
        final String funcName = "flywheelTriggerEvent";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "active=%s", active);
        }

        robot.ledIndicator.setFlywheelOnTarget(active);

        if (active && flywheelToSpeedEvent != null)
        {
            flywheelToSpeedEvent.signal();
            flywheelToSpeedEvent = null;
        }
    }   //flywheelTriggerEvent

    /**
     * This method returns the lower flywheel velocity in RPM.
     *
     * @return lower flywheel veloicty in RPM.
     */
    public double getLowerFlywheelVelocity()
    {
        return lowerFlywheelMotor.getVelocity() / RobotParams.FLYWHEEL_ENCODER_PPR /
               RobotParams.FLYWHEEL_GEAR_RATIO * 60.0;
    }   //getLowerFlywheelVelocity

    /**
     * This method returns the upper flywheel velocity in RPM.
     *
     * @return upper flywheel veloicty in RPM.
     */
    public double getUpperFlywheelVelocity()
    {
        return upperFlywheelMotor.getVelocity() / RobotParams.FLYWHEEL_ENCODER_PPR /
               RobotParams.FLYWHEEL_GEAR_RATIO * 60.0;
    }   //getUpperFlywheelVelocity

    //
    // Tilter methods.
    //

    /**
     * This method returns the tilter lower limit switch state.
     *
     * @return true if lower limit switch is active, false otherwise.
     */
    public boolean isTilterLowerLimitSwitchActive()
    {
        return tilterLowerLimitSwitch.isActive();
    }   //isTilterLowerLimitSwitchActive

    /**
     * This method enables/disables titler manual override mode.
     *
     * @param enable specifies true to enable manual override, false to disable.
     */
    public void setTilterManualOverride(boolean enable)
    {
        tilter.setManualOverride(enable);
    }   //setTilterManualOverride

    public void zeroCalibrateTilter()
    {
        tilter.zeroCalibrate(RobotParams.TILTER_CAL_POWER);
    }   //zeroCalibrateTilter

    /**
     * This method returns the current power applied to the tilter motor.
     *
     * @return tilter motor power.
     */
    public double getTilterPower()
    {
        return tilterMotor.getMotorPower();
    }   //getTilterPower

    /**
     * This method sets the tilter motor power.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     * @param power specifies the tilter motor power.
     */
    public void setTilterPower(String owner, double power)
    {
        final String funcName = "setTilterPower";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "owner=%s, power=%.1f", owner, power);
        }

        if (validateOwnership(owner))
        {
            tilter.setPower(power);
        }
    }   //setTilterPower

    /**
     * This method sets the tilter motor power.
     *
     * @param power specifies the tilter motor power.
     */
    public void setTilterPower(double power)
    {
        setTilterPower(null, power);
    }   //setTilterPower

    /**
     * This method returns the tilter position in degrees from horizontal.
     *
     * @return tilter position in degrees.
     */
    public double getTilterPosition()
    {
        return tilter.getPosition();
    }   //getTilterPosition

    /**
     * This method sets the tilter position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     * @param angle specifies the tilter angle in degrees from horizontal.
     * @param event specifies the event to notify when the tilter has reached target, null if not provided.
     */
    public void setTilterPosition(String owner, double angle, TrcEvent event)
    {
        final String funcName = "setTilterPosition";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "owner=%s, angle=%.1f, event=%s", owner, angle, event);
        }

        if (validateOwnership(owner))
        {
            tilter.setTarget(angle, false, event);
        }
    }   //setTilterPosition

    /**
     * This method sets the tilter position.
     *
     * @param angle specifies the tilter angle in degrees from horizontal.
     * @param event specifies the event to notify when the tilter has reached target, null if not provided.
     */
    public void setTilterPosition(double angle, TrcEvent event)
    {
        setTilterPosition(null, angle, event);
    }   //setTilterPosition

    /**
     * This method sets the tilter position.
     *
     * @param angle specifies the tilter angle in degrees from horizontal.
     */
    public void setTilterPosition(double angle)
    {
        setTilterPosition(null, angle, null);
    }   //setTilterPosition

    /**
     * This method is the worker preparing to shoot all balls. It is called by both shootAllBallsWithVision or
     * shootAllBallsNoVision.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    private boolean prepareToShoot(String owner, TrcEvent event)
    {
        final String funcName = "prepareToShoot";
        boolean success = false;

        // Acquire ownership of all subsystems involved. Don't need drivebase ownership if not using vision.
        if (this.acquireExclusiveAccess(owner) &&
            robot.conveyor.acquireExclusiveAccess(owner) &&
            (!usingVision || robot.robotDrive.driveBase.acquireExclusiveAccess(owner)))
        {
            currOwner = owner;
            onFinishShootingEvent = event;
            sm.start(State.START);
            shooterTaskObj.registerTask(TaskType.POSTPERIODIC_TASK);
        }

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "owner=%s, event=%s, success=%s", owner, event, success);
        }

        return success;
    }   //prepareToShoot

    /**
     * This method starts the auto shoot operation to shoot all balls in the conveyor using vision. It will use
     * vision info to align, aim and determine flywheel velocities before shooting.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootAllBallsWithVision(String owner, TrcEvent event)
    {
        usingVision = true;

        return prepareToShoot(owner, event);
    }   //shootAllBallsWithVision

    /**
     * This method starts the auto shoot operation to shoot all balls in the conveyor with no vision. It assumes the
     * robot is already aligned with the target. It will adjust the aim with the provided tilter angle and will spin
     * the flywheels with the given velocities.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @param lowerFlywheelVel specifies the lower flywheel velocity.
     * @param upperFlywheelVel
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootAllBallsNoVision(
        String owner, TrcEvent event, double lowerFlywheelVel, double upperFlywheelVel, double tilterAngle)
    {
        usingVision = false;
        this.lowerFlywheelSetVel = lowerFlywheelVel;
        this.upperFlywheelSetVel = upperFlywheelVel;
        this.tilterSetAngle = tilterAngle;

        return prepareToShoot(owner, event);
    }   //shootAllBallsWithVision

    /**
     * This method is called periodically to execute the auto shoot task.
     *
     * @param taskType specifies the task type.
     * @param runMode specifies the robot run mode.
     */
    private void autoShootTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            switch (state)
            {
                case START:
                    boolean ballAtEntrance = robot.conveyor.isEntranceSensorActive();
                    boolean ballAtExit = robot.conveyor.isExitSensorActive();
                    State nextState;

                    if (ballAtExit)
                    {
                        // Ball at the exit, shoot it.
                        nextState = State.SHOOT;
                    }
                    else if (ballAtEntrance)
                    {
                        // No ball at the exit but there is a ball at the entrance, advance it.
                        nextState = State.SHOOT;
                        robot.conveyor.advance(currOwner, conveyorEvent);
                        sm.addEvent(conveyorEvent);
                    }
                    else
                    {
                        // No more ball, we are done.
                        nextState = State.DONE;
                    }

                    if (nextState == State.SHOOT)
                    {
                        // Before we can shoot, we need to:
                        // - Spin the flywheel to the proper speed.
                        // - Aim the shooter at the target.
                        // - Align the robot to the target.
                        if (usingVision)
                        {
                            Double alignAngle = null, aimAngle = null;
                            double lowerFlywheelVel, upperFlywheelVel;
                            double robotX = robot.robotDrive.driveBase.getXPosition();
                            double robotY = robot.robotDrive.driveBase.getYPosition();
                            double distance = TrcUtil.magnitude(robotX, robotY);
                            double theta = 90.0 - Math.abs(Math.atan(robotY / robotX));

                            if (robot.vision != null && robot.vision.vision.get("tv") == 1.0)
                            {
                                // Vision detected target.
                                alignAngle = robot.vision.vision.get("tx");
                                aimAngle = robot.vision.vision.get("ty");
                            }

                            if (alignAngle == null || aimAngle == null)
                            {
                                // Vision did not find target, use robot odometry instead.
                                // //horizontal angle is in absolute degrees 
                                if (robotX > 0.0 && robotY > 0.0)
                                {
                                    // Quadrant 1
                                    alignAngle = 180 + theta;
                                }
                                else if (robotX < 0 && robotY > 0)
                                {
                                    // Quadrant 2
                                    alignAngle = 180 - theta;
                                }
                                else if (robotX < 0 && robotY < 0)
                                {
                                    // Quadrant 3
                                    alignAngle = theta;
                                }
                                else
                                {
                                    // Quadrant 4
                                    alignAngle = -theta;
                                }
                                aimAngle = interpolateVector(distance)[1];
                            }
                            lowerFlywheelVel = interpolateVector(distance)[0];
                            upperFlywheelVel = interpolateVector(distance)[1];

                            setFlywheelValue(currOwner, lowerFlywheelVel, upperFlywheelVel, flywheelEvent);
                            sm.addEvent(flywheelEvent);

                            setTilterPosition(currOwner, aimAngle, tilterEvent);
                            sm.addEvent(tilterEvent);

                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(
                                    robot.robotDrive.driveBase.getXPosition(),
                                    robot.robotDrive.driveBase.getYPosition(),
                                    alignAngle));
                            sm.addEvent(driveEvent);
                        }
                        else
                        {
                            // Not using vision or any other algorithms:
                            // - Spin the flywheel to the set speed.
                            // - Aim the shooter at the pre-determined angle.
                            // - Driver is responsible for aligning the robot possibly using streaming camera
                            //   or spotlight.
                            robot.shooter.setFlywheelValue(
                                currOwner, lowerFlywheelSetVel, upperFlywheelSetVel, flywheelEvent);
                            sm.addEvent(flywheelEvent);

                            setTilterPosition(tilterSetAngle, tilterEvent);
                            sm.addEvent(tilterEvent);
                        }
                        sm.waitForEvents(State.SHOOT, 0.0, true);
                    }
                    else
                    {
                        sm.setState(nextState);
                    }
                    break;

                case SHOOT:
                    robot.conveyor.advance(currOwner, conveyorEvent);
                    sm.waitForSingleEvent(conveyorEvent, State.START);
                    break;

                default: 
                case DONE:
                    cancel();
                    break; 
            }
        }

        if (debugEnabled)
        {
            robot.globalTracer.traceStateInfo(
                state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive, robot.robotDrive.purePursuitDrive,
                null);
        }
    }   //autoShootTask

    public static double[] interpolateVector(double distance)
    {
        double[] distances = new double[] { 76, 124, 180, 220, 280, 330 };   //these values from 2020, need new ones
        double[] velocities = new double[] { 464, 537, 720, 780, 820, 920 }; //these values from 2020, need new ones
        double[] angles = new double[] { 37, 30.5, 24.5, 25, 23, 21 };       //these values from 2020, need new ones
        for (int i = 0; i < distances.length - 1; i++)
        {
            if (distances[i] <= distance && distance <= distances[i + 1])
            {
                double w = (distance - distances[i]) / (distances[i + 1] - distances[i]);
                double v = (1 - w) * velocities[i] + w * velocities[i + 1];
                double angle = (1 - w) * angles[i] + w * angles[i + 1];
                double[] vector = {v, angle};
                return vector;
            }
        }
        double[] fail = {0.0, 0.0};
        return fail;
    }

}   //class Shooter
