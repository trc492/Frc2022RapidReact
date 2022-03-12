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

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcThresholdTrigger;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Shooter implements TrcExclusiveSubsystem
{
    private static final String moduleName  = "Shooter";
    private static final boolean debugEnabled = false;

    private final Robot robot;
    private final FrcCANFalcon lowerFlywheelMotor, upperFlywheelMotor;
    private final TrcThresholdTrigger flywheelVelocityTrigger;
    private final FrcPneumatic tilterPneumatic;
    // private final FrcCANTalon tilterMotor;
    // private final FrcDigitalInput tilterLowerLimitSwitch;
    // private final TrcPidActuator tilter;
    // private final TrcEvent tilterEvent;
    private final TrcEvent conveyorEvent;
    private final TrcEvent driveEvent;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject shooterTaskObj;

    private final ShootParamTable shootParamTable = new ShootParamTable()
        .add("preload", 1.0, 1900.0, 1700.0, RobotParams.TILTER_FAR_ANGLE)
        .add("tarmac_mid", 2.0, 2000, 1900, RobotParams.TILTER_FAR_ANGLE)
        .add("tarmac_auto", 3.0, 1900, 1700, RobotParams.TILTER_FAR_ANGLE);

    private boolean flywheelInVelocityMode = false;
    private TrcEvent flywheelToSpeedEvent = null;
    private String currOwner = null;

    private boolean usingVision = false;
    private ShootParamTable.Params shootParams = null;
    private boolean aimOnly = false;
    private TrcEvent onFinishShootingEvent = null;

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
        flywheelVelocityTrigger = new TrcThresholdTrigger(
            moduleName + ".flywheelVelTrigger", this::getUpperFlywheelVelocity, this::flywheelTriggerEvent);
        setFlywheelVelocityModeEnabled(true);
        //
        // Create and configure Tilter related objects.
        //
        tilterPneumatic = new FrcPneumatic(
            moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_TILTER_RETRACT, RobotParams.PNEUMATIC_TILTER_EXTEND);
        // tilterMotor = createTilterMotor(moduleName + ".tilterMotor", RobotParams.CANID_SHOOTER_TILTER);
        // tilterMotor.setPositionSensorInverted(true);
        // tilterLowerLimitSwitch = new FrcDigitalInput(
        //     moduleName + ".lowerLimitSwitch", RobotParams.DIO_TILTER_LOWER_LIMIT_SWITCH);
        // Parameters tilterParams = new Parameters()
        //     .setPidParams(
        //         new PidParameters(
        //             RobotParams.TILTER_KP, RobotParams.TILTER_KI, RobotParams.TILTER_KD, RobotParams.TILTER_TOLERANCE))
        //     .setPosRange(RobotParams.TILTER_MIN_POS, RobotParams.TILTER_MAX_POS)
        //     .setScaleOffset(RobotParams.TILTER_DEG_PER_COUNT, RobotParams.TILTER_OFFSET)
        //     .setZeroCalibratePower(RobotParams.TILTER_CAL_POWER);
        //     // .setStallProtectionParams(
        //     //     RobotParams.TILTER_STALL_MIN_POWER, RobotParams.TILTER_STALL_TOLERANCE,
        //     //     RobotParams.TILTER_STALL_TIMEOUT, RobotParams.TILTER_RESET_TIMEOUT);
        // tilter = new TrcPidActuator(moduleName + ".tilter", tilterMotor, tilterLowerLimitSwitch, null, tilterParams);

        //
        // Create and initialize other objects.
        //
        // tilterEvent = new TrcEvent(moduleName + ".tilterEvent");
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
        motor.motor.config_kP(0, RobotParams.FLYWHEEL_KP, 10);
        motor.motor.config_kI(0, RobotParams.FLYWHEEL_KI, 10);
        motor.motor.config_kD(0, RobotParams.FLYWHEEL_KD, 10);
        motor.motor.config_kF(0, RobotParams.FLYWHEEL_KF, 10);
        motor.motor.config_IntegralZone(0, RobotParams.FLYWHEEL_IZONE, 10);
        motor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);
        motor.setBrakeModeEnabled(false);
        motor.setInverted(inverted);

        return motor;
    }   //createFlywheelMotor

    // /**
    //  * This method creates and confiugres a tilter motor.
    //  *
    //  * @param name specifies the name of the motor.
    //  * @param canID specifies the CAN ID of the motor.
    //  */
    // private FrcCANTalon createTilterMotor(String name, int canID)
    // {
    //     FrcCANTalon motor = new FrcCANTalon(name, canID);

    //     motor.motor.configFactoryDefault();
    //     // We are going with software PID control, so disable motor PID for now.
    //     // motor.motor.config_kP(0, RobotParams.TILTER_KP, 10);
    //     // motor.motor.config_kI(0, RobotParams.TILTER_KI, 10);
    //     // motor.motor.config_kD(0, RobotParams.TILTER_KD, 10);
    //     // motor.motor.config_kF(0, RobotParams.TILTER_KF, 10);
    //     // motor.motor.configAllowableClosedloopError(0, RobotParams.TILTER_TOLERANCE/RobotParams.TILTER_DEG_PER_COUNT, 10);
    //     motor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
    //     motor.motor.enableVoltageCompensation(true);
    //     motor.setBrakeModeEnabled(true);
    //     motor.setInverted(RobotParams.TILTER_MOTOR_INVERTED);

    //     motor.motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    //     motor.motor.setSensorPhase(false);
    //     SensorCollection sensorCollection = motor.motor.getSensorCollection();
    //     sensorCollection.setPulseWidthPosition(0, 10); // reset index
    //     TrcUtil.sleep(50); // guarantee reset
    //     ErrorCode error = sensorCollection.syncQuadratureWithPulseWidth(0, 0, true, RobotParams.TILTER_ZERO, 10);
    //     if (error != ErrorCode.OK)
    //     {
    //         robot.globalTracer.traceErr(moduleName, "Failed to configure encoder (error=%s).", error.name());
    //     }
    //     TrcUtil.sleep(50); // guarantee reset

    //     robot.globalTracer.traceInfo(
    //         moduleName, "Tilter: zero=%d, pwmPos=%d, quadPos=%d, selectedPos=%f",
    //         RobotParams.TILTER_ZERO, sensorCollection.getPulseWidthPosition(),
    //         sensorCollection.getQuadraturePosition(), motor.motor.getSelectedSensorPosition());

    //     return motor;
    // }   //createTilterMotor

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
                // Convert values from RPM back to the native unit of encoder count per second.
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
                // Stop the flywheels in a gentler way by using coast mode that is only applicable in PercentOutput
                // mode.
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

    // CodeReview: What is this for?
    public void setFlywheelPower(double power)
    {
        lowerFlywheelMotor.setMotorPower(power);
    }

    /**
     * This method checks if the flywheel is spinning at target velocity.
     *
     * @return true if flywheel velocity is on target, false otherwise.
     */
    public boolean isFlywheelVelOnTarget()
    {
        return flywheelVelocityTrigger.getState();
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
     * This method returns the state of the tilter pneumatic.
     *
     * @return true if the tilter is at FAR position, false if at CLOSE.
     */
    public boolean isTilterAtFarPosition()
    {
        return tilterPneumatic.isExtended();
    }   //isTilterAtFarPosition

    public double getTilterPosition()
    {
        return isTilterAtFarPosition()? RobotParams.TILTER_FAR_ANGLE: RobotParams.TILTER_CLOSE_ANGLE;
    }   //getTilterPosition

    /**
     * This method sets the tilter position to shoot close.
     */
    public void setTilterPositionClose()
    {
        tilterPneumatic.retract();
    }   //setTilterPositionClose

    /**
     * This method sets the tilter position to shoot far.
     */
    public void setTilterPositionFar()
    {
        tilterPneumatic.extend();
    }   //setTilterPositionFar

    /**
     * This method sets the tilter position to either CLOSE or FAR. Therefore, the position value must be one of the
     * two preset angles. If the caller is giving us a position other than these two, we will assume FAR.
     *
     * @param position specifies the tilter position angle in degrees.
     */
    public void setTilterPosition(double position)
    {
        if (position == RobotParams.TILTER_CLOSE_ANGLE)
        {
            setTilterPositionClose();
        }
        else
        {
            // If position is any random number, assume FAR.
            setTilterPositionFar();
        }
    }   //setTilterPosition

    // /**
    //  * This method returns the tilter lower limit switch state.
    //  *
    //  * @return true if lower limit switch is active, false otherwise.
    //  */
    // public boolean isTilterLowerLimitSwitchActive()
    // {
    //     return tilterLowerLimitSwitch.isActive();
    // }   //isTilterLowerLimitSwitchActive

    // /**
    //  * This method enables/disables titler manual override mode.
    //  *
    //  * @param enable specifies true to enable manual override, false to disable.
    //  */
    // public void setTilterManualOverride(boolean enable)
    // {
    //     tilter.setManualOverride(enable);
    // }   //setTilterManualOverride

    // public void zeroCalibrateTilter()
    // {
    //     tilter.zeroCalibrate(RobotParams.TILTER_CAL_POWER);
    // }   //zeroCalibrateTilter

    // /**
    //  * This method returns the current power applied to the tilter motor.
    //  *
    //  * @return tilter motor power.
    //  */
    // public double getTilterPower()
    // {
    //     return tilterMotor.getMotorPower();
    // }   //getTilterPower

    // /**
    //  * This method sets the tilter motor power.
    //  *
    //  * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
    //  *              require exclusive access.
    //  * @param power specifies the tilter motor power.
    //  */
    // public void setTilterPower(String owner, double power)
    // {
    //     final String funcName = "setTilterPower";

    //     if (debugEnabled)
    //     {
    //         robot.globalTracer.traceInfo(funcName, "owner=%s, power=%.1f", owner, power);
    //     }

    //     if (validateOwnership(owner))
    //     {
    //         tilter.setPower(power);
    //     }
    // }   //setTilterPower

    // /**
    //  * This method sets the tilter motor power.
    //  *
    //  * @param power specifies the tilter motor power.
    //  */
    // public void setTilterPower(double power)
    // {
    //     setTilterPower(null, power);
    // }   //setTilterPower

    // /**
    //  * This method returns the tilter position in degrees from horizontal.
    //  *
    //  * @return tilter position in degrees.
    //  */
    // public double getTilterPosition()
    // {
    //     return tilter.getPosition();
    // }   //getTilterPosition

    // /**
    //  * This method sets the tilter position.
    //  *
    //  * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
    //  *              require exclusive access.
    //  * @param angle specifies the tilter angle in degrees from horizontal.
    //  * @param event specifies the event to notify when the tilter has reached target, null if not provided.
    //  */
    // public void setTilterPosition(String owner, double angle, TrcEvent event)
    // {
    //     final String funcName = "setTilterPosition";

    //     if (debugEnabled)
    //     {
    //         robot.globalTracer.traceInfo(funcName, "owner=%s, angle=%.1f, event=%s", owner, angle, event);
    //     }

    //     if (validateOwnership(owner))
    //     {
    //         tilter.setTarget(angle, false, event);
    //     }
    // }   //setTilterPosition

    // /**
    //  * This method sets the tilter position.
    //  *
    //  * @param angle specifies the tilter angle in degrees from horizontal.
    //  * @param event specifies the event to notify when the tilter has reached target, null if not provided.
    //  */
    // public void setTilterPosition(double angle, TrcEvent event)
    // {
    //     setTilterPosition(null, angle, event);
    // }   //setTilterPosition

    // /**
    //  * This method sets the tilter position.
    //  *
    //  * @param angle specifies the tilter angle in degrees from horizontal.
    //  */
    // public void setTilterPosition(double angle)
    // {
    //     setTilterPosition(null, angle, null);
    // }   //setTilterPosition

    //
    // Auto-Assist Shooting.
    //

    private enum State
    {
        START,
        PREP_TO_SHOOT,
        SHOOT_WHEN_READY,
        DONE
    }   //enum State

    /**
     * This method is called to cancel any pending operations and stop the subsystem. It is typically called before
     * exiting a competition mode.
     */
    public void cancel()
    {
        final String funcName = "cancel";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "Canceling: currOwner=%s", currOwner);
        }

        setFlywheelValue(currOwner, 0.0, 0.0, null);
        // tilter.cancel(currOwner);
        robot.conveyor.setPower(currOwner, 0.0, 0.0, 0.0, null);
        robot.robotDrive.setAntiDefenseEnabled(currOwner, false);
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
     * This method starts the auto shoot operation to shoot all balls in the conveyor using full vision. It will use
     * vision info to align the robot and use odometry info for looking up shoot parameters in the preset table to
     * aim and determine flywheel velocities for the shooting.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @param aimOnly specifies true if aiming target only and no shooting, false to also shoot.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootAllBallsFullVision(String owner, TrcEvent event, boolean aimOnly)
    {
        usingVision = true;
        shootParams = null;
        this.aimOnly = aimOnly;
        return prepareToShoot(owner, event);
    }   //shootAllBallsFullVision

    /**
     * This method starts the auto shoot operation to shoot all balls in the conveyor using vision only for alignment.
     * The caller will provide the preset table entry for the rest of the shooting parameters.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @param shootParams specifies the shoot parameters for flywheel velocities and tilter angle.
     * @param aimOnly specifies true if aiming target only and no shooting, false to also shoot.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootAllBallsVisionAlignment(
        String owner, TrcEvent event, ShootParamTable.Params shootParams, boolean aimOnly)
    {
        if (shootParams != null)
        {
            usingVision = true;
            this.shootParams = shootParams;
            this.aimOnly = aimOnly;
            return prepareToShoot(owner, event);
        }

        return false;
    }   //shootAllBallsVisionAlignment

    /**
     * This method starts the auto shoot operation to shoot all balls in the conveyor using vision only for alignment.
     * The caller will provide the preset table entry for the rest of the shooting parameters.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @param presetName specifies name of the preset table entry for shoot parameters.
     * @param aimOnly specifies true if aiming target only and no shooting, false to also shoot.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootAllBallsVisionAlignment(String owner, TrcEvent event, String presetName, boolean aimOnly)
    {
        return shootAllBallsVisionAlignment(owner, event, shootParamTable.get(presetName), aimOnly);
    }   //shootAllBallsVisionAlignment

    /**
     * This method starts the auto shoot operation to shoot all balls in the conveyor with no vision. It assumes the
     * robot is already aligned with the target. It will adjust the aim and spin the flywheels with the provided shoot
     * parameters.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @param shootParams specifies the shooter parameters for shooting.
     * @param aimOnly specifies true if aiming target only and no shooting, false to also shoot.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootAllBallsNoVision(
        String owner, TrcEvent event, ShootParamTable.Params shootParams, boolean aimOnly)
    {
        if (shootParams == null)
        {
            throw new IllegalArgumentException("shootAllBallsNoVision must provide shootParams.");
        }

        usingVision = false;
        this.shootParams = shootParams;
        this.aimOnly = aimOnly;
        return prepareToShoot(owner, event);
    }   //shootAllBallsNoVision

    /**
     * This method starts the auto shoot operation to shoot all balls in the conveyor with no vision. It assumes the
     * robot is already aligned with the target. It will adjust the aim and spin the flywheels with the provided shoot
     * parameters.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @param presetName specifies name of the preset table entry for shoot parameters.
     * @param aimOnly specifies true if aiming target only and no shooting, false to also shoot.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootAllBallsNoVision(String owner, TrcEvent event, String presetName, boolean aimOnly)
    {
        return shootAllBallsNoVision(owner, event, shootParamTable.get(presetName), aimOnly);
    }   //shootAllBallsNoVision

    /**
     * This method assumes the robot is aligned with the target, the shooter has aimed at the target and the flywheels
     * have been started. Once the flywheel is up to speed, it will shoot all balls in the conveyor. This is intended
     * to be called by TeleOp responding to the shoot trigger. It assumes a separate button has been pressed that did
     * a shootAllBallsVisionAlignment which resulted in the robot aligning and aiming at the target already.
     *
     * @param owner specifies the owner ID who is shooting.
     */
    public void shootAllBallsWhenReady(String owner)
    {
        if (validateOwnership(owner))
        {
            sm.start(State.SHOOT_WHEN_READY);
        }
    }   //shootAllBallsWhenReady

    /**
     * This method is called periodically to execute the auto shoot task.
     *
     * @param taskType specifies the task type.
     * @param runMode specifies the robot run mode.
     */
    private void autoShootTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "autoShootTask";
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            switch (state)
            {
                case START:
                    boolean ballAtEntrance = robot.conveyor.isEntranceSensorActive();
                    boolean ballAtExit = robot.conveyor.isExitSensorActive();
                    State nextState;

                    if (debugEnabled)
                    {
                        robot.globalTracer.traceInfo(funcName, "Entrance=%s, Exit=%s", ballAtEntrance, ballAtExit);
                    }

                    if (ballAtExit)
                    {
                        // Ball at the exit, shoot it.
                        nextState = State.PREP_TO_SHOOT;
                    }
                    else if (ballAtEntrance)
                    {
                        // No ball at the exit but there is a ball at the entrance, advance it.
                        nextState = State.PREP_TO_SHOOT;
                        robot.conveyor.advance(currOwner, conveyorEvent);
                        sm.addEvent(conveyorEvent);
                    }
                    else
                    {
                        // No more ball, we are done.
                        nextState = State.DONE;
                    }

                    if (nextState == State.PREP_TO_SHOOT)
                    {
                        // Before we can shoot, we need to:
                        // - Spin the flywheel to the proper speed.
                        // - Aim the shooter at the target.
                        // - Align the robot to the target.
                        if (usingVision)
                        {
                            Double alignAngle = null, aimAngle = null;
                            double distance;
                            double robotX = robot.robotDrive.driveBase.getXPosition();
                            double robotY = robot.robotDrive.driveBase.getYPosition();

                            if (robot.vision != null && robot.vision.vision.get("tv") == 1.0)
                            {
                                // Vision detected target.
                                alignAngle = robot.vision.vision.get("tx");
                                aimAngle = robot.vision.vision.get("ty");
                                if (debugEnabled)
                                {
                                    robot.globalTracer.traceInfo(
                                        funcName, "Vision: alignAngle=%.1f, aimAngle=%.1f", alignAngle, aimAngle);
                                }
                            }

                            if (shootParams != null)
                            {
                                // If shootParams are provided, use the provided distance and aimAngle instead because
                                // vision doesn't provide distance and the vision aimAngle doesn't account real world
                                // factors so the calibrated aimAngle in the preset table is more accurate.
                                // We still use the alignAngle from vision because that should be reliable.
                                distance = shootParams.distance;
                                aimAngle = shootParams.tilterAngle;

                                if (debugEnabled)
                                {
                                    robot.globalTracer.traceInfo(funcName, "ProvidedParams: %s", shootParams);
                                }
                            }
                            else
                            {
                                distance = TrcUtil.magnitude(robotX, robotY);
                                shootParams = shootParamTable.get(distance);
                                if (aimAngle == null)
                                {
                                    // Vision did not detect target, use the angle from the interpolated shoot params.
                                    aimAngle = shootParams.tilterAngle;
                                }
                                else
                                {
                                    aimAngle = aimAngle > RobotParams.TILTER_ANGLE_THRESHOLD?
                                        RobotParams.TILTER_CLOSE_ANGLE: RobotParams.TILTER_FAR_ANGLE;
                                }

                                if (debugEnabled)
                                {
                                    robot.globalTracer.traceInfo(funcName, "InterpolatedParams: %s", shootParams);
                                }
                            }

                            if (alignAngle == null)
                            {
                                double theta = 90.0 - Math.abs(Math.atan(robotY / robotX));
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
                            }

                            if (debugEnabled)
                            {
                                robot.globalTracer.traceInfo(
                                    funcName, "OdometryAlignment: robotX=%.1f, robotY=%.1f, align=%.1f",
                                    robotX, robotY, alignAngle);
                            }
                            // Don't need to wait for flywheel here. SHOOT_WHEN_READY will wait for it.
                            setFlywheelValue(
                                currOwner, shootParams.lowerFlywheelVelocity, shootParams.upperFlywheelVelocity,
                                null);

                            // Pneumatic takes hardly any time, so fire and forget.
                            setTilterPosition(aimAngle);
                            // setTilterPosition(currOwner, aimAngle, tilterEvent);
                            // sm.addEvent(tilterEvent);

                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(
                                    robot.robotDrive.driveBase.getXPosition(),
                                    robot.robotDrive.driveBase.getYPosition(),
                                    alignAngle));

                            sm.waitForSingleEvent(driveEvent, nextState);
                        }
                        else
                        {
                            // Not using vision or any other algorithms:
                            // - Spin the flywheel to the set speed.
                            // - Aim the shooter at the pre-determined angle.
                            // - Driver is responsible for aligning the robot possibly using streaming camera
                            //   or spotlight.
                            if (debugEnabled)
                            {
                                robot.globalTracer.traceInfo(funcName, "NoVisionShootParams: %s", shootParams);
                            }
                            // Don't need to wait for flywheel here. SHOOT_WHEN_READY will wait for it.
                            setFlywheelValue(
                                currOwner, shootParams.lowerFlywheelVelocity, shootParams.upperFlywheelVelocity,
                                null);

                            // Pneumatic takes hardly any time, so fire and forget.
                            setTilterPosition(shootParams.tilterAngle);
                            // setTilterPosition(shootParams.tilterAngle, tilterEvent);
                            // sm.addEvent(tilterEvent);
                            // sm.waitForSingleEvent(tilterEvent, nextState);

                            sm.setState(nextState);
                        }
                    }
                    else
                    {
                        sm.setState(nextState);
                    }
                    break;

                case PREP_TO_SHOOT:
                    robot.robotDrive.setAntiDefenseEnabled(currOwner, true);
                    if (aimOnly)
                    {
                        // This must be from TeleOp. Done for now and wait for the operator to press the shoot
                        // trigger which will continue on to the SHOOT_WHEN_READY state. Just stop the state
                        // machine but holding onto the exclusive locks and keeping the flywheels running.
                        sm.stop();
                    }
                    else
                    {
                        sm.setState(State.SHOOT_WHEN_READY);
                    }
                    break;

                case SHOOT_WHEN_READY:
                    ballAtEntrance = robot.conveyor.isEntranceSensorActive();
                    ballAtExit = robot.conveyor.isExitSensorActive();

                    if (ballAtExit)
                    {
                        // Ball at the exit, shoot it.
                        if (isFlywheelVelOnTarget())
                        {
                            robot.conveyor.advance(currOwner, conveyorEvent);
                            sm.waitForSingleEvent(conveyorEvent, State.SHOOT_WHEN_READY);
                        }
                    }
                    else if (ballAtEntrance)
                    {
                        // No ball at the exit but there is a ball at the entrance, advance it.
                        robot.conveyor.advance(currOwner, conveyorEvent);
                        sm.waitForSingleEvent(conveyorEvent, State.SHOOT_WHEN_READY);
                    }
                    else
                    {
                        sm.waitForEvents(State.DONE);
                    }
                    break;

                case DONE:
                default:
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

}   //class Shooter
