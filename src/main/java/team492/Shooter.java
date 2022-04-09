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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcThresholdTrigger;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import team492.ShootParamTable.ShootLoc;

public class Shooter implements TrcExclusiveSubsystem
{
    private static final String moduleName  = "Shooter";

    private final Robot robot;
    private final FrcCANFalcon lowerFlywheelMotor, upperFlywheelMotor;
    private final TrcThresholdTrigger flywheelVelocityTrigger;
    private final FrcPneumatic tilterPneumatic;
    private final TrcEvent conveyorEvent;
    private final TrcEvent timerEvent;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final TrcPidController alignPidCtrl;
    private final TrcTaskMgr.TaskObject shooterTaskObj;

    private TrcDbgTrace msgTracer = null;
    private boolean flywheelInVelocityMode = false;
    private TrcEvent flywheelToSpeedEvent = null;
    private double nextFlywheelUpdateTime = 0.0;

    private String currOwner = null;
    private boolean visionAlignEnabled = true;
    private boolean committedToShoot = false;
    private boolean allowToShoot = false;
    private boolean ballShot = false;
    private Double targetAngle = null;
    private Double targetDistance = null;

    private boolean usingVision = false;
    private boolean useOdometry = false;
    private ShootParamTable.Params providedParams = null;
    private TrcEvent onFinishPrepEvent = null;
    private TrcEvent onFinishShootEvent = null;

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
        setTilterPositionFar();
        //
        // Create and initialize other objects.
        //
        conveyorEvent = new TrcEvent(moduleName + ".conveyorEvent");
        timerEvent = new TrcEvent(moduleName + ".timerEvent");
        timer = new TrcTimer(moduleName + ".timer");
        sm = new TrcStateMachine<>(moduleName);

        TrcPidController.PidCoefficients alignPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.GYRO_ALIGN_KP, RobotParams.GYRO_ALIGN_KI, RobotParams.GYRO_ALIGN_KD);
        alignPidCtrl = new TrcPidController(
            "aligPidCtrl", alignPidCoeff, RobotParams.GYRO_ALIGN_TOLERANCE, RobotParams.GYRO_ALIGN_SETTLING_TIME,
            RobotParams.GYRO_ALIGN_STEADY_STATE_ERROR, RobotParams.GYRO_ALIGN_ERRRATE_THRESHOLD,
            this::getTargetAngle);
        alignPidCtrl.setAbsoluteSetPoint(true);
        alignPidCtrl.setInverted(true);
        alignPidCtrl.setTarget(0.0);

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

    /**
     * This method enables/disables tracing for the shooter subsystem.
     *
     * @param tracer specifies the tracer to use for logging events.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        msgTracer = tracer;
    }   //setMsgTracer

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

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "enabled=%s, flywheelMaxVel=%d", enabled, RobotParams.FLYWHEEL_MAX_RPM);
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
     * This method stops both flywheels.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     */
    public void stopFlywheel(String owner)
    {
        if (validateOwnership(owner))
        {
            lowerFlywheelMotor.stopMotor();
            upperFlywheelMotor.stopMotor();
        }
    }   //stopFlywheel

    /**
     * This method stops both flywheels.
     */
    public void stopFlywheel()
    {
        stopFlywheel(null);
    }   //stopFlywheel

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

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] owner=%s, lower=%.2f, upper=%.2f, event=%s",
                TrcUtil.getModeElapsedTime(), owner, lowerValue, upperValue, event);
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

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] active=%s, lowerVel=%.0f, upperVel=%.0f",
                TrcUtil.getModeElapsedTime(), active, getLowerFlywheelVelocity(), getUpperFlywheelVelocity());
        }

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

    /**
     * This method returns the current tilter position in degrees.
     *
     * @return tilter poosition in degrees.
     */
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

    //
    // Auto-Assist Shooting.
    //

    private enum State
    {
        START,
        PREP_TO_SHOOT,
        SHOOT_WHEN_READY,
        WAIT_FOR_SHOT_COMPLETE,
        DONE
    }   //enum State

    /**
     * This method enables/disables auto vision alignment.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setVisionAlignEnabled(boolean enabled)
    {
        visionAlignEnabled = enabled;
    }   //setVisionAlignEnabled

    /**
     * This method checks if vision align is enabled.
     *
     * @return true if vision align is enabled, false if disabled.
     */
    public boolean isVisionAlignEnabled()
    {
        return visionAlignEnabled;
    }   //isVisionAlignedEnabled

    /**
     * This method enables/disables NoOscillation for the align PID controller.
     *
     * @param noOscillation specifies true to enable no oscillation, false otherwise.
     */
    public void setVisionAlignNoOscillation(boolean noOscillation)
    {
        alignPidCtrl.setNoOscillation(noOscillation);
    }   //setVisionAlignNoOscillation

    /**
     * This method allows/disallows the use of robot odometry to estimate target distance and angle.
     *
     * @param enabled specifies true to allow and false to disallow.
     */
    public void allowUseOdometry(boolean enabled)
    {
        useOdometry = enabled;
    }   //allowUseOdometry

    /**
     * This method is called to cancel any pending operations and stop the subsystems. It is typically called before
     * exiting the competition mode.
     */
    public void cancel()
    {
        final String funcName = "cancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "[%.3f] Canceling: currOwner=%s", TrcUtil.getModeElapsedTime(), currOwner);
        }

        // Don't stop the flywheels, we still want flywheels to be spinning to save time from spinning down and up
        // again for subsequent shooting. 
        robot.conveyor.cancel(currOwner);
        // robot.robotDrive.setAntiDefenseEnabled(currOwner, false);

        if (currOwner != null)
        {
            this.releaseExclusiveAccess(currOwner);
            robot.conveyor.releaseExclusiveAccess(currOwner);
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }

        if (onFinishPrepEvent != null)
        {
            onFinishPrepEvent.signal();
            onFinishPrepEvent = null;
        }

        if (onFinishShootEvent != null)
        {
            onFinishShootEvent.signal();
            onFinishShootEvent = null;
        }

        alignPidCtrl.reset();
        committedToShoot = false;
        allowToShoot = false;
        usingVision = false;

        robot.ledIndicator.setTargetAligned(false);
        robot.ledIndicator.setTargetInView(false);

        sm.stop();
        shooterTaskObj.unregisterTask();
    }   //cancel

    /**
     * This method shuts down the shooter subsystem by canceling any pending auto-shooting operation and turning off
     * the flywheels.
     */
    public void shutdown()
    {
        cancel();
        stopFlywheel();
    }   //shutdown

    /**
     * This method is the worker preparing to shoot all balls. It is called by both shootWithVision or
     * shootWithNoVision.
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

        // Acquire ownership of all subsystems involved.
        if (this.acquireExclusiveAccess(owner) &&
            robot.conveyor.acquireExclusiveAccess(owner) &&
            robot.robotDrive.driveBase.acquireExclusiveAccess(owner))
        {
            currOwner = owner;
            onFinishShootEvent = event;
            committedToShoot = false;
            sm.start(State.START);
            shooterTaskObj.registerTask(TaskType.POSTCONTINUOUS_TASK);
            success = true;
        }

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] owner=%s, event=%s, success=%s (providedParams=%s)",
                TrcUtil.getModeElapsedTime(), owner, event, success, providedParams);
            if (!success)
            {
                TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
                msgTracer.traceInfo(
                    funcName, "shooterOwern=%s, conveyorOwner=%s, driveBaseOwner=%s",
                    ownershipMgr.getOwner(this), ownershipMgr.getOwner(robot.conveyor),
                    ownershipMgr.getOwner(robot.robotDrive.driveBase));
            }
        }

        return success;
    }   //prepareToShoot

    /**
     * This method starts the auto shoot operation to shoot all balls in the conveyor using vision. The caller can
     * optionally provide shootParams to augment vision such as in TeleOp for shooting from preset spots. shootParams
     * can be null when called from autonomous where it will use vision info to determine shootParams.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @param shootParams specifies the shoot parameters for flywheel velocities and tilter angle, can be null if not
     *        provided (i.e. in autonomous).
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootWithVision(String owner, TrcEvent event, ShootParamTable.Params shootParams)
    {
        usingVision = true;
        this.providedParams = shootParams;
        return prepareToShoot(owner, event);
    }   //shootWithVision

    /**
     * This method starts the auto shoot operation to shoot all balls in the conveyor using full vision. It means it
     * does not require any ShootParams provided by the caller. It will use vision to determine target distance and
     * will interpolate the ShootParams from the ShootParamTable.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootWithVision(String owner, TrcEvent event)
    {
        return shootWithVision(owner, event, (ShootParamTable.Params) null);
    }   //shootWithVision

    /**
     * This method starts the auto shoot operation to shoot all balls in the conveyor using vision only for alignment.
     * The caller will provide the preset location entry for the rest of the shooting parameters.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @param presetLoc specifies shoot location of the preset table entry for shoot parameters.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootWithVision(String owner, TrcEvent event, ShootLoc presetLoc)
    {
        ShootParamTable.Params params = presetLoc != null? robot.shootParamTable.get(presetLoc): null;

        if (params == null)
        {
            throw new IllegalArgumentException(
                "presetLoc must not be null and must specify an entry in the ShootParamTable.");
        }

        return shootWithVision(owner, event, params);
    }   //shootWithVision

    /**
     * This method starts the auto shoot operation to shoot all balls in the conveyor without vision. The caller must
     * provide shootParams.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @param shootParams specifies the shoot parameters for flywheel velocities and tilter angle.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootWithNoVision(String owner, TrcEvent event, ShootParamTable.Params shootParams)
    {
        if (shootParams == null)
        {
            throw new IllegalArgumentException("prepareToShootNoVision must provide shootParams.");
        }

        usingVision = false;
        this.providedParams = shootParams;
        return prepareToShoot(owner, event);
    }   //shootWithNoVision

    /**
     * This method starts the auto shoot operation to shoot all balls in the conveyor without vision. The caller must
     * provide shootParams.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @param presetLoc specifies shoot location of the preset table entry for shoot parameters.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootWithNoVision(String owner, TrcEvent event, ShootLoc presetLoc)
    {
        return shootWithNoVision(owner, event, presetLoc != null? robot.shootParamTable.get(presetLoc): null);
    }   //shootWithNoVision

    /**
     * This method is intended to be called by teleop to commit to shoot all balls in the conveyor. This assumes
     * a previous call to shootWithVision or shootWithNoVision has been made which will prep the shooter to aim
     * at the target. This method is allowed to proceed only if the previous call to shootWithVision or
     * shootWithNoVision signals the aiming is complete.
     *
     * @param owner specifies the owner ID who is shooting.
     */
    public void commitToShoot(String owner)
    {
        final String funcName = "commitToShoot";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] owner=%s, allowToShoot=%s, currOwner=%s",
                TrcUtil.getModeElapsedTime(), owner, allowToShoot, currOwner);
        }

        if ((robot.isTeleop() || robot.isTest()) &&
            allowToShoot && currOwner != null && validateOwnership(owner))
        {
            committedToShoot = true;
            allowToShoot = false;
        }
    }   //commitToShoot

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
            double matchTime = TrcUtil.getModeElapsedTime();
            ShootParamTable.Params params = null;
            boolean traceState = true;

            switch (state)
            {
                case START:
                    boolean ballAtEntrance = robot.conveyor.isEntranceSensorActive();
                    boolean ballAtExit = robot.conveyor.isExitSensorActive();

                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(
                            funcName, "[%.3f] Entrance=%s, Exit=%s", matchTime, ballAtEntrance, ballAtExit);
                    }

                    if (ballAtExit)
                    {
                        // Ball at the exit, shoot it.
                        sm.setState(State.PREP_TO_SHOOT);
                    }
                    else if (ballAtEntrance)
                    {
                        // No ball at the exit but there is a ball at the entrance, advance it.
                        sm.waitForSingleEvent(conveyorEvent, State.PREP_TO_SHOOT);
                        robot.conveyor.advance(currOwner, conveyorEvent);
                    }
                    else
                    {
                        // No more ball, we are done.
                        sm.setState(State.DONE);
                    }
                    break;

                case PREP_TO_SHOOT:
                    //
                    // Before we can shoot, we need to:
                    // - Spin the flywheels to the proper velocities.
                    // - Aim the shooter at the target.
                    // - Align the robot to the target.
                    //
                    // Flywheel velocities and shooter angle can be obtained from vision and interpolated from
                    // ShootParam table or provided by the caller in the form of a ShootParam table entry.
                    // Alignment angle can only be obtained by vision. If there is no vision, the caller is
                    // responsible for aligning the robot. In the scenario where vision is enabled but vision
                    // failed to find the target and the caller did not provide shoot params, we will attempt to
                    // calculate the params from robot odometry location.
                    //
                    double xPower = 0.0, yPower = 0.0, rotPower = 0.0;
                    boolean visionPidOnTarget;

                    targetAngle = null;
                    targetDistance = null;

                    // Use vision to determine shoot parameters.
                    if (usingVision && robot.vision != null)
                    {
                        boolean targetInView = robot.vision.targetAcquired();

                        if (targetInView)
                        {
                            targetAngle = robot.vision.getTargetHorizontalAngle();
                            targetDistance = robot.vision.getTargetDistance() + RobotParams.VISION_TARGET_RADIUS;

                            if (msgTracer != null)
                            {
                                msgTracer.traceInfo(
                                    funcName, "[%.3f] Vision: targetAngle=%.2f, targetDistance=%.2f",
                                    matchTime, targetAngle, targetDistance);
                            }
                        }

                        robot.ledIndicator.setTargetInView(targetInView);
                    }

                    if (targetAngle == null && useOdometry)
                    {
                        // Either we are not using vision or vision did not detect target, use robot odometry instead.
                        double robotX = robot.robotDrive.driveBase.getXPosition();
                        double robotY = robot.robotDrive.driveBase.getYPosition();

                        targetAngle = robot.getAlignAngleFromOdometry(robotX, robotY) -
                                      robot.robotDrive.driveBase.getHeading();
                        targetDistance = TrcUtil.magnitude(robotX, robotY);

                        if (msgTracer != null)
                        {
                            msgTracer.traceInfo(
                                funcName, "[%.3f] Odometry: targetAngle=%.2f, targetDistance=%.2f",
                                matchTime, targetAngle, targetDistance);
                        }
                    }

                    if (providedParams != null)
                    {
                        // Caller provided shootParams, let's use it.
                        params = providedParams;
                    }
                    else if (targetDistance != null)
                    {
                        // Caller did not provide shootParams (e.g. full vision), use distance to lookup ShootParam
                        // table to interpolate/extrapolate shootParams.
                        params = robot.shootParamTable.get(targetDistance);
                    }

                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(
                            funcName, "[%.3f] using %s ShootParams: %s",
                            matchTime, providedParams != null? "provided": "interpolated", params);
                    }

                    double currTime = TrcUtil.getCurrentTime();
                    if (params != null && currTime > nextFlywheelUpdateTime)
                    {
                        // To reduce CAN traffic, we only update flywheel at a slower update interval.
                        nextFlywheelUpdateTime = currTime + RobotParams.FLYWHEEL_UPDATE_INTERVAL;
                        // Apply shoot parameters to flywheels only. Not setting tilter position here,
                        // otherwise if the robot is at the inflection point, the tilter will oscillate
                        // between close and far quickly, rapidly decreasing pneumatic pressure.
                        // Don't need to wait for flywheel here. SHOOT_WHEN_READY will wait for it.
                        setFlywheelValue(
                            currOwner, params.lowerFlywheelVelocity, params.upperFlywheelVelocity, null);
                    }

                    // Fetch driver inputs if we are in teleOp or Test mode.
                    if (robot.isTeleop() || robot.isTest())
                    {
                        // In Teleop, we allow joystick control to drive the robot around before shooting.
                        // The joystick can control X and Y driving but vision is controlling the heading.
                        // Therefore, the robot is always aiming at the vision target. However, in case
                        // vision was wrong, we also allow the driver to override vision by controlling turn
                        // using joystick.
                        // Note: we are driving with slower speed while using vision.
                        double[] inputs = robot.robotDrive.getDriveInputs();
                        xPower = inputs[0]*0.3;
                        yPower = inputs[1]*0.3;
                        rotPower = inputs[2]*0.3;
                    }

                    if (visionAlignEnabled && rotPower == 0.0)
                    {
                        // Vision alignment is enabled and driver is not overriding.
                        rotPower = alignPidCtrl.getOutput();
                        visionPidOnTarget = alignPidCtrl.isOnTarget();

                        if (msgTracer != null)
                        {
                            msgTracer.traceInfo(
                                funcName, "[%.3f] visionPidOnTarget=%s, rotPower=%.2f",
                                matchTime, visionPidOnTarget, rotPower);
                            alignPidCtrl.printPidInfo(msgTracer);
                        }

                        robot.ledIndicator.setTargetAligned(visionPidOnTarget);
                    }
                    else
                    {
                        visionPidOnTarget = false;
                    }

                    if (!robot.robotDrive.isAntiDefenseEnabled())
                    {
                        robot.robotDrive.driveBase.holonomicDrive(currOwner, xPower, yPower, rotPower);
                    }

                    if (RobotParams.Preferences.debugShooter)
                    {
                        robot.dashboard.displayPrintf(
                            11, "x=%.1f, y=%.1f, rot=%.1f, onTarget=%s", xPower, yPower, rotPower, visionPidOnTarget);
                    }
                    //
                    // In Teleop or test mode, if visionAlign is disabled, let operator decide when to shoot. This
                    // allows the operator to shoot even if vision has problem detecting target by disabling
                    // visionAlign. Otherwise, we always rely on vision to shoot but the operator can decide when
                    // to commit shooting (by releasing the trigger).
                    //
                    double upperFlywheelPower = getUpperFlywheelPower();
                    if (upperFlywheelPower > 0.0)
                    {
                        // Flywheels must be spinning to allow shooting or the shooter will jam.
                        boolean goShoot = false;

                        if ((robot.isTeleop() || robot.isTest()) && !committedToShoot)
                        {
                            // Wait for trigger release to commit shooting.
                            allowToShoot = true;
                        }
                        else if (visionPidOnTarget)
                        {
                            if (robot.isAutonomous() || committedToShoot)
                            {
                                goShoot = true;
                            }
                        }
                        else if (!robot.isAutonomous() && !visionAlignEnabled && committedToShoot)
                        {
                            // Target is not aligned but operator is doing manual override, shoot anyway.
                            goShoot = true;
                        }

                        if (msgTracer != null)
                        {
                            msgTracer.traceInfo(
                                funcName,
                                "[%.3f] upperFlywheelPower=%.1f, allowToShoot=%s, committedToShoot=%s, goShoot=%s",
                                matchTime, upperFlywheelPower, allowToShoot, committedToShoot, goShoot);
                        }

                        if (goShoot)
                        {
                            robot.robotDrive.driveBase.stop(currOwner);
                            // robot.robotDrive.setAntiDefenseEnabled(currOwner, true);

                            if (params != null)
                            {
                                // Pneumatic takes hardly any time, so fire and forget.
                                setTilterPosition(params.tilterAngle);
                                setFlywheelValue(
                                    currOwner, params.lowerFlywheelVelocity, params.upperFlywheelVelocity, null);
                                if (msgTracer != null)
                                {
                                    msgTracer.traceInfo(funcName, "[%.3f] final ShootParams: %s", matchTime, params);
                                }
                            }
                            ballShot = false;
                            sm.setState(State.SHOOT_WHEN_READY);
                        }
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
                            sm.waitForSingleEvent(conveyorEvent, State.SHOOT_WHEN_READY);
                            robot.conveyor.advance(currOwner, conveyorEvent);
                            ballShot = true;
                            if (msgTracer != null)
                            {
                                msgTracer.traceInfo(
                                    funcName, "[%.3f] Shot a ball (lowerFlywheel=%.0f, upperFlywheel=%.0f).",
                                    matchTime, getLowerFlywheelVelocity(), getUpperFlywheelVelocity());
                            }
                        }
                        else
                        {
                            traceState = false;
                        }
                    }
                    else if (ballAtEntrance)
                    {
                        // No ball at the exit but there is a ball at the entrance, advance it.
                        sm.waitForSingleEvent(conveyorEvent, State.SHOOT_WHEN_READY);
                        robot.conveyor.advance(currOwner, conveyorEvent);
                        if (msgTracer != null)
                        {
                            msgTracer.traceInfo(funcName, "[%.3f] Advance ball to exit.", matchTime);
                        }
                    }
                    else
                    {
                        sm.setState(ballShot? State.WAIT_FOR_SHOT_COMPLETE: State.DONE);
                    }
                    break;

                case WAIT_FOR_SHOT_COMPLETE:
                    sm.waitForSingleEvent(timerEvent, State.DONE);
                    timer.set(RobotParams.SHOT_COMPLETION_DELAY, timerEvent);
                    break;

                case DONE:
                default:
                    cancel();
                    break; 
            }

            if (msgTracer != null && traceState)
            {
                msgTracer.traceStateInfo(
                    sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
            }
        }
    }   //autoShootTask

    /**
     * This method is called by alignPidCtrl to get the vision target heading for aligning the robot to the target.
     *
     * @return target angle relative to the robot's heading.
     */
    private double getTargetAngle()
    {
        return targetAngle != null? targetAngle: 0.0;
    }   //getTargetAngle

}   //class Shooter
