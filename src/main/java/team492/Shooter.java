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

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcPidActuator;
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

public class Shooter implements TrcExclusiveSubsystem
{
    private static final String moduleName  = "Shooter";
    private static final boolean debugEnabled = false;

    private enum State
    {
        START,
        SHOOT,
        DONE,

        START_NO_VISION,
        SHOOT_NO_VISION
    }   //enum State

    private final Robot robot;
    private final FrcCANFalcon lowerFlywheelMotor, upperFlywheelMotor;
    private final TrcValueSensorTrigger flywheelVelocityTrigger;
    private final FrcCANTalon tilterMotor;
    private final TrcPidActuator tilter;
    private final TrcEvent flywheelEvent;
    private final TrcEvent tilterEvent;
    private final TrcEvent conveyorEvent;
    private final TrcEvent driveEvent;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject shooterTaskObj;

    private boolean flyWheelInVelocityMode = false;
    private TrcEvent onFinishEvent = null;
    private String currOwner = null;

    //autoshooter stuff
    //owner is the name of the thing using autoAssist shooter, ie a button 

    //vision stuff from limelight 
    // RelativePose targetPos; 
    // double tx; 
    // double ty;
    // // tilter stuff 
    // //tiltAngle = ty+compensation(from the lookup table )
    // double tiltAngle;
    // double distanceToTarget; 
    // //ideal flywheel velocity  
    // double targetFlywheelVelocity;//based on lookup table values

    // Double expireTime; 

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
        Parameters tilterParams = new Parameters()
            .setMotorParams(
                RobotParams.TILTER_MOTOR_INVERTED,
                RobotParams.TILTER_HAS_LOWERLIMIT_SWITCH, false,
                RobotParams.TILTER_HAS_UPPERLIMIT_SWITCH, false,
                0.0)
            .setPidParams(
                new PidParameters(RobotParams.TILTER_KP, RobotParams.TILTER_KI, RobotParams.TILTER_KD,
                                  RobotParams.TILTER_TOLERANCE))
            .setPosRange(RobotParams.TILTER_MIN_POS, RobotParams.TILTER_MAX_POS)
            .setScaleOffset(RobotParams.TILTER_DEG_PER_COUNT, 0.0);
        tilter = new TrcPidActuator(moduleName + ".tilter", tilterMotor, null, null, tilterParams);
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

        motor.motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        motor.motor.setSensorPhase(true);
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
        //after shooting do the following:
        //set flywheel velocity to 0 
        //notify auto shooter event
        //clear all values associated with auto shooter
        // robot.dashboard.displayPrintf(3, "CAncel cancel cancel ");
        // Stop flywheels.
        // Stop Intake.
        // Stop Conveyor.
        // Stop DriveBase.
        // this.targetFlywheelVelocity = 0.0;
        // this.tiltAngle = 0.0; 
        // this.tx = 0.0; 
        // this.ty = 0.0;
        // this.distanceToTarget = 0.0; 
        // robot.dashboard.displayPrintf(4, "");
        // robot.dashboard.displayPrintf(5, "");
        // robot.dashboard.displayPrintf(12, "");

        if (onFinishEvent != null)
        {
            onFinishEvent.signal();
            onFinishEvent = null;
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
            flyWheelInVelocityMode = enabled;
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
        return flyWheelInVelocityMode;
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
            if (flyWheelInVelocityMode)
            {
                // adjust values to percent max RPM.
                lowerValue /= RobotParams.FLYWHEEL_MAX_RPM;
                upperValue /= RobotParams.FLYWHEEL_MAX_RPM;
                if (event != null)
                {
                    // We only care about the upper flywheel velocity. This is the exit velocity of the ball.
                    onFinishEvent = event;
                    flywheelVelocityTrigger.setTrigger(
                        upperValue - RobotParams.FLYWHEEL_TOLERANCE, upperValue + RobotParams.FLYWHEEL_TOLERANCE,
                        RobotParams.FLYWHEEL_SETTLING_TIME);
                    flywheelVelocityTrigger.setEnabled(true);
                }
            }
            lowerFlywheelMotor.set(lowerValue);
            upperFlywheelMotor.set(upperValue);
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

        if (active && flywheelEvent != null)
        {
            flywheelVelocityTrigger.setEnabled(false);
            flywheelEvent.signal();
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
     * This method starts the auto shoot operation to shoot all balls in the conveyor.
     *
     * @param owner specifies the owner ID who is shooting.
     * @param event specifies the events to signal when completed, can be null if not provided.
     * @return true if the operation was started successfully, false otherwise (could not acquire exclusive ownership
     *         of the involved subsystems).
     */
    public boolean shootAllBalls(String owner, TrcEvent event)
    {
        final String funcName = "shootAllBalls";
        boolean success = false;

        //assume the target is in view of limelight before this is called, at least one ball in the robot 
        //this method shoots all the balls in the robot
        //boolean tells you if it succeeded in starting the auto shooter
        //bug where if ball isnt triggering exit sensor the thing doesnt work 
        if (this.acquireExclusiveAccess(owner) &&
            robot.conveyor.acquireExclusiveAccess(owner) &&
            robot.robotDrive.driveBase.acquireExclusiveAccess(owner))
        {
            currOwner = owner;
            sm.start(State.START_NO_VISION);
            shooterTaskObj.registerTask(TaskType.POSTPERIODIC_TASK);
        }

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "owner=%s, event=%s, success=%s", owner, event, success);
        }

        return success;
    }   //shootAllBalls

    /**
     * This method is called periodically to execute the auto shoot task.
     *
     * @param taskType specifies the task type.
     * @param runMode specifies the robot run mode.
     */
    private void autoShootTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();
        boolean ballAtEntrance; 
        boolean ballAtExit; 
        if (state != null)
        {
            switch (state)
            {
                case START_NO_VISION:
                    ballAtEntrance = robot.conveyor.isExitSensorActive();
                    ballAtExit = robot.conveyor.isExitSensorActive();      
                    if(ballAtEntrance || ballAtExit){
                        flyWheelInVelocityMode = true; 
                        robot.shooter.setFlywheelValue(currOwner, 1300, 3200, flywheelEvent);
                        sm.addEvent(flywheelEvent);
                        if(!ballAtExit){
                            robot.conveyor.advance(currOwner, conveyorEvent); 
                            sm.addEvent(conveyorEvent);
                        }
                        sm.waitForEvents(State.SHOOT_NO_VISION, 0.0, true);
                    }
                    else{
                        sm.setState(State.DONE);
                    }

                break; 
                case SHOOT_NO_VISION:
                    robot.conveyor.advance(currOwner, conveyorEvent);
                    sm.waitForSingleEvent(conveyorEvent, State.START_NO_VISION);
                break; 
                case START:
                    ballAtEntrance = robot.conveyor.isExitSensorActive();
                    ballAtExit = robot.conveyor.isExitSensorActive();

                    if (ballAtEntrance || ballAtExit)
                    {
                        Double horizontalAngle = null, verticalAngle = null;
                        double lowerFlywheelVelocity = 0.0, upperFlywheelVelocity = 0.0;

                        if (robot.vision != null && robot.vision.vision.get("tv") == 1.0)
                        {
                            // Vision detected target.
                            horizontalAngle = robot.vision.vision.get("tx");
                            verticalAngle = robot.vision.vision.get("ty");  // may need to add adjustment from lookup table
                            // lowerFlywheelVelocity = ;
                            // upperFlywheelVelocity = ;
                        }

                        if (horizontalAngle == null || verticalAngle == null)
                        {
                            // Vision did not find target, use robot odometry instead.
                            // Create a vector from the robot back to field origin to find horizontal angle
                            // and use distance to field origin to find vertical angle.
                            // horizontalAngle = 
                            // verticalAngle = 
                            // lowerFlywheelVelocity =
                            // upperFlywheelVelocity = 
                        }

                        // robot.robotDrive.purePursuitDrive.start(
                        //     currOwner, driveEvent, robot.robotDrive.driveBase.getFieldPosition(), true,
                        //     new TrcPose2D(0.0, 0.0, horizontalAngle));
                        sm.addEvent(driveEvent);

                        setTilterPosition(currOwner, verticalAngle, tilterEvent);
                        sm.addEvent(tilterEvent);

                        setFlywheelValue(currOwner, lowerFlywheelVelocity, upperFlywheelVelocity, flywheelEvent);
                        sm.addEvent(flywheelEvent);

                        if (!ballAtExit)
                        {
                            robot.conveyor.advance(currOwner, conveyorEvent);
                            sm.addEvent(conveyorEvent);
                        }

                        sm.waitForEvents(State.SHOOT, 0.0, true);
                    }
                    else
                    {
                        sm.setState(State.DONE);
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
    
                // case SHIFT_BALL_IN_CONVEYOR:
                //     // if there is no ball at the exit sensor advance the ball to there 
                //     if(!robot.conveyor.isExitSensorActive()){
                //         robot.conveyor.advance(owner, event);
                //         sm.waitForSingleEvent(event, State.PREP_SHOOTER_BALL_1 );
                //     }
                //     //otherwise go to the aim state
                //     else{
                //         sm.setState(State.PREP_SHOOTER_BALL_1);
                //     }
                // break; 
                // //this state prepares the shooter for shooting the first ball 
                // case PREP_SHOOTER_BALL_1:
                //     //rotate robot tx degrees based on vision
                //     //use vision.getLastPose() to find distance to target 
                //     //prepare shooter to be ready to shoot(tilt and flywheel velocity )
                //     if(RobotParams.Preferences.useVision&&robot.vision.vision.targetDetected()){
                //         targetPos = robot.vision.getLastPose();
                //         tx = robot.vision.vision.get("tx");
                //         ty = robot.vision.vision.get("ty");
                //         distanceToTarget = TrcUtil.magnitude(targetPos.x, targetPos.y);
                //         //turn the robot tx degrees
                //         robot.robotDrive.purePursuitDrive.start(event, robot.robotDrive.driveBase.getFieldPosition(), true, new TrcPose2D(0, 0, tx));
                //         //Kenny, write method to  calculate flywheel Velocity and tilt angle, save to fields targetFlywheelVelocity and tiltAngle
                //         robot.shooter.setFlywheelVelocity(targetFlywheelVelocity, event2);
                //         robot.shooter.tilter.setTarget(tiltAngle, false, event3);
                //         //one event for turning robot, one event for turning tilter, one event for getting flywheels up to speed 
                //         sm.addEvent(event);
                //         sm.addEvent(event2);
                //         sm.addEvent(event3);
                //         sm.waitForEvents(State.SHOOT, 0.0, true);
                //     }
                //     else{
                //         if(expireTime==null){

                //             expireTime= TrcUtil.getCurrentTime()+2;
                //             robot.shooter.setFlywheelVelocity(2750, 2250);
                //         }
                //         else{
                //             if(TrcUtil.getCurrentTime()>=expireTime){
                //                 expireTime = null; 
                //                 sm.setState(State.SHOOT);
                //             }
                //         }
                //         // robot.shooter.setFlywheelVelocity(2750, 2250, event);
                //         // sm.waitForSingleEvent(event, State.SHOOT);
                //         //timer.set(2.0, event);
                //         //sm.waitForSingleEvent(event, State.SHOOT);

                //         //sm.setState(State.DONE);     
                //     }
                // break; 
                //this method never gets called 
                //this state is for preparing to shoot the second ball(no longer need to turn robot or tilter )
                // case PREP_SHOOTER_BALL_2: 
                //     //because we just shot a ball from the exit sensor, need to advance the other ball to the exit sensor in case 
                //     //the conveyor did not advance the second ball to the exit sensor while it shot the first ball 
                //     // if(!robot.conveyor.isExitSensorActive()){
                //     //     robot.conveyor.advance(instanceName, event);
                //     //     sm.addEvent(event);
    
                //     // }
                //     //spin flywheels up to speed
                //     //robot.shooter.setFlywheelVelocity(targetFlywheelVelocity-100, event);
                //     robot.shooter.setFlywheelVelocity(targetFlywheelVelocity-100, 2250, event);
                //     //sm.addEvent(event);
                //     sm.waitForSingleEvent(event, State.SHOOT);
                //     break; 
                // case SHOOT:
                //     //this sends ball to flywheels, shooting the ball
                //     //robot.conveyor.advance(owner, event);// TODO DEBUG THE EVENT 
                //     robot.conveyor.advance(owner);
                //     numBallsToShoot--;
                //     if(expireTime==null){
                //         expireTime = TrcUtil.getCurrentTime() + 5; 
                //     }
                //     else {
                //         if(TrcUtil.getCurrentTime()>=expireTime){
                //             if(numBallsToShoot>0){
                //                 expireTime = null; 

                //                 sm.setState(State.PREP_SHOOTER_BALL_2);
                //             }
                //             else{
                //                 robot.dashboard.displayPrintf(14, "DONE");

                //                 sm.setState(State.DONE);
                //             }
                //         }
                //     }
                    // robot.dashboard.displayPrintf(2, "Shooting");
                    // sm.waitForSingleEvent(event, State.DONE);
                    // totalBallsToShoot--;
                    // robot.dashboard.displayPrintf(1, "Cancel, %d", totalBallsToShoot);  
                    // if(totalBallsToShoot>0){
                    //     sm.waitForSingleEvent(event, State.PREP_SHOOTER_BALL_1);
                    // }
                    // else{

                    //     sm.waitForSingleEvent(event, State.DONE);
                    // }
                    // break;
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
