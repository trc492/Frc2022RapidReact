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

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.TitlePaneLayout;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import org.apache.commons.math3.linear.RealVector;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidMotor;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcPidActuator.Parameters;
import TrcCommonLib.trclib.TrcPidController.PidCoefficients;
import TrcCommonLib.trclib.TrcPidController.PidParameters;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcCANTalonLimitSwitch;
import TrcFrcLib.frclib.FrcRemoteVisionProcessor.RelativePose;
import edu.wpi.first.math.trajectory.Trajectory;

public class Shooter implements TrcExclusiveSubsystem
{
    private enum State{
        SHIFT_BALL_IN_CONVEYOR, PREP_SHOOTER_BALL_1, PREP_SHOOTER_BALL_2, SHOOT, DONE
    }
    public final FrcCANFalcon lowerFlywheelMotor, upperFlywheelMotor;
    public final FrcCANTalon tilterMotor;
    // public final FrcCANTalonLimitSwitch tilterUpperLimitSwitch, tilterLowerLimitSwitch;
    public final Parameters tilterParams;
    public final TrcPidActuator tilter;
    public boolean flyWheelInVelocityMode = false;
    public TrcEvent onFinishEvent = null; 
    public TrcEvent event;
    public TrcEvent event2;
    public TrcEvent event3;
    public Robot robot; 
    
    //autoshooter stuff
    //owner is the name of the thing using autoAssist shooter, ie a button 
    public static final String owner = "auto shooter"; 
    public int totalBallsToShoot; 
    public TrcStateMachine<State> sm; 
    State state; 
    public String instanceName  = "Shooter"; 
    public TrcTaskMgr.TaskObject shooterTask; 
    //vision stuff from limelight 
    RelativePose targetPos; 
    double tx; 
    double ty;
    // tilter stuff 
    //tiltAngle = ty+compensation(from the lookup table )
    double tiltAngle;
    double distanceToTarget; 
    //ideal flywheel velocity  
    double targetFlywheelVelocity;//based on lookup table values

    public Shooter(Robot robot)
    {
        this.robot = robot; 
        lowerFlywheelMotor = new FrcCANFalcon("lowerFlywheelMotor", RobotParams.CANID_SHOOTER_LOWER_FLYWHEEL);
        lowerFlywheelMotor.setInverted(true);
        lowerFlywheelMotor.setBrakeModeEnabled(false);
        upperFlywheelMotor = new FrcCANFalcon("upperFlywheelMotor", RobotParams.CANID_SHOOTER_UPPER_FLYWHEEL);
        upperFlywheelMotor.setInverted(true);
        upperFlywheelMotor.setBrakeModeEnabled(false);
        setFlywheelVelocityModeEnabled(true);
        
        tilterMotor = new FrcCANTalon("tilterMotor", RobotParams.CANID_SHOOTER_TILTER);
        tilterMotor.motor.configFactoryDefault();
        tilterMotor.motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        tilterMotor.motor.setSensorPhase(true);
        tilterMotor.motor.getSensorCollection().setPulseWidthPosition(0, 10); // reset index
        TrcUtil.sleep(50); // guarantee reset
        ErrorCode error = tilterMotor.motor.getSensorCollection().syncQuadratureWithPulseWidth(0, 0, true, 1582, 10);
        if (error != ErrorCode.OK)
        {
            System.out.printf("Encoder error! - Shooter Tilter, error=%s\n", error.name());
        }
        TrcUtil.sleep(50); // guarantee reset
        int modPos = (int) TrcUtil.modulo(tilterMotor.motor.getSelectedSensorPosition(), 4096);
        int pos = modPos > 2048 ? modPos - 4096 : modPos;
        tilterMotor.motor.setSelectedSensorPosition(pos, 0, 10);
        TrcUtil.sleep(50);

        System.out.printf("TilterMotor: Zero=%d, PwmPos=%d, quadPos=%d, selectedPos=%d\n", 1582,
            tilterMotor.motor.getSensorCollection().getPulseWidthPosition(),
            tilterMotor.motor.getSensorCollection().getQuadraturePosition(),
            tilterMotor.motor.getSelectedSensorPosition());

        // tilterLowerLimitSwitch = new FrcCANTalonLimitSwitch("tilterLowerLimitSwitch", tilterMotor, false);
        // tilterUpperLimitSwitch = new FrcCANTalonLimitSwitch("tilterUpperLimitSwitch", tilterMotor, true);
        tilterParams = new Parameters().setPidParams(new PidParameters(RobotParams.TILTER_KP, RobotParams.TILTER_KI, RobotParams.TILTER_KD, RobotParams.TILTER_KF, RobotParams.TILTER_TOLERANCE));
        tilter = new TrcPidActuator("tilter", tilterMotor, null, null, tilterParams);
        event = new TrcEvent("event");
        //shooter task things
        shooterTask = TrcTaskMgr.createTask(instanceName + ".shooterTask", this::aimAndShoot);
        sm = new TrcStateMachine<>(instanceName);
    }
    //assume the target is in view of limelight before this is called, at least one ball in the robot 
    //this method shoots all the balls in the robot
    //boolean tells you if it succeeded in starting the auto shooter
    public boolean shootAllBalls (TrcEvent event){
        boolean succeeded = true; 
        //assume method in conveyor for get number of balls 
        // totalBallsToShoot = robot.conveyor.getNumBalls();
        this.onFinishEvent = event; 
        if(robot.conveyor.isEntranceSensorActive()){
            totalBallsToShoot++;
        }
        if(robot.conveyor.isExitSensorActive()){
            totalBallsToShoot++;
        }
        if(totalBallsToShoot>0){
            if(robot.shooter.acquireExclusiveAccess(owner)&&robot.conveyor.acquireExclusiveAccess(owner)
                &&robot.intake.acquireExclusiveAccess(owner)&&robot.robotDrive.driveBase.acquireExclusiveAccess(owner)){
                sm.start(State.SHIFT_BALL_IN_CONVEYOR);
                shooterTask.registerTask(TaskType.POSTPERIODIC_TASK);
            }
            else{
                succeeded = false; 

            }
        }
        else{
            succeeded = false; 
        }
        return succeeded; 

    }
    //the auto aim task 
    public void aimAndShoot(TrcTaskMgr.TaskType taskMgr, TrcRobot.RunMode runmode){
        state = sm.checkReadyAndGetState();
        if(state!=null){
            switch(state){
                case SHIFT_BALL_IN_CONVEYOR:
                    // if there is no ball at the exit sensor advance the ball to there 
                    if(!robot.conveyor.isExitSensorActive()){
                        robot.conveyor.advance(instanceName, event);
                        sm.waitForSingleEvent(event, State.PREP_SHOOTER_BALL_1 );
                    }
                    //otherwise go to the aim state
                    else{
                        sm.setState(State.PREP_SHOOTER_BALL_1);
                    }
                break; 
                //this state prepares the shooter for shooting the first ball 
                case PREP_SHOOTER_BALL_1:
                    //rotate robot tx degrees based on vision
                    //use vision.getLastPose() to find distance to target 
                    //prepare shooter to be ready to shoot(tilt and flywheel velocity )
                    if(RobotParams.Preferences.useVision&&robot.vision.vision.targetDetected()){
                        targetPos = robot.vision.getLastPose();
                        tx = robot.vision.vision.get("tx");
                        ty = robot.vision.vision.get("ty");
                        distanceToTarget = TrcUtil.magnitude(targetPos.x, targetPos.y);
                        //turn the robot tx degrees
                        robot.robotDrive.purePursuitDrive.start(event, robot.robotDrive.driveBase.getFieldPosition(), true, new TrcPose2D(0, 0, tx));
                        //Kenny, write method to  calculate flywheel Velocity and tilt angle, save to fields targetFlywheelVelocity and tiltAngle
                        robot.shooter.setFlywheelVelocity(targetFlywheelVelocity, event2);
                        robot.shooter.tilter.setTarget(tiltAngle, false, event3);
                        //one event for turning robot, one event for turning tilter, one event for getting flywheels up to speed 
                        sm.addEvent(event);
                        sm.addEvent(event2);
                        sm.addEvent(event3);
                        sm.waitForEvents(State.SHOOT, 0.0, true);
                    }
                    else{
                        robot.shooter.setFlywheelVelocity(2000, event);
                        sm.waitForSingleEvent(event, State.SHOOT);
                        //sm.setState(State.DONE);     
                    }
                break; 
                //this state is for preparing to shoot the second ball(no longer need to turn robot or tilter )
                case PREP_SHOOTER_BALL_2: 
                    //because we just shot a ball from the exit sensor, need to advance the other ball to the exit sensor in case 
                    //the conveyor did not advance the second ball to the exit sensor while it shot the first ball 
                    if(!robot.conveyor.isExitSensorActive()){
                        robot.conveyor.advance(instanceName, event);
                        sm.addEvent(event);
    
                    }
                    //spin flywheels up to speed
                    robot.shooter.setFlywheelVelocity(targetFlywheelVelocity, event2);
                    sm.addEvent(event2);
                    sm.waitForEvents(State.SHOOT, 0.0, true);
                break; 
                case SHOOT:
                    //this sends ball to flywheels, shooting the ball 
                    robot.conveyor.advance(owner, event);
                    totalBallsToShoot--;
                    if(totalBallsToShoot>0){
                        sm.waitForSingleEvent(event, State.PREP_SHOOTER_BALL_1);
                    }
                    else{
                        sm.setState(State.DONE);
                    }
                break; 
                case DONE:
                    cancel();
                break; 
            }
        
        }
    }
    public void cancel(){
        //after shooting do the following:
        //set flywheel velocity to 0 
        //notify auto shooter event
        //clear all values associated with auto shooter
        robot.shooter.setFlywheelPower(0.0);
        shooterTask.unregisterTask();
        sm.stop();
        this.targetFlywheelVelocity = 0.0;
        this.tiltAngle = 0.0; 
        this.tx = 0.0; 
        this.ty = 0.0;
        this.distanceToTarget = 0.0; 
        this.onFinishEvent.notify(); 

    }

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
                
                
    public void setFlywheelVelocityModeEnabled(String owner, boolean enabled)
    {
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
    }


    public void setFlywheelVelocityModeEnabled(boolean enabled)
    {
        setFlywheelVelocityModeEnabled(null, enabled);
    }

    public void setFlywheelPower(String owner, double lowerPower, double upperPower)
    {
        if (validateOwnership(owner))
        {
            lowerFlywheelMotor.set(lowerPower);
            upperFlywheelMotor.set(upperPower);
        }
    }

    public void setFlywheelPower(double lowerPower, double upperPower)
    {
        setFlywheelPower(null, lowerPower, upperPower);
    }

    public void setFlywheelPower(String owner, double power)
    {
        if (validateOwnership(owner))
        {
            lowerFlywheelMotor.set(power);
            upperFlywheelMotor.set(power);
        }
    }

    public void setFlywheelPower(double power)
    {
        setFlywheelPower(null, power);
    }

    /**
     * Set the flywheel velocity in RPM
     * @param upperVelocity
     * @param lowerVelocity
     * @param event
     */
    public void setFlywheelVelocity(double lowerVelocity, double upperVelocity, TrcEvent event) {
        lowerFlywheelMotor.set(lowerVelocity/RobotParams.FLYWHEEL_MAX_RPM);
        upperFlywheelMotor.set(upperVelocity/RobotParams.FLYWHEEL_MAX_RPM, 0.0, event);
    }

    public void setFlywheelVelocity(double lowerVelocity, double upperVelocity) {
        setFlywheelVelocity(lowerVelocity, upperVelocity, null);
    }

    public void setFlywheelVelocity(double velocity, TrcEvent event){
        setFlywheelVelocity(velocity, velocity, event);
    }

    public void setFlywheelVelocity(double velocity) {
        setFlywheelVelocity(velocity, velocity);
    }

    /**
     * @return In theory, will return lower flywheel RPM
     */
    public double getLowerFlywheelVelocity()
    {
        return lowerFlywheelMotor.getVelocity() / RobotParams.FLYWHEEL_COUNTS_PER_REVOLUTION / RobotParams.FLYWHEEL_GEAR_RATIO * 60.0;
    }

    /**
     * @return In theory, will return upper flywheel RPM
     */
    public double getUpperFlywheelVelocity()
    {
        return upperFlywheelMotor.getVelocity() / RobotParams.FLYWHEEL_COUNTS_PER_REVOLUTION / RobotParams.FLYWHEEL_GEAR_RATIO * 60.0;
    }

    public void setTilterPower(String owner, double power)
    {
        if (validateOwnership(owner))
        {
            tilter.setPower(power);
        }
    }

    public void setTilterPower(double power)
    {
        setTilterPower(null, power);
    }

    public double getTilterPosition()
    {
        return tilter.getPosition();
    }

    public void setTilterPosition(String owner, double pos, TrcEvent event)
    {
        if (validateOwnership(owner))
        {
            tilter.setTarget(pos, false, event);
        }
    }

    public void setTilterPosition(double pos, TrcEvent event)
    {
        setTilterPosition(null, pos, event);

    }
    public void setTilterPosition(double pos)
    {
        setTilterPosition(null, pos, null);
    }

    // public void shoot(String owner, TrcEvent event){

    // }
    // //assumes there is already a ball at the exit sensor 
    // public void shoot(String owner){
    //     if(validateOwnership(owner)){
    //     //Start spinning flywheels
    //     //If flywheels are spinning fast enough(based on this.idealShooterVelocity)
    //     //Tell conveyor to shoot ball from the exitSensor
    //     }   
    // }
    // public void shoot()
    // {
    //     shoot(null);
    // }
    // //this method returns the ideal velocity of the shooter calculated for the first ball that can be used for the second ball 





}   //class Shooter
