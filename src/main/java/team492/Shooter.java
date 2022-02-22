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

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidMotor;
import TrcCommonLib.trclib.TrcPidActuator.Parameters;
import TrcCommonLib.trclib.TrcPidController.PidCoefficients;
import TrcCommonLib.trclib.TrcPidController.PidParameters;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcCANTalonLimitSwitch;

public class Shooter implements TrcExclusiveSubsystem
{
    public final FrcCANFalcon lowerFlywheelMotor, upperFlywheelMotor;
    public final FrcCANTalon tilterMotor;
    public final FrcCANTalonLimitSwitch tilterUpperLimitSwitch, tilterLowerLimitSwitch;
    public final Parameters tilterParams;
    public final TrcPidActuator tilter;
    public boolean flyWheelInVelocityMode = false;
    public double idealShooterVelocity; 
    public TrcEvent onFinishEvent = null; 

    public Shooter()
    {
        lowerFlywheelMotor = new FrcCANFalcon("lowerFlywheelMotor", RobotParams.CANID_SHOOTER_LOWER_FLYWHEEL);
        lowerFlywheelMotor.setInverted(true);
        lowerFlywheelMotor.setBrakeModeEnabled(false);
        upperFlywheelMotor = new FrcCANFalcon("upperFlywheelMotor", RobotParams.CANID_SHOOTER_UPPER_FLYWHEEL);
        upperFlywheelMotor.setInverted(true);
        upperFlywheelMotor.setBrakeModeEnabled(false);
        setFlywheelVelocityModeEnabled(false);
        
        tilterMotor = new FrcCANTalon("tilterMotor", RobotParams.CANID_SHOOTER_TILTER);
        tilterLowerLimitSwitch = new FrcCANTalonLimitSwitch("tilterLowerLimitSwitch", tilterMotor, false);
        tilterUpperLimitSwitch = new FrcCANTalonLimitSwitch("tilterUpperLimitSwitch", tilterMotor, true);
        tilterParams = new Parameters().setPidParams(new PidParameters(RobotParams.TILTER_KP, RobotParams.TILTER_KI, RobotParams.TILTER_KD, RobotParams.TILTER_KF, RobotParams.TILTER_TOLERANCE));
        tilter = new TrcPidActuator("tilter", tilterMotor, tilterLowerLimitSwitch, tilterUpperLimitSwitch, tilterParams);
    }

    public void setFlywheelVelocityModeEnabled(String owner, boolean enabled)
    {
        if (validateOwnership(owner))
        {
            flyWheelInVelocityMode = enabled;
            if (enabled)
            {
                lowerFlywheelMotor.enableVelocityMode(RobotParams.SHOOTER_FLYWHEEL_MAX_VEL, new PidCoefficients(0.05, 1e-4, 5, 0.0479, 2000));
                upperFlywheelMotor.enableVelocityMode(RobotParams.SHOOTER_FLYWHEEL_MAX_VEL, new PidCoefficients(0.05, 1e-4, 5, 0.0479, 2000));
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
        return upperFlywheelMotor.getVelocity() / RobotParams.FLYWHEEL_COUNTS_PER_REVOLUTION / RobotParams.FLYWHEEL_GEAR_RATIO;
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
    public void prepForShoot(double tiltAngle, double distanceToTarget, TrcEvent event, boolean isSecondBall){
        if(isSecondBall){
            //dont tilt, set flywheel velocity to idealShooterVelocity  
        }
        else{
            //set tilter to tiltAngle(ty from vision)+compensation(look up table and interpolation)
            //calculate flywheel velocity based on distanceToTarget(look up table and interpolation)
            //notify event once the shooter has the right flywheel velocity 
            //set idealSHooterVelocity to what you want the flywheel velocity to be 
        }
    }
    public void prepForShoot(double tiltAngle, double distanceToTarget, TrcEvent event){
        prepForShoot(tiltAngle, distanceToTarget, event, false);
    }
    public void shoot(String owner, TrcEvent event){

    }
    //assumes there is already a ball at the exit sensor 
    public void shoot(String owner){
        if(validateOwnership(owner)){
        //Start spinning flywheels
        //If flywheels are spinning fast enough(based on this.idealShooterVelocity)
        //Tell conveyor to shoot ball from the exitSensor
        }   
    }
    public void shoot()
    {
        shoot(null);
    }
    //this method returns the ideal velocity of the shooter calculated for the first ball that can be used for the second ball 
    public double idealShooterVelocity(){
        return idealShooterVelocity; 
    }
    public void resetShooter(){
        //set tilter angle to default position
        idealShooterVelocity = 0.0; 
        onFinishEvent = null; 
    }
}   //class Shooter
