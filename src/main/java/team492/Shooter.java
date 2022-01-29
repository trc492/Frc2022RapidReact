package team492;

import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
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
    public final TrcPidActuator.Parameters tilterParams;
    public final TrcPidActuator tilter;
    public boolean flyWheelInVelocityMode = false;

    public Shooter() {
        lowerFlywheelMotor = new FrcCANFalcon("lowerFlywheelMotor", RobotParams.CANID_SHOOTER_LOWER_FLYWHEEL);
        lowerFlywheelMotor.setBrakeModeEnabled(false);
        upperFlywheelMotor = new FrcCANFalcon("upperFlywheelMotor", RobotParams.CANID_SHOOTER_UPPER_FLYWHEEL);
        upperFlywheelMotor.setBrakeModeEnabled(false);
        setFlywheelVelocityModeEnabled(true);

        tilterMotor = new FrcCANTalon("tilterMotor", RobotParams.CANID_SHOOTER_TILTER);
        tilterLowerLimitSwitch = new FrcCANTalonLimitSwitch("tilterLowerLimitSwitch", tilterMotor, false);
        tilterUpperLimitSwitch = new FrcCANTalonLimitSwitch("tilterUpperLimitSwitch", tilterMotor, true);
        tilterParams = new Parameters()
            .setPidParams(new PidParameters(new PidCoefficients(0.0, 0.0, 0.0, 0.0), 0.0, 0.0));
        tilter = new TrcPidActuator("tilter", tilterMotor, tilterLowerLimitSwitch, tilterUpperLimitSwitch, tilterParams);
    }

    public void shoot() {
        //Start spinning flywheels
        //Auto-aim (Turn to target)
        //If flywheels are spinning fast enough & the robot is asligned with the target,
        //Tell the conveyor to send a ball into the shooter
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

    public void setFlywheelPower(double lowerPower, double upperPower)
    {
        lowerFlywheelMotor.set(lowerPower);
        upperFlywheelMotor.set(upperPower);
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

    public double getLowerFlywheelVelocity()
    {
        return lowerFlywheelMotor.getVelocity();
    }

    public double getUpperFlywheelVelocity()
    {
        return upperFlywheelMotor.getVelocity();
    }

    public double getTilterPosition()
    {
        return tilter.getPosition();
    }

    public void setTilterPosition(String owner, double pos)
    {
        if (validateOwnership(owner))
        {
            tilter.setTarget(pos);
        }
    }

    public void setTilterPosition(double pos)
    {
        setTilterPosition(null, pos);
    }

}
