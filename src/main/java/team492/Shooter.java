package team492;

import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidActuator.Parameters;
import TrcCommonLib.trclib.TrcPidController.PidCoefficients;
import TrcCommonLib.trclib.TrcPidController.PidParameters;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcCANTalonLimitSwitch;

public class Shooter implements TrcPidController.PidInput{

    public final FrcCANFalcon bottomShooterMotor, upperShooterMotor;
    public final FrcCANTalon tiltingElevatorMotor;
    public final FrcCANTalonLimitSwitch tiltingElevatorUpperLimitSwitch, tiltingElevatorBottomLimitSwitch;
    public final TrcPidActuator.Parameters tiltingElevatorParamaters;
    public final TrcPidActuator tiltingElevator;

    public Shooter() {
        bottomShooterMotor = new FrcCANFalcon("bottomShooterMotor", RobotParams.CANID_SHOOTER_LOWER);
        bottomShooterMotor.setBrakeModeEnabled(false);
        upperShooterMotor = new FrcCANFalcon("upperShooterMotor", RobotParams.CANID_SHOOTER_UPPER);
        upperShooterMotor.setBrakeModeEnabled(false);
        tiltingElevatorMotor = new FrcCANTalon("tiltingElevatorMotor", RobotParams.CANID_TILTING_ELEVATOR);
        tiltingElevatorBottomLimitSwitch = new FrcCANTalonLimitSwitch("tiltingElevatorBottomLimitSwitch", tiltingElevatorMotor, false);
        tiltingElevatorUpperLimitSwitch = new FrcCANTalonLimitSwitch("tiltingElevatorUpperLimitSwitch", tiltingElevatorMotor, true);
        tiltingElevatorParamaters = new Parameters();
        tiltingElevatorParamaters.setPidParams(new PidParameters(new PidCoefficients(0.0, 0.0, 0.0, 0.0), 0.0, 0.0));
        tiltingElevator = new TrcPidActuator("tiltingElevator", tiltingElevatorMotor, tiltingElevatorBottomLimitSwitch, tiltingElevatorUpperLimitSwitch, tiltingElevatorParamaters);
    }

    public void shoot() {
        //Start spinning flywheels
        //Auto-aim (Turn to target)
        //If flywheels are spinning fast enough & the robot is asligned with the target,
        //Tell the conveyor to send a ball into the shooter
    }

    public void setUpperPower(double power) {
        upperShooterMotor.setMotorPower(power);
    }
    public void setLowerPower(double power) {
        bottomShooterMotor.setMotorPower(power);
    }
    public void setShooterPower(double power) {
        setUpperPower(power);
        setLowerPower(power);
    }

    public void setElevatorDist(double position) {
        tiltingElevator.setTarget(position);
    }

    @Override
    public double get() {
        return tiltingElevatorMotor.getMotorPosition();
    }

}
