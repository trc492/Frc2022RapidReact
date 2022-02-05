package team492;

import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/*primitive operations
- setPower
- extend
- retract
*/

class Intake{
    
    public final String instanceName;
    public final FrcCANFalcon intakeMotor;
    public final FrcPneumatic intakePneumatic;

    //constructor
    public Intake(String instanceName)
    {
        this.instanceName = instanceName;
        intakeMotor = new FrcCANFalcon(instanceName + ".motor", RobotParams.CANID_INTAKE);
        intakePneumatic = new FrcPneumatic(
            instanceName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_INTAKE_RETRACT, RobotParams.PNEUMATIC_INTAKE_EXTEND);
    }

    //public String toString
    public String toString(){
        return instanceName;
    }
    
    //public void extend
    public void extend(){
        intakePneumatic.extend();
    }

    //public void retract
    public void retract(){
        intakePneumatic.retract();
    }

    //public void setPower(double)
    public void setPower(double pwr){
        intakeMotor.set(pwr);
    }

    //public bool isExtended
    public boolean isExtended(){
        return intakePneumatic.isExtended();
    }

    //public void intakeIn
    public void intakeIn(){
        setPower(RobotParams.INTAKE_PICKUP_POWER);
    }

    //public void intakeOut
    public void intakeOut(){
        setPower(RobotParams.INTAKE_SPITOUT_POWER);
    }

    public void stop(){
        setPower(0);
    }
}