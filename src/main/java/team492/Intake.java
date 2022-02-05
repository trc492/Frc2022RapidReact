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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/*primitive operations
- setPower
- extend
- retract
*/

public class Intake
{
    public final String instanceName;
    public final FrcCANFalcon intakeMotor;
    public final FrcPneumatic intakePneumatic;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public Intake(String instanceName)
    {
        this.instanceName = instanceName;
        intakeMotor = new FrcCANFalcon(instanceName + ".motor", RobotParams.CANID_INTAKE);
        intakePneumatic = new FrcPneumatic(
            instanceName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_INTAKE_RETRACT, RobotParams.PNEUMATIC_INTAKE_EXTEND);
    }   //Intake

    @Override
    public String toString()
    {
        return instanceName;
    }   //toString
    
    public void setPower(double power)
    {
        intakeMotor.set(power);
    }   //setPower

    public void pickup()
    {
        setPower(RobotParams.INTAKE_PICKUP_POWER);
    }   //pickup

    public void spitOut()
    {
        setPower(RobotParams.INTAKE_SPITOUT_POWER);
    }   //spitOut

    public void stop()
    {
        setPower(0);
    }   //stop

    public void extend()
    {
        intakePneumatic.extend();
    }   //extend

    public void retract()
    {
        intakePneumatic.retract();
    }   //retract

    public boolean isExtended()
    {
        return intakePneumatic.isExtended();
    }   //isExtended

}   //class Intake
