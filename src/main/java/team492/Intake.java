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

import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake implements TrcExclusiveSubsystem
{
    private static final String moduleName = "Intake";
    private final FrcCANFalcon intakeMotor;
    private final FrcPneumatic intakePneumatic;
    private boolean gotBall = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param conveyor specifies the conveyor object.
     */
    public Intake(Conveyor conveyor)
    {
        intakeMotor = new FrcCANFalcon(moduleName + ".motor", RobotParams.CANID_INTAKE);
        intakePneumatic = new FrcPneumatic(
            moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.REVPH,
            RobotParams.PNEUMATIC_INTAKE_RETRACT, RobotParams.PNEUMATIC_INTAKE_EXTEND);

        conveyor.registerEntranceEventHandler(this::conveyorEntranceTrigger);
    }   //Intake

    public boolean hasBall()
    {
        return gotBall;
    }   //gotBall

    public void setPower(String owner, double power)
    {
        if (validateOwnership(owner))
        {
            intakeMotor.set(power);
        }
    }   //setPower

    public void setPower(double power)
    {
        setPower(null, power);
    }   //setPower

    public void pickup(String owner)
    {
        if (validateOwnership(owner))
        {
            if (!gotBall)
            {
                setPower(RobotParams.INTAKE_PICKUP_POWER);
            }
        }
    }   //pickup

    public void pickup()
    {
        pickup(null);
    }   //pickup

    public void spitOut(String owner)
    {
        if (validateOwnership(owner))
        {
            if (gotBall)
            {
                setPower(RobotParams.INTAKE_SPITOUT_POWER);
            }
        }
    }   //spitOut

    public void spitOut()
    {
        spitOut(null);
    }   //spitOut

    public void stop(String owner)
    {
        if (validateOwnership(owner))
        {
            setPower(0.0);
        }
    }   //stop

    public void stop()
    {
        stop(null);
    }   //stop

    public void extend(String owner)
    {
        if (validateOwnership(owner))
        {
            intakePneumatic.extend();
        }
    }   //extend

    public void extend()
    {
        extend(null);
    }   //extend

    public void retract(String owner)
    {
        if (validateOwnership(owner))
        {
            intakePneumatic.retract();
        }
    }   //retract

    public void retract()
    {
        retract(null);
    }   //retract

    public boolean isExtended()
    {
        return intakePneumatic.isExtended();
    }   //isExtended

    private void conveyorEntranceTrigger(Object active)
    {
        this.gotBall = (Boolean) active;
        setPower(0.0);
    }   //conveyorEntranceTrigger

}   //class Intake