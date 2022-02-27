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

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake implements TrcExclusiveSubsystem
{
    private static final String moduleName = "Intake";
    private final Conveyor conveyor;
    private final FrcCANFalcon intakeMotor;
    private final FrcPneumatic intakePneumatic;
    private TrcEvent onFinishedEvent;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param conveyor specifies the conveyor object.
     */
    public Intake(Conveyor conveyor)
    {
        this.conveyor = conveyor;
        intakeMotor = new FrcCANFalcon(moduleName + ".motor", RobotParams.CANID_INTAKE);
        intakePneumatic = new FrcPneumatic(
            moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_INTAKE_RETRACT, RobotParams.PNEUMATIC_INTAKE_EXTEND);

        //conveyor.registerEntranceEventHandler(this::conveyorEntranceTrigger);
    }   //Intake

    public void setPower(String owner, double delay, double power, double duration)
    {
        if (validateOwnership(owner))
        {
            intakeMotor.set(delay, power, duration);
        }
    }   //setPower

    public void setPower(double delay, double power, double duration)
    {
        setPower(null, delay, power, duration);
    }   //setPower

    public void setPower(double power)
    {
        setPower(null, 0.0, power, 0.0);
    }   //setPower

    public void pickup(String owner, TrcEvent event)
    {
        this.onFinishedEvent = event; 
        if (validateOwnership(owner))
        {
            boolean ballAtEntrance = conveyor.isEntranceSensorActive();
            boolean ballAtExit = conveyor.isExitSensorActive();

            // Only do this if there is room. There are 4 scenarios:
            // 1. Ball at the entrnace and ball at the exit (full): do nothing.
            // 2. Ball at the entrance: move ball to the exit and start intake.
            // 3. Ball at the exit: start intake.
            // 4. No ball at all (empty): start intake.
            if (!ballAtEntrance || !ballAtExit)
            {
                this.onFinishedEvent = event;
                if (ballAtEntrance)
                {
                    conveyor.advance();
                    setPower(RobotParams.INTAKE_PICKUP_DELAY, RobotParams.INTAKE_PICKUP_POWER, 0.0);
                }
                else
                {
                    setPower(0.0, RobotParams.INTAKE_PICKUP_POWER, 0.0);
                }
            }
        }
    }   //pickup

    public void pickup(TrcEvent event)
    {
        pickup(null, event);
    }   //pickup

    public void pickup()
    {
        pickup(null, null);
    }   //pickup

    public void spitOut(String owner)
    {
        if (validateOwnership(owner))
        {
            if (conveyor.isEntranceSensorActive())
            {
                setPower(0.0, RobotParams.INTAKE_SPITOUT_POWER, 0.0);
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
            setPower(0.0, 0.0, 0.0);
        }
    }   //stop

    public void stop()
    {
        stop(null);
    }   //stop

    public void deploy(String owner)
    {
        if (validateOwnership(owner))
        {
            intakePneumatic.extend();
        }
    }   //deploy

    public void deploy()
    {
        deploy(null);
    }   //deploy

    public void extend(String owner) {
        if(validateOwnership(owner)) {
            intakePneumatic.extend();
        }
    }

    public void extend() {
        extend(null);
    }

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
        // setPower(0.0, 0.0, 0.0);
        // if (onFinishedEvent != null)
        // {
        //     onFinishedEvent.signal();
        //     onFinishedEvent = null;
        // }

    }   //conveyorEntranceTrigger

}   //class Intake