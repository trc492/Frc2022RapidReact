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
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake implements TrcExclusiveSubsystem
{
    private static final String moduleName = "Intake";
    private static final boolean debugEnabled = true;

    private final Robot robot;
    private final FrcCANFalcon intakeMotor;
    private final FrcPneumatic intakePneumatic;
    private TrcEvent onFinishedEvent;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param conveyor specifies the conveyor object.
     */
    public Intake(Robot robot)
    {
        this.robot = robot;
        intakeMotor = new FrcCANFalcon(moduleName + ".motor", RobotParams.CANID_INTAKE);
        intakeMotor.motor.configFactoryDefault();
        intakePneumatic = new FrcPneumatic(
            moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_INTAKE_RETRACT, RobotParams.PNEUMATIC_INTAKE_EXTEND);
        intakePneumatic.retract();

        robot.conveyor.registerEntranceEventHandler(this::conveyorEntranceTrigger);
    }   //Intake

    /**
     * This method is called to cancel any pending operations and stop the subsystem. It is typically called before
     * exiting a competition mode.
     */
    public void cancel()
    {
        intakeMotor.set(0.0);
    }   //cancel

    /**
     * This method returns the motor power set on the intake.
     *
     * @return intake motor power.
     */
    public double getMotorPower()
    {
        return intakeMotor.getMotorPower();
    }   //getMotorPower

    public void setPower(String owner, double delay, double power, double duration)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(
                funcName, "[%.3f] owner=%s, delay=%.1f, power=%.1f, duration=%.3f",
                TrcUtil.getModeElapsedTime(), owner, delay, power, duration);
        }

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
        final String funcName = "pickup";
        boolean ballAtEntrance = robot.conveyor.isEntranceSensorActive();
        boolean ballAtExit = robot.conveyor.isExitSensorActive();

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(
                funcName, "[%.3f] owner=%s, event=%s, entrance=%s, exit=%s",
                TrcUtil.getModeElapsedTime(), owner, event, ballAtEntrance, ballAtExit);
        }

        if (validateOwnership(owner))
        {
            // Only do this if there is room. There are 4 scenarios:
            // 1. Ball at the entrance and ball at the exit (full): do nothing.
            // 2. Ball at the entrance: move ball to the exit and start intake.
            // 3. Ball at the exit: start intake.
            // 4. No ball at all (empty): start intake.
            if (!ballAtEntrance || !ballAtExit)
            {
                this.onFinishedEvent = event;
                if (ballAtEntrance)
                {
                    robot.conveyor.advance();
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
        final String funcName = "spitOut";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(
                funcName, "[%.3f] owner=%s, entrance=%s, exit=%s",
                TrcUtil.getModeElapsedTime(), owner, robot.conveyor.isEntranceSensorActive(), robot.conveyor.isExitSensorActive());
        }

        if (validateOwnership(owner))
        {
            if (robot.conveyor.isEntranceSensorActive())
            {
                setPower(0.0, RobotParams.INTAKE_SPITOUT_POWER, 0.0);
            }
        }
    }   //spitOut

    public void spitOut()
    {
        spitOut(null);
    }   //spitOut

    public void stop(String owner, double delay)
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "[%.3f] owner=%s", TrcUtil.getModeElapsedTime(), owner);
        }

        if (validateOwnership(owner))
        {
            setPower(delay, 0.0, 0.0);
        }
    }   //stop

    public void stop(double delay)
    {
        stop(null, delay);
    }   //stop

    public void stop()
    {
        stop(null, 0.0);
    }   //stop

    public void extend(String owner)
    {
        final String funcName = "extend";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "[%.3f] owner=%s", TrcUtil.getModeElapsedTime(), owner);
        }

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
        final String funcName = "retract";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(funcName, "[%.3f] owner=%s", TrcUtil.getModeElapsedTime(), owner);
        }

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
        final String funcName = "conveyorEntranceTrigger";
        boolean exitHasBall = robot.conveyor.isExitSensorActive();

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(
                funcName, "[%.3f] active=%s, exitHasBall=%s, onFinishedEvent=%s",
                TrcUtil.getModeElapsedTime(), active, exitHasBall, onFinishedEvent);
        }

        if ((boolean) active)
        {
            // Stop only when a ball has reached exit. If the conveyor has only one ball, that ball will be
            // transported all the way to the exit before we will stop the conveyor. Keep the intake spinning
            // and we will stop the intake when we pick up the second ball.
            if (exitHasBall)
            {
                setPower(0.0, 0.0, 0.0);
            }

            if (onFinishedEvent != null)
            {
                onFinishedEvent.signal();
                onFinishedEvent = null;
            }
        }
    }   //conveyorEntranceTrigger

}   //class Intake