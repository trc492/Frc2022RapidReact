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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInputTrigger;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcNotifier;
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcDigitalInput;

public class Conveyor implements TrcExclusiveSubsystem
{
    private static final String moduleName = "Conveyor";

    private enum TriggerAction
    {
        DoNothing,
        StopOnForward,
        StopOnBackward
    }   //enum TriggerState

    private final FrcCANTalon conveyorMotor;
    private final FrcDigitalInput entranceSensor, exitSensor;
    private final TrcDigitalInputTrigger entranceTrigger, exitTrigger;

    private TrcNotifier.Receiver entranceEventHandler, exitEventHandler;
    private TrcEvent onFinishedEvent;
    private TriggerAction triggerAction = TriggerAction.DoNothing;
    private TrcDbgTrace msgTracer = null;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Conveyor()
    {
        conveyorMotor = new FrcCANTalon(moduleName + ".motor", RobotParams.CANID_CONVEYOR);
        conveyorMotor.motor.configFactoryDefault();
        conveyorMotor.setInverted(RobotParams.CONVEYOR_MOTOR_INVERTED);

        entranceSensor =
            new FrcDigitalInput(moduleName + ".entranceSensor", RobotParams.DIO_CONVEYOR_ENTRANCE_SENSOR);
        entranceSensor.setInverted(RobotParams.CONVEYOR_ENTRANCE_SENSOR_INVERTED);
        entranceTrigger = new TrcDigitalInputTrigger(
            moduleName + ".entranceTrigger", entranceSensor, this::entranceEvent);
        entranceTrigger.setEnabled(true);

        exitSensor = new FrcDigitalInput(moduleName + ".exitSensor", RobotParams.DIO_CONVEYOR_EXIT_SENSOR);
        exitSensor.setInverted(RobotParams.CONVEYOR_EXIT_SENSOR_INVERTED);
        exitTrigger = new TrcDigitalInputTrigger(
            moduleName + ".exitTrigger", exitSensor, this::exitEvent);
        exitTrigger.setEnabled(true);
    }   //Conveyor

    /**
     * This method is called to cancel any pending operations and stop the subsystem. It is typically called before
     * exiting a competition mode.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     */
    public void cancel(String owner)
    {
        if (validateOwnership(owner))
        {
            conveyorMotor.set(0.0);
        }
    }   //cancel

    /**
     * This method enables/disables tracing for the shooter subsystem.
     *
     * @param tracer specifies the tracer to use for logging events.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        msgTracer = tracer;
    }   //setMsgTracer

    /**
     * This method is called to cancel any pending operations and stop the subsystem. It is typically called before
     * exiting a competition mode.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method registers an event handler to be notified when the entrance sensor is triggered.
     *
     * @param handler event handler to be notified.
     */
    public void registerEntranceEventHandler(TrcNotifier.Receiver handler)
    {
        entranceEventHandler = handler;
    }   //registerEntranceEventHandler

    /**
     * This method registers an event handler to be notified when the exit sensor is triggered.
     *
     * @param handler event handler to be notified.
     */
    public void registerExitEventHandler(TrcNotifier.Receiver handler)
    {
        exitEventHandler = handler;
    }   //registerExitEventHandler

    /**
     * This method returns the sensor state read from the digital sensor.
     *
     * @return digital sensor state.
     */
    public boolean isEntranceSensorActive()
    {
        return entranceSensor.isActive();
    }   //isEntranceSensorActive

    /**
     * This method returns the sensor state read from the digital sensor.
     *
     * @return digital sensor state.
     */
    public boolean isExitSensorActive()
    {
        return exitSensor.isActive();
    }   //isExitSensorActive

    /**
     * This method returns the motor power set on the conveyor.
     *
     * @return conveyor motor power.
     */
    public double getMotorPower()
    {
        return conveyorMotor.getMotorPower();
    }   //getMotorPower

    /**
     * This method determines the number of balls in the conveyor.
     *
     * @return number of balls in conveyor.
     */
    public int getNumBalls()
    {
        int numBalls = 0;

        if (entranceSensor.isActive())
        {
            numBalls++;
        }

        if (exitSensor.isActive())
        {
            numBalls++;
        }

        return numBalls;
    }   //getNumBalls

    /**
     * This method sets the power of the conveyor. Note that setPower methods do not pay attention to entrance or
     * exit sensors. For moving the conveyor that will stop on entrance or exit sensor triggers use the advance or
     * backup methods instead.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay.
     * @param power specifies the power to set the conveyor to.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *                 turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void setPower(String owner, double delay, double power, double duration, TrcEvent event)
    {
        if (validateOwnership(owner))
        {
            conveyorMotor.set(delay, power, duration, event);
        }
    }   //setPower

    /**
     * This method sets the power of the conveyor. Note that setPower methods do not pay attention to entrance or
     * exit sensors. For moving the conveyor that will stop on entrance or exit sensor triggers use the advance or
     * backup methods instead.
     *
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay.
     * @param power specifies the power to set the conveyor to.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *                 turning off.
     * @param event specifies the event to signal when the motor operation is completed.
     */
    public void setPower(double delay, double power, double duration, TrcEvent event)
    {
        setPower(null, delay, power, duration, event);
    }   //setPower

    /**
     * This method sets the power of the conveyor. Note that setPower methods do not pay attention to entrance or
     * exit sensors. For moving the conveyor that will stop on entrance or exit sensor triggers use the advance or
     * backup methods instead.
     *
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay.
     * @param power specifies the power to set the conveyor to.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *                 turning off.
     */
    public void setPower(double delay, double power, double duration)
    {
        setPower(null, delay, power, duration, null);
    }   //setPower

    /**
     * This method sets the power of the conveyor. Note that setPower methods do not pay attention to entrance or
     * exit sensors. For moving the conveyor that will stop on entrance or exit sensor triggers use the advance or
     * backup methods instead.
     *
     * @param power specifies the power to set the conveyor to.
     */
    public void setPower(double power)
    {
        setPower(null, 0.0, power, 0.0, null);
    }   //setPower

    /**
     * This method moves the ball forward or backward with the given power.
     *
     * @param power specifies positive power to move the ball forward, negative to move backward.
     * @param event specifies the event to notify when done, can be null if not provided.
     */
    private void move(double power, TrcEvent event)
    {
        final String funcName = "move";
        boolean entranceHasBall = entranceSensor.isActive();
        boolean exitHasBall = exitSensor.isActive();

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] power=%.1f, event=%s, entrance=%s, exit=%s, triggerAction=%s",
                TrcUtil.getModeElapsedTime(), power, event, entranceSensor.isActive(), exitSensor.isActive(),
                triggerAction);
        }

        // Turn on conveyor only if there is a ball to move, either to take in a ball from the entrance, to shoot a
        // ball at the exit or to back up a ball to the entrance. The sensor trigger event will turn the conveyor off
        // if necessary.
        if (entranceHasBall || exitHasBall)
        {
            this.onFinishedEvent = event;
            conveyorMotor.set(power);
        }
    }   //move

    /**
     * This method moves the ball(s) forward. It will either take in a ball from the entrance or shoot a ball at
     * the exit.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     * @param event specifies the event to notify when done, can be null if not provided.
     */
    public void advance(String owner, TrcEvent event)
    {
        if (validateOwnership(owner))
        {
            triggerAction = TriggerAction.StopOnForward;
            move(RobotParams.CONVEYOR_MOVE_POWER, event);
        }
    }   //advance

    /**
     * This method moves the ball(s) forward. It will either take in a ball from the entrance or shoot a ball at
     * the exit.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     */
    public void advance(String owner)
    {
        advance(owner, null);
    }   //advance

    /**
     * This method moves the ball(s) forward. It will either take in a ball from the entrance or shoot a ball at
     * the exit.
     */
    public void advance()
    {
        advance(null, null);
    }   //advance

    /**
     * This method moves the ball(s) backward. It will either move a ball from the exit back to the entrance or
     * eject a ball out to the intake.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     * @param event specifies the event to notify when done, can be null if not provided.
     */
    public void backup(String owner, TrcEvent event)
    {
        if (validateOwnership(owner))
        {
            triggerAction = TriggerAction.StopOnBackward;
            move(-RobotParams.CONVEYOR_MOVE_POWER, event);
        }
    }   //backup

    /**
     * This method moves the ball(s) backward. It will either move a ball from the exit back to the entrance or
     * eject a ball out to the intake.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller does not
     *              require exclusive access.
     */
    public void backup(String owner)
    {
        backup(owner, null);
    }   //backup

    /**
     * This method moves the ball(s) backward. It will either move a ball from the exit back to the entrance or
     * eject a ball out to the intake.
     */
    public void backup()
    {
        backup(null, null);
    }   //backup

    /**
     * This method is called when the entrance sensor is triggered.
     *
     * @param active specifies true if an object has activated the sensor, false if the object has deactivated it.
     */
    private void entranceEvent(boolean active)
    {
        final String funcName = "entranceEvent";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] active=%s, triggerAction=%s, onFinishedEvent=%s",
                TrcUtil.getModeElapsedTime(), active, triggerAction, onFinishedEvent);
        }

        if (active)
        {
            if (triggerAction == TriggerAction.StopOnBackward)
            {
                // backing up a ball to the entrance.
                conveyorMotor.set(0.0);
                triggerAction = TriggerAction.DoNothing;
                if (onFinishedEvent != null)
                {
                    onFinishedEvent.signal();
                }
            }
            else if (triggerAction == TriggerAction.DoNothing)
            {
                // Entrance sensor is triggered not because of advance or backup calls. It must be caused by intake.
                // In this case, we want to see if there is space to advance the ball to the exit. Can only do this
                // if nobody is currently owning exclusive access.
                if (!exitSensor.isActive())
                {
                    // No ball at the exit.
                    advance();
                }
            }

            if (exitSensor.isActive())
            {
                conveyorMotor.set(-0.25, 0.2);
            }
        }

        if (entranceEventHandler != null)
        {
            entranceEventHandler.notify(active);
        }
    }   //entranceEvent

    /**
     * This method is called when the exit sensor is triggered.
     *
     * @param active specifies true if an object has activated the sensor, false if the object has deactivated it.
     */
    private void exitEvent(boolean active)
    {
        final String funcName = "exitEvent";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] active=%s, triggerAction=%s, onFinishedEvent=%s",
                TrcUtil.getModeElapsedTime(), active, triggerAction, onFinishedEvent);
        }

        if (triggerAction == TriggerAction.StopOnForward)
        {
            // advancing a ball to the exit or moving the ball to the shooter.
            conveyorMotor.set(0.0);
            triggerAction = TriggerAction.DoNothing;
            if (onFinishedEvent != null)
            {
                onFinishedEvent.signal();
            }
        }

        if (exitEventHandler != null)
        {
            exitEventHandler.notify(active);
        }
    }   //exitEvent

}   //class Conveyor
