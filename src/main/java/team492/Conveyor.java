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

import TrcCommonLib.trclib.TrcDigitalInputTrigger;
import TrcCommonLib.trclib.TrcNotifier;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcDigitalInput;

public class Conveyor
{
    private static final String moduleName = "Conveyor";
    private FrcCANTalon conveyorMotor;
    private FrcDigitalInput entranceSensor, exitSensor;
    private final TrcDigitalInputTrigger entranceTrigger, exitTrigger;
    private TrcNotifier.Receiver entranceEventHandler, exitEventHandler;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Conveyor()
    {
        conveyorMotor = new FrcCANTalon(moduleName + ".motor", RobotParams.CANID_CONVEYOR);
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
            moduleName + ".exitTrigger", entranceSensor, this::exitEvent);
        exitTrigger.setEnabled(true);
    }   //Conveyor

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
     * This method sets the power of the conveyor.
     *
     * @param power specifies the power to set the conveyor to.
     */
    public void setPower(double power)
    {
        conveyorMotor.set(power);
    }   //setPower

    /**
     * This method moves the ball(s) forward. It will either take in a ball from the entrance or shoot a ball at
     * the exit.
     */
    public void advance()
    {
        move(RobotParams.CONVEYOR_MOVE_POWER);
    }   //advance

    /**
     * This method moves the ball(s) backward. It will either move a ball from the exit back to the entrance or
     * eject a ball out to the intake.
     */
    public void backup()
    {
        move(-RobotParams.CONVEYOR_MOVE_POWER);
    }   //backup

    private void move(double power)
    {
        // Turn on conveyor only if there is a ball to move, either to take in a ball from the entrance, to shoot a
        // ball at the exit or to back up a ball to the entrance. The sensor trigger event will turn the conveyor off.
        if (entranceSensor.isActive() || exitSensor.isActive())
        {
            conveyorMotor.setMotorPower(power);
        }
    }   //move

    /**
     * This method is called when the entrance sensor is triggered.
     *
     * @param active specifies true if an object has activated the sensor, false if the object has deactivated it.
     */
    private void entranceEvent(boolean active)
    {
        conveyorMotor.setMotorPower(0.0);

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
        conveyorMotor.setMotorPower(0.0);

        if (exitEventHandler != null)
        {
            exitEventHandler.notify(active);
        }
    }   //exitEvent

}   //class Conveyor
