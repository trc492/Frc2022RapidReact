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

import TrcCommonLib.trclib.TrcPidConveyor;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcDigitalInput;

public class Conveyor
{
    private static final String moduleName = "Conveyor";
    private FrcCANTalon conveyorMotor;
    private FrcDigitalInput entranceSensor, exitSensor;
    private TrcPidConveyor conveyor;

    public Conveyor()
    {
        conveyorMotor = new FrcCANTalon(moduleName + ".motor", RobotParams.CANID_CONVEYOR);
        conveyorMotor.setInverted(RobotParams.CONVEYOR_MOTOR_INVERTED);

        if (RobotParams.CONVEYOR_HAS_ENTRANCE_SENSOR)
        {
            entranceSensor =
                new FrcDigitalInput(moduleName + ".entranceSensor", RobotParams.DIO_CONVEYOR_ENTRANCE_SENSOR);
            entranceSensor.setInverted(RobotParams.CONVEYOR_ENTRANCE_SENSOR_INVERTED);
        }

        if(RobotParams.CONVEYOR_HAS_EXIT_SENSOR)
        {
            exitSensor = new FrcDigitalInput(moduleName + ".exitSensor", RobotParams.DIO_CONVEYOR_EXIT_SENSOR);
            exitSensor.setInverted(RobotParams.CONVEYOR_EXIT_SENSOR_INVERTED);
        }

        TrcPidConveyor.Parameters params = new TrcPidConveyor.Parameters()
            .setScale(RobotParams.CONVEYOR_INCHES_PER_COUNT)
            .setPidParams(RobotParams.CONVEYOR_KP, RobotParams.CONVEYOR_KI, RobotParams.CONVEYOR_KD,
                          RobotParams.CONVEYOR_TOLERANCE)
            .setMovePower(RobotParams.CONVEYOR_MOVE_POWER)
            .setObjectDistance(RobotParams.CONVEYOR_ADVANCE_BALL_DISTANCE)
            .setMaxCapacity(RobotParams.CONVEYOR_MAX_CAPACITY);
        conveyor = new TrcPidConveyor(moduleName, conveyorMotor, entranceSensor, exitSensor, params);
        //write autochoices for number of preload ball 
    }

    public TrcPidConveyor getConveyor()
    {
        return conveyor;
    }   //getConveyor

    //pipeline notes
    // when entrance is broken,
    // if there is NOT a ball breaking the exit one,
    //  then activate conveyor until the intook ball breaks exit
    // if there IS a ball already breaking the exit one,
    //  stop the intake, because we have two balls
    //  lift the intake back up

}   //class Conveyor
