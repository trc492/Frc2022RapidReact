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

import TrcCommonLib.trclib.TrcAddressableLED;
import TrcFrcLib.frclib.FrcAddressableLED;
import TrcFrcLib.frclib.FrcColor;

public class LEDIndicator
{
    private static final TrcAddressableLED.Pattern nominalPattern = // Black
        new TrcAddressableLED.Pattern("Nominal", new FrcColor(0, 0, 0), RobotParams.NUM_LEDS);

    private static final TrcAddressableLED.Pattern fieldOrientedPattern = // Cyan
        new TrcAddressableLED.Pattern("FieldOriented", new FrcColor(0, 63, 63), RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern robotOrientedPattern = // Red
        new TrcAddressableLED.Pattern("RobotOriented", new FrcColor(63, 0, 0), RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern inverseOrientedPattern = // Magenta
        new TrcAddressableLED.Pattern("InverseOriented", new FrcColor(63, 0, 63), RobotParams.NUM_LEDS);

    private static final TrcAddressableLED.Pattern conveyorFullPattern = // White
        new TrcAddressableLED.Pattern("TargetInView", new FrcColor(63, 63, 63), RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern targetInViewPattern = // Yellow
        new TrcAddressableLED.Pattern("TargetInView", new FrcColor(63, 63, 0), RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern targetAlignedPattern = // Green
        new TrcAddressableLED.Pattern("TargetAligned", new FrcColor(0, 63, 0), RobotParams.NUM_LEDS);
    // private static final TrcAddressableLED.Pattern flywheelAtSpeedPattern = // Green
    //     new TrcAddressableLED.Pattern("ReadyToShoot", new FrcColor(63, 63, 0), RobotParams.NUM_LEDS);

    private static final TrcAddressableLED.Pattern[] priorities =
        new TrcAddressableLED.Pattern[]
        {
            nominalPattern,
            inverseOrientedPattern,
            robotOrientedPattern,
            fieldOrientedPattern,
            conveyorFullPattern,
            targetInViewPattern,
            targetAlignedPattern,
        };

    private FrcAddressableLED led;

    /**
     * Constructor: Create an instance of the object.
     */
    public LEDIndicator()
    {
        led = new FrcAddressableLED("LED", RobotParams.NUM_LEDS, RobotParams.PWM_CHANNEL_LED);
        reset();
    }   //LEDIndicator

    /**
     * This method resets the LED strip to the nominal pattern.
     */
    public void reset()
    {
        led.setEnabled(true);
        led.setPatternPriorities(priorities);
        led.reset();
        led.resetAllPatternStates();
        led.setPatternState(nominalPattern, true);
    }   //reset

    /**
     * This method sets the LED to indicate the drive orientation mode of the robot.
     *
     * @param orientation specifies the drive orientation mode.
     */
    public void setDriveOrientation(FrcTeleOp.DriveOrientation orientation)
    {
        switch (orientation)
        {
            case INVERTED:
                led.setPatternState(inverseOrientedPattern, true);
                led.setPatternState(robotOrientedPattern, false);
                led.setPatternState(fieldOrientedPattern, false);
                break;

            case ROBOT:
                led.setPatternState(inverseOrientedPattern, false);
                led.setPatternState(robotOrientedPattern, true);
                led.setPatternState(fieldOrientedPattern, false);
                break;

            case FIELD:
                led.setPatternState(inverseOrientedPattern, false);
                led.setPatternState(robotOrientedPattern, false);
                led.setPatternState(fieldOrientedPattern, true);
                break;
        }
    }   //setDriveOrientation

    public void setConveyorFull(boolean enabled)
    {
        led.setPatternState(conveyorFullPattern, enabled);
    }   //setConveyorFull

    public void setTargetInView(boolean enabled)
    {
        led.setPatternState(targetInViewPattern, enabled);
    }   //setVisionOnTarget

    public void setTargetAligned(boolean enabled)
    {
        led.setPatternState(targetAlignedPattern, enabled);
    }   //setTargetAligned

}   //class LEDIndicator
