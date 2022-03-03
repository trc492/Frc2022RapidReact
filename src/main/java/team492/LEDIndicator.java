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
    private static final TrcAddressableLED.Pattern nominalPattern =
        new TrcAddressableLED.Pattern(FrcColor.FULL_GREEN, RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern fieldOrientedPattern =
        new TrcAddressableLED.Pattern(FrcColor.FULL_WHITE, RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern robotOrientedPattern =
        new TrcAddressableLED.Pattern(FrcColor.FULL_BLUE, RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern inverseOrientedPattern =
        new TrcAddressableLED.Pattern(FrcColor.FULL_RED, RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern flywheelVelOnTargetPattern = 
        new TrcAddressableLED.Pattern(FrcColor.FULL_MAGENTA, RobotParams.NUM_LEDS); 
    private static final TrcAddressableLED.Pattern[] priorities =
        new TrcAddressableLED.Pattern[]
        {
            flywheelVelOnTargetPattern,
            nominalPattern,
            robotOrientedPattern,
            inverseOrientedPattern,
            fieldOrientedPattern
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
    public void setFlywheelTarget(boolean enabled)
    {
        led.setPatternState(flywheelVelOnTargetPattern, enabled);
    }

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
                led.setPatternState(robotOrientedPattern, false);
                led.setPatternState(fieldOrientedPattern, false);
                led.setPatternState(inverseOrientedPattern, true);
                break;

            case FIELD:
                led.setPatternState(inverseOrientedPattern, false);
                led.setPatternState(robotOrientedPattern, false);
                led.setPatternState(fieldOrientedPattern, true);
                break;

            case ROBOT:
                led.setPatternState(inverseOrientedPattern, false);
                led.setPatternState(fieldOrientedPattern, false);
                led.setPatternState(robotOrientedPattern, true);
                break;
        }
    }   //setDriveOrientation

}   //class LEDIndicator
