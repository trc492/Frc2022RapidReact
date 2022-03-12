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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;

public class ShootParamTable
{
    public static class Params
    {
        public final String name;
        public final double distance;
        public final double lowerFlywheelVelocity;
        public final double upperFlywheelVelocity;
        public final double tilterAngle;

        public Params(
            String name, double distance, double lowerFlywheelVelocity, double upperFlywheelVelocity,
            double tilterAngle)
        {
            this.name = name;
            this.distance = distance;
            this.lowerFlywheelVelocity = lowerFlywheelVelocity;
            this.upperFlywheelVelocity = upperFlywheelVelocity;
            this.tilterAngle = tilterAngle;
        }   //Params

        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "name=%s, distance=%.1f, lowerFwVel=%.0f, upperFwVel=%.0f, tilterAngle=%.2f",
                name, distance, lowerFlywheelVelocity, upperFlywheelVelocity, tilterAngle);
        }   //toString

    }   //class Params

    private final ArrayList<Params> paramTable;
    private final HashMap<String, Params> paramMap;

    /**
     * Constructor: Create an instance of the object.
     */
    public ShootParamTable()
    {
        paramTable = new ArrayList<>();
        paramMap = new HashMap<>();
    }   //ShootParamTable

    /**
     * This method adds an entry to the ShootParamTable sorted by distance.
     *
     * @param name specifies the name of the entry.
     * @param distance specifies the target distance.
     * @param lowerFlywheelVel specifies the lower flywheel velocity RPM.
     * @param upperFlywheelVel specifies the upper flywheel velocity in RPM.
     * @param tilterAngle specifies the tilter angle in degrees.
     *
     * @return this instance object.
     */
    public ShootParamTable add(
        String name, double distance, double lowerFlywheelVel, double upperFlywheelVel, double tilterAngle)
    {
        Params newEntry = new Params(name, distance, lowerFlywheelVel, upperFlywheelVel, tilterAngle);
        int insertPoint = paramTable.size();

        if (tilterAngle != RobotParams.TILTER_FAR_ANGLE && tilterAngle != RobotParams.TILTER_CLOSE_ANGLE)
        {
            throw new IllegalArgumentException("Tilter angle must either be FAR or CLOSE.");
        }

        for (int i = 0; i < paramTable.size(); i++)
        {
            Params entry = paramTable.get(i);
            if (distance == entry.distance)
            {
                throw new RuntimeException("An entry with the same distance already exist.");
            }
            else if (distance < entry.distance)
            {
                insertPoint = i;
                break;
            }
        }

        paramTable.add(insertPoint, newEntry);
        paramMap.put(newEntry.name, newEntry);
        return this;
    }   //add

    /**
     * This method returns the Shoot Param entry that matches the given name.
     *
     * @param name specifies the name of the entry.
     * @return shoot param entry that matches the name, null if not found.
     */
    public Params get(String name)
    {
        return paramMap.get(name);
    }   //get

    /**
     * This method returns the Shoot Param entry with the given distance. If there is no exact match, it will return
     * an entry that linearly interpolates between two entries in the table.
     *
     * @param distance specifies the distance to lookup in the table.
     * @return shoot param entry that matches the distance. If no exact match, an interpolated entry is returned.
     *         Returns null if the distance is out of range of the table entries.
     */
    public Params get(double distance)
    {
        Params foundEntry = null;

        for (int i = 0; i < paramTable.size(); i++)
        {
            Params entry = paramTable.get(i);
            if (distance == entry.distance)
            {
                foundEntry = entry;
                break;
            }
            else if (distance < entry.distance)
            {
                if (i > 0)
                {
                    Params prevEntry = paramTable.get(i - 1);
                    double w = (distance - prevEntry.distance) / (entry.distance - prevEntry.distance);
                    foundEntry = new Params(
                        "interpolated", distance,
                        (1 - w) * prevEntry.lowerFlywheelVelocity + w * entry.lowerFlywheelVelocity,
                        (1 - w) * prevEntry.upperFlywheelVelocity + w * entry.upperFlywheelVelocity,
                        entry.tilterAngle);
                }
            }
        }

        return foundEntry;
    }   //get

}   //class ShootParamTable
