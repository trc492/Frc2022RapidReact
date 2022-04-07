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
    public static enum ShootLoc
    {
        TarmacAuto,
        TarmacMid,
        OldThreshold,
        Distance11ft,
        StartFar,
        Calibration125in,
        Distance12ft,
        FarThreshold,
        TarmacEdge,
        RingMid,
        LaunchPad,
        Tower,
        Distance156in,
        Distance162in,
        Distance170in,
        Distance7ft,
        Distance13ft,
        Distance14ft,
        Distance15ft,
        Distance17ft,
        Distance18ft,
        Distance195in,
        Distance250in,
        Distance275in,
        Distance300in,
        Interpolated,
        Extrapolated,
        Calibration
    }   //enum ShootLoc

    public static class Params
    {
        public final ShootLoc loc;
        public final double distance;
        public final double lowerFlywheelVelocity;
        public final double upperFlywheelVelocity;
        public final double tilterAngle;

        public Params(
            ShootLoc loc, double distance, double lowerFlywheelVelocity, double upperFlywheelVelocity,
            double tilterAngle)
        {
            this.loc = loc;
            this.distance = distance;
            this.lowerFlywheelVelocity = lowerFlywheelVelocity;
            this.upperFlywheelVelocity = upperFlywheelVelocity;
            this.tilterAngle = tilterAngle;
        }   //Params

        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "loc=%s, distance=%.1f, lowerFwVel=%.0f, upperFwVel=%.0f, tilterAngle=%.2f",
                loc, distance, lowerFlywheelVelocity, upperFlywheelVelocity, tilterAngle);
        }   //toString

    }   //class Params

    private final ArrayList<Params> paramTable;
    private final HashMap<ShootLoc, Params> paramMap;

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
     * @param loc specifies the shoot location for the entry.
     * @param distance specifies the target distance.
     * @param lowerFlywheelVel specifies the lower flywheel velocity RPM.
     * @param upperFlywheelVel specifies the upper flywheel velocity in RPM.
     * @param tilterAngle specifies the tilter angle in degrees.
     *
     * @return this instance object.
     */
    public ShootParamTable add(
        ShootLoc loc, double distance, double lowerFlywheelVel, double upperFlywheelVel, double tilterAngle)
    {
        Params newEntry = new Params(loc, distance, lowerFlywheelVel, upperFlywheelVel, tilterAngle);
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
        paramMap.put(newEntry.loc, newEntry);
        return this;
    }   //add

    /**
     * This method returns the Shoot Param entry that matches the given name.
     *
     * @param loc specifies the shoot location for the entry.
     * @return shoot param entry that matches the shoot location, null if not found.
     */
    public Params get(ShootLoc loc)
    {
        return paramMap.get(loc);
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

        if (paramTable.size() < 2)
        {
            throw new RuntimeException("ShootParamTable must have at least 2 entries.");
        }

        Params firstEntry = paramTable.get(0);
        Params lastEntry = paramTable.get(paramTable.size() - 1);

        if (distance <= firstEntry.distance)
        {
            // The provided distance is below the table range, extropolate.
            Params nextEntry = paramTable.get(1);
            foundEntry = new Params(
                ShootLoc.Extrapolated, distance,
                extrapolateVelocity(false, distance, firstEntry, nextEntry),
                extrapolateVelocity(true, distance, firstEntry, nextEntry),
                firstEntry.tilterAngle);
        }
        else if (distance > lastEntry.distance)
        {
            // The provided distance is above the table range, extropolate.
            Params prevEntry = paramTable.get(paramTable.size() - 2);
            foundEntry = new Params(
                ShootLoc.Extrapolated, distance,
                extrapolateVelocity(false, distance, prevEntry, lastEntry),
                extrapolateVelocity(true, distance, prevEntry, lastEntry),
                lastEntry.tilterAngle);
        }
        else
        {
            for (int i = 1; i < paramTable.size(); i++)
            {
                Params entry = paramTable.get(i);

                if (distance <= entry.distance)
                {
                    Params prevEntry = paramTable.get(i - 1);
                    double w = (distance - prevEntry.distance) / (entry.distance - prevEntry.distance);
                    foundEntry = new Params(
                        ShootLoc.Interpolated, distance,
                        (1 - w) * prevEntry.lowerFlywheelVelocity + w * entry.lowerFlywheelVelocity,
                        (1 - w) * prevEntry.upperFlywheelVelocity + w * entry.upperFlywheelVelocity,
                        entry.tilterAngle);
                    break;
                }
            }
        }

        return foundEntry;
    }   //get

    /**
     * This method extrapolates the lower or upper flywheel velocity with the given distance and the two points
     * of the neighboring segment in the table.
     *
     * @param upper specifies true to extrapolate upper flywheel velocity, false to extrapolate lower flywheel
     *              velocity.
     * @param distance specifies the target distance.
     * @param entry1 specifies the lower entry of the neighboring segment.
     * @param entry2 specifies the upper entry of the neighboring segment.
     * @return
     */
    private double extrapolateVelocity(boolean upper, double distance, Params entry1, Params entry2)
    {
        double deltaVel = upper? entry2.upperFlywheelVelocity - entry1.upperFlywheelVelocity:
                                 entry2.lowerFlywheelVelocity - entry1.lowerFlywheelVelocity;
        double deltaDistance = entry2.distance - entry1.distance;
        double m = deltaVel / deltaDistance;
        double b = (upper? entry1.upperFlywheelVelocity: entry1.lowerFlywheelVelocity) - m*entry1.distance;
        double vel = m*distance + b;

        if (vel < 0.0)
        {
            // If extrapolated velocity is negative, just return the velocity of one of the two entries whoever is
            // closer to the given distance.
            double d1 = Math.abs(distance - entry1.distance);
            double d2 = Math.abs(distance - entry2.distance);
            if (d1 < d2)
            {
                vel = upper? entry1.upperFlywheelVelocity: entry1.lowerFlywheelVelocity;
            }
            else
            {
                vel = upper? entry2.upperFlywheelVelocity: entry2.lowerFlywheelVelocity;
            }
        }

        return m*distance + b;
    }   //extrapolateVelocity

}   //class ShootParamTable
