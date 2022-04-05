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

import TrcCommonLib.trclib.TrcFilter;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcCANTimeOfFlight;
import TrcFrcLib.frclib.FrcMedianFilter;

public class WallAlignSensor
{
    private FrcCANTimeOfFlight leftLidar, rightLidar;
    private TrcFilter leftFilter, rightFilter;
    private TrcTaskMgr.TaskObject lidarTaskObj;


    public WallAlignSensor()
    {
        leftLidar = new FrcCANTimeOfFlight("LeftLidar", RobotParams.CANID_LEFT_LIDAR);
        leftFilter = new FrcMedianFilter("LeftLidarFilter", 5);

        rightLidar = new FrcCANTimeOfFlight("RightLidar", RobotParams.CANID_RIGHT_LIDAR);
        rightFilter = new FrcMedianFilter("RightLidarFilter", 5);

        lidarTaskObj = TrcTaskMgr.createTask("LidarTask", this::lidarTask);
    }   //WallAlignSensor

    public void setRangingEnabled(boolean enabled)
    {
        if (enabled)
        {
            lidarTaskObj.registerTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            lidarTaskObj.unregisterTask();
        }
        leftFilter.reset();
        rightFilter.reset();
    }   //setRangingEnabled

    public double getLeftDistance()
    {
        double distance = leftLidar.getDistanceInches();

        if (lidarTaskObj.isRegistered())
        {
            distance = leftFilter.filterData(distance);
        }

        return distance + RobotParams.LIDAR_SENSOR_Y_OFFSET;
    }   //getLeftDistance

    public double getRightDistance()
    {
        double distance = rightLidar.getDistanceInches();

        if (lidarTaskObj.isRegistered())
        {
            distance = rightFilter.filterData(distance);
        }
        return distance + RobotParams.LIDAR_SENSOR_Y_OFFSET;
    }   //getRightDistance

    public double getForwardDistanceToWall()
    {
        return TrcUtil.average(getLeftDistance(), getRightDistance());
    }   //getForwardDistanceToWall

    public double getAngleToWall()
    {
        double l = getLeftDistance();
        double r = getRightDistance();
        double theta = Math.atan2(r - l, RobotParams.LIDAR_INTER_SENSOR_DIST);

        return Math.toDegrees(theta);
    }   //getAngleToWall

    public double getShortestDistanceToWall()
    {
        return Math.cos(Math.toRadians(getAngleToWall())) * getForwardDistanceToWall();
    }   //getShortestDistanceToWall

    private void lidarTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        // Don't cache the data, just keep reading to keep the filters up to date.
        leftFilter.filterData(leftLidar.getDistanceInches());
        rightFilter.filterData(rightLidar.getDistanceInches());
    }   //lidarTask

}   //class WallAlignSensor
