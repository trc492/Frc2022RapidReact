package team492;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import TrcCommonLib.trclib.TrcUtil;

public class TrajectoryCalculator
{
    public static final double G = 9.81; // m/s^2
    public static final double METERS_PER_INCH = 0.0254;
    public static final double C = 0.5; // drag coefficient of sphere
    public static final double A = Math.PI * Math.pow(3.5 * METERS_PER_INCH, 2); // m^2, cross-sectional area of ball
    public static final double P = 1.225; // kg/m^3, density of air at sea level
    public static final double M = 0.14; // kg, mass of ball pulled from FIRST website

    public static final int MAX_ITER = 10;
    public static final double ITER_MAX_ERROR = 0.5; // degrees
    public static final double ITER_OVERRUN_MAX_ERROR = 5; // degrees
    public static final boolean debugEnabled = false;

    private static RealVector getVertexFromShooter(RealVector vertexFromPivot, double thetaRad)
    {
        return vertexFromPivot.subtract(
            TrcUtil.createVector(Math.cos(thetaRad), Math.sin(thetaRad)).mapMultiply(RobotParams.SHOOTER_BARREL_LENGTH));
    }

    private static void debug(String format, Object... args)
    {
        if (debugEnabled)
        {
            System.out.printf(format, args);
        }
    }

    public static RealVector interpolateVector(double distance)
    {
        double[] distances = new double[] { 76, 124, 180, 220, 280, 330 };
        double[] velocities = new double[] { 464, 537, 720, 780, 820, 920 };
        double[] angles = new double[] { 37, 30.5, 24.5, 25, 23, 21 };
        for (int i = 0; i < distances.length - 1; i++)
        {
            if (TrcUtil.inRange(distance, distances[i], distances[i + 1]))
            {
                double w = (distance - distances[i]) / (distances[i + 1] - distances[i]);
                double v = (1 - w) * velocities[i] + w * velocities[i + 1];
                double angle = (1 - w) * angles[i] + w * angles[i + 1];
                return TrcUtil.createVector(v, angle);
            }
        }
        return TrajectoryCalculator.calculateWithArmWithDrag(TrcUtil
            .createVector((distance + RobotParams.CAMERA_Y_OFFSET_TO_PIVOT) * 0.84,
                RobotParams.HIGH_TARGET_HEIGHT - RobotParams.PIVOT_HEIGHT + 6));
    }

    /**
     * Calculate the trajectory to place the vertex at a given point using a linear drag model. This method accounts for
     * an arm with the shooter at the end of the arm. Rotating the arm to match the trajectory angle will change the
     * y and z displacement of the target from the shooter, making analytical calculation of the optimal angle impossible.
     * This is solved by using iterative calculations. The end result should have the arm angle be the same as the
     * shooter angle. The initial arm angle is given as a straight line from the pivot to the target. The trajectory
     * angle is calculated from this, which must be larger, due to the negative concavity of the path due to gravity.
     * As the arm angle increases, the trajectory angle must decrease, and at some point they intersect. This point is
     * the optimal arm angle. We find this point by continually averaging the arm angle and its corresponding trajectory
     * angle.
     *
     * @param vertexFromPivot The y and z displacement of the vertex of the path from the pivot of the arm,
     *                        in the robot reference frame.
     * @return 2d vector where the first entry is initial speed and the second entry is pitch in degrees.
     * If the algorithm failed to converge, null is returned instead.
     */
    public static RealVector calculateWithArmWithDrag(RealVector vertexFromPivot)
    {
        // initial angle should be pointing right at the target
        double theta = Math.atan2(vertexFromPivot.getEntry(1), vertexFromPivot.getEntry(0));
        RealVector relVertex = getVertexFromShooter(vertexFromPivot, theta);
        // Get trajectory from the tip of the arm using the initial angle
        RealVector traj = TrajectoryCalculator.calculateWithVertexWithDrag(relVertex);
        debug("Vertex=%s, theta=%.2f, trajTheta=%.2f\n", vertexFromPivot.toString(), Math.toDegrees(theta),
            traj.getEntry(1));
        int i = 0;
        while (Math.abs(traj.getEntry(1) - Math.toDegrees(theta)) > ITER_MAX_ERROR)
        {
            // Keep averaging the arm angle and the trajectory angle until they converge
            double trajTheta = traj.getEntry(1);
            i++;
            double lastTheta = theta;
            debug("%.2f, %.2f", Math.toDegrees(lastTheta), trajTheta);
            // the new arm angle is the average between the trajectory angle and the arm angle
            theta = TrcUtil.average(theta, Math.toRadians(trajTheta));
            // recalculate the trajectory using the new arm angle
            relVertex = getVertexFromShooter(vertexFromPivot, theta);
            traj = TrajectoryCalculator.calculateWithVertexWithDrag(relVertex);
            debug(" -> %.2f, %.2f\n", Math.toDegrees(theta), traj.getEntry(1));
            // If the path is invalid or the new target angle is less than the last arm angle, use weighted average with binary fractions
            // Essentially, the theta must always be less than trajTheta. If it's not, the update increment must be smaller.
            // Since two continuous functions are heading in opposite directions and have points at either end, they must intersect at some point.
            int pow = 2; // start with weights of 3/4 and 1/4, since equal weights is just the average
            while (Double.isNaN(traj.getEntry(1)) || Math.toRadians(traj.getEntry(1)) < theta)
            {
                // theta is the weighted average of the last arm angle and the trajectory angle of that arm angle
                theta = (1 - Math.pow(2, -pow)) * lastTheta + Math.pow(2, -pow) * Math.toRadians(trajTheta);
                // recalculate the new trajectory angle from the new arm angle
                relVertex = getVertexFromShooter(vertexFromPivot, theta);
                traj = TrajectoryCalculator.calculateWithVertexWithDrag(relVertex);
                debug("pow=%d,theta=%.2f,trajTheta=%.2f\n", pow, Math.toDegrees(theta), traj.getEntry(1));
                // If this angle is still invalid, the update must be smaller
                pow++;
                if (pow >= 5)
                {
                    System.err.println("Pow is getting too small! Something went wrong!");
                    return null;
                }
            }
            // Prevent infinite loop
            if (i > MAX_ITER)
            {
                if (Math.abs(traj.getEntry(1) - Math.toDegrees(theta)) > ITER_OVERRUN_MAX_ERROR)
                {
                    System.err.println("Didn't converge fast enough!");
                    return null;
                }
                System.out.println("Overrun iteration count! Exiting early...");
                break;
            }
        }
        return traj;
    }

    /**
     * Calculate initial velocity and pitch in order to place the vertex of the trajectory at the supplied point.
     * This model does not account for air resistance.
     *
     * @param vertex 2d vector where the first entry is the robot y distance to the target, and the second is the robot
     *               z distance. Both distances should be in inches and relative to the ball exit point.
     * @return 2d vector where the first entry is initial speed and the second entry is pitch in degrees.
     */
    public static RealVector calculateWithVertexNoDrag(RealVector vertex)
    {
        vertex = vertex.mapMultiply(METERS_PER_INCH);
        double y = vertex.getEntry(0);
        double z = vertex.getEntry(1);
        // derived from basic kinematic equations
        double theta = Math.atan2(2 * z, y);
        double v = Math.sqrt(G * y * y / (2 * z * Math.pow(Math.cos(theta), 2)));
        return new ArrayRealVector(new double[] { v / METERS_PER_INCH, Math.toDegrees(theta) });
    }

    /**
     * Calculate initial velocity and pitch in order to place the vertex of the trajectory at the supplied point.
     * This model accounts for a linear model of air resistance. It's not exactly correct, but it's approximate.
     *
     * @param vertex 2d vector where the first entry is the robot y distance to the target, and the second is the robot
     *               z distance. Both distances should be in inches and relative to the ball exit point.
     * @return 2d vector where the first entry is initial speed and the second entry is pitch in degrees.
     */
    public static RealVector calculateWithVertexWithDrag(RealVector vertex)
    {
        // uses linear approximation of drag to avoid numerically solving the equation
        // then it uses a rational approximation of e^x to get a pretty accurate estimation of trajectory
        // math used from this research paper: https://www.hindawi.com/journals/ijcgt/2014/463489/
        vertex = vertex.mapMultiply(METERS_PER_INCH);
        double y = vertex.getEntry(0);
        double z = vertex.getEntry(1);
        double terminalVel = Math.sqrt(2 * M * G / (C * A * P)); // mg=1/2capv^2
        double k = G / (2 * terminalVel);
        double t = Math.sqrt(2 * z / G);
        double vy = y * (k + 1.0 / t);
        double vz = z * (k + 1.0 / t) + k * terminalVel * t;
        double v = TrcUtil.magnitude(vy, vz);
        double theta = Math.atan2(vz, vy);
        return new ArrayRealVector(new double[] { v / METERS_PER_INCH, Math.toDegrees(theta) });
    }
}
