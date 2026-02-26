package org.firstinspires.ftc.teamcode.Subsystem;

import com.pedropathing.geometry.Pose;

public final class MathUtilities {
    public static double expo(double input)
    {
        return Math.signum(input) * Math.pow(Math.abs(input), 1.4);
    }

    public static double expo(double input, double expo)
    {
        return Math.signum(input) * Math.pow(Math.abs(input), expo);
    }

    // Utility distance calculations, written for applying linear regression on shooting velocity.
    public static double calculateDistance(double x1, double y1, double x2, double y2) {
        double xDifference = x2 - x1;
        double yDifference = y2 - y1;
        double distanceSquared = Math.pow(xDifference, 2) + Math.pow(yDifference, 2);
        return Math.sqrt(distanceSquared);
    }

    public static double lowPass(double currentValue, double lastValue, double smoothingFactor)
    {
        return smoothingFactor * currentValue + (1 - smoothingFactor) * lastValue;
    }

    public static Pose blend(Pose odometryPose, Pose limelight_pose)
    {
        // newPose = odometryPose + alpha * (limelight_pose - odometryPose)
        Pose error = limelight_pose.minus(odometryPose);
        Pose partial = error.scale(0.7);
        Pose blended = odometryPose.plus(partial);

        return blended;
    }
}
