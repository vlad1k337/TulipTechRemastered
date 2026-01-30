package org.firstinspires.ftc.teamcode.Subsystem;

public final class MathUtilities {
    public static double expo(double input)
    {
        return Math.signum(input) * Math.pow(Math.abs(input), 1.4);
    }

    // Utility distance calculations, written for applying linear regression on shooting velocity.
    public static double calculateDistance(double x1, double y1, double x2, double y2) {
        double xDifference = x2 - x1;
        double yDifference = y2 - y1;
        double distanceSquared = Math.pow(xDifference, 2) + Math.pow(yDifference, 2);
        return Math.sqrt(distanceSquared);
    }
}
