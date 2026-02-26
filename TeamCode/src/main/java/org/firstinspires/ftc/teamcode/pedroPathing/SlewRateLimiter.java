package org.firstinspires.ftc.teamcode.pedroPathing;

import com.arcrobotics.ftclib.util.MathUtils;

public class SlewRateLimiter {
    private double rateLimit;
    private double previousValue;

    public SlewRateLimiter(double rateLimit)
    {
        this.rateLimit = rateLimit;
        this.previousValue = 0;
    }

    public double calculate(double input)
    {
        previousValue += MathUtils.clamp(input - previousValue, -rateLimit, rateLimit);
        return previousValue;
    }
}
