package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Shooter {
    // Velocity required to shoot from the middle of a white line.
    // Decrease it if robot is overshooting, increase if it's undershooting..

    private DcMotorEx flywheel;

    public static final double MID_LINE_VELOCITY = 1320;
    private double targetVelocity = 0;

    // kV = 1 / MAX_RPM, and adjusted for kS
    public double kS = 0.06, kV = 0.00035, kP = 0.005;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "Shooter");
        MotorConfigurationType flywheelConfig = flywheel.getMotorType().clone();
        flywheelConfig.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(flywheelConfig);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void update(Gamepad gamepad)
    {
        updateFeedforward();
        updateControls(gamepad);
    }

    private void updateControls(Gamepad gamepad)
    {
        if(gamepad.xWasPressed())
        {
            targetVelocity = MID_LINE_VELOCITY;
        }

        if(gamepad.yWasPressed())
        {
            targetVelocity = 0;
        }
    }

    public void updateFeedforward()
    {
        flywheel.setPower((kV * targetVelocity) + (kP * (targetVelocity - flywheel.getVelocity())) + kS);
    }

    public void updateTelemetry(TelemetryManager telemetry)
    {
        telemetry.addData("Target velocity", targetVelocity);
        telemetry.addData("Actual Velocity", flywheel.getVelocity());
    }

    public void setVelocity(double target)
    {
        targetVelocity = target;
    }

    public void gateClose() {

    }

    public void gateOpen() {

    }
}
