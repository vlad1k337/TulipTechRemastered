package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import dev.nextftc.core.commands.utility.InstantCommand;
import smile.interpolation.LinearInterpolation;

public class Shooter {
    private final DcMotorEx flywheel;
    private final Servo hood;
    private final Servo gate;
    private final LED gateIndicator;
    private final LED flywheelIndicator;

    private LinearInterpolation hood_lut;
    private LinearInterpolation flywheel_lut;
    public static final double MID_LINE_VELOCITY = 1340;
    public static final double FAR_VELOCITY = 1700;

    private double hoodManualAdjustment = 0.0;
    private final double HOOD_NEAR = 0.42;
    private final double HOOD_FAR  = 0.53;

    private double targetVelocity = 0;
    private double targetGatePos  = HOOD_NEAR;

    private boolean flywheelOn = false;
    private boolean gateClosed = false;

    // kV = 1 / MAX_RPM, and adjusted for kS
    private final double kS = 0.049, kV = 0.00038, kP = 0.005;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "Shooter");
        MotorConfigurationType flywheelConfig = flywheel.getMotorType().clone();
        flywheelConfig.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(flywheelConfig);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        initVelocityLut();

        hood = hardwareMap.get(Servo.class, "Hood");
        gate = hardwareMap.get(Servo.class, "Gate");
        gateIndicator = hardwareMap.get(LED.class, "GateIndicator");
        flywheelIndicator = hardwareMap.get(LED.class, "FlywheelIndicator");
        gateClose();
    }

    public void update(Gamepad gamepad, double distance)
    {
        updateFeedforward();
        updateControls(gamepad, distance);
    }

    private void updateControls(Gamepad gamepad, double distance)
    {
        if(gamepad.dpadUpWasPressed())
        {
            flywheelOn = true;
        }

        if(gamepad.dpadDownWasPressed())
        {
            targetVelocity = 0;
            flywheelOn = false;
        }

        if(gamepad.dpadLeftWasPressed()) {
            hoodManualAdjustment += 0.005;
        } else if(gamepad.dpadRightWasPressed()) {
            hoodManualAdjustment -= 0.005;
        }

        if(gamepad.rightBumperWasPressed())
        {
            setVelocity(targetVelocity + 10);
        } else if(gamepad.leftBumperWasPressed()){
            setVelocity(targetVelocity - 10);
        }

        if(gamepad.left_trigger > 0.2)
        {
            gateClose();
        } else if(gamepad.right_trigger > 0.2) {
            gateOpen();
        }

        velocityRegression(distance);
        hood.setPosition(targetGatePos);
    }

    public void updateFeedforward()
    {
        flywheel.setPower((kV * targetVelocity) + (kP * (targetVelocity - flywheel.getVelocity())) + kS);
        if(targetVelocity > 0.0)
        {
            flywheelIndicator.on();
        } else {
            flywheelIndicator.off();
        }
    }

    public void updateTelemetry(TelemetryManager telemetry)
    {
        telemetry.addData("Target velocity", targetVelocity);
        telemetry.addData("Actual Velocity", flywheel.getVelocity());
        telemetry.addData("Hood Position", targetGatePos);
        telemetry.addData("Gate Closed", gateClosed);
    }

    public void setVelocity(double target)
    {
        targetVelocity = target;
    }

    public void hoodNear()
    {
        targetGatePos = HOOD_NEAR;
        hood.setPosition(targetGatePos);
    }

    public void hoodFar()
    {
        targetGatePos = HOOD_FAR;
        hood.setPosition(targetGatePos);
    }

    public void gateOpen()
    {
        gateClosed = true;
        gate.setPosition(0.0);
        gateIndicator.on();
    }

    public void gateClose()
    {
        gateClosed = false;
        gate.setPosition(1.0);
        gateIndicator.off();
    }

    public InstantCommand gateOpenCommand()
    {
        return new InstantCommand(this::gateOpen);
    }

    public InstantCommand gateCloseCommand()
    {
        return new InstantCommand(this::gateClose);
    }

    public InstantCommand hoodFarCommand()
    {
        return new InstantCommand(this::hoodFar);
    }

    public InstantCommand hoodNearCommand()
    {
        return new InstantCommand(this::hoodNear);
    }

    public void hoodRegression(double distance)
    {
        targetGatePos = hood_lut.interpolate(distance) + hoodManualAdjustment;
    }

    public void initVelocityLut()
    {
        double[] distances = {60, 52, 68, 65, 71, 78, 54, 57, 41, 46, 37, 89, 77, 84, 82};
        double[] hood_positions = {0.43, 0.415, 0.435, 0.435, 0.43, 0.435, 0.415, 0.415, 0.42, 0.425, 0.41, 0.45, 0.44, 0.45, 0.45};
        double[] velocities = {1340, 1320, 1400, 1360, 1410, 1470, 1320, 1350, 1310, 1320, 1290, 1520, 1500, 1450, 1450};

        hood_lut = new LinearInterpolation(distances, hood_positions);
        flywheel_lut = new LinearInterpolation(distances, velocities);
    }

    public void velocityRegression(double distance)
    {
        if(!flywheelOn)
        {
            return;
        }

        targetVelocity = flywheel_lut.interpolate(distance);
    }
}
