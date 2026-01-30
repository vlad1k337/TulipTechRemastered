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

public class Shooter {
    private final DcMotorEx flywheel;
    private final Servo hood;
    private final Servo gate;
    private final LED gateIndicator;

    public static final double MID_LINE_VELOCITY = 1370;
    public static final double FAR_VELOCITY = 1800;

    private double hoodManualAdjustment = 0.0;
    private double HOOD_NEAR = 0.41;
    private double HOOD_FAR  = 0.53;

    private double targetVelocity = 0;
    private double targetGatePos  = HOOD_NEAR;

    private boolean gateClosed = false;

    // kV = 1 / MAX_RPM, and adjusted for kS
    public double kS = 0.061, kV = 0.00038, kP = 0.01;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "Shooter");
        MotorConfigurationType flywheelConfig = flywheel.getMotorType().clone();
        flywheelConfig.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(flywheelConfig);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "Hood");
        gate = hardwareMap.get(Servo.class, "Gate");
        gateIndicator = hardwareMap.get(LED.class, "GateIndicator");
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
            targetVelocity = MID_LINE_VELOCITY;
            gateClose();
        }

        if(gamepad.dpadDownWasPressed())
        {
            targetVelocity = 0;
        }

        if(gamepad.dpadLeftWasPressed()) {
            hoodManualAdjustment += 0.025;
        } else if(gamepad.dpadRightWasPressed()) {
            hoodManualAdjustment -= 0.025;
        }

        if(gamepad.rightBumperWasPressed())
        {
            setVelocity(targetVelocity + 50);
        } else if(gamepad.leftBumperWasPressed()){
            setVelocity(targetVelocity - 50);
        }

        if(gamepad.rightStickButtonWasPressed())
        {
            hoodRegression(distance);
        } else if(gamepad.leftStickButtonWasPressed()) {
            hoodNear();
        }

        if(gamepad.left_trigger > 0.2)
        {
            gateClose();
        } else if(gamepad.right_trigger > 0.2) {
            gateOpen();
        }

        hood.setPosition(targetGatePos);
    }

    public void updateFeedforward()
    {
        flywheel.setPower((kV * targetVelocity) + (kP * (targetVelocity - flywheel.getVelocity())) + kS);
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
        gate.setPosition(-1.0);
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
        targetGatePos = -0.0000751855 * distance * distance + 0.0132971 * distance - 0.142975 + hoodManualAdjustment;
    }

    public void velocityRegression(double distance)
    {
        targetVelocity = -0.0773848 * distance * distance + 14.4288 * distance + 724.07684;
    }
}
