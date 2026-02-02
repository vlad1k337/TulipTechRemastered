package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import dev.nextftc.core.commands.utility.InstantCommand;

public class Intake {
    private final DcMotorEx intake;

    public Intake(HardwareMap hardwareMap)
    {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        MotorConfigurationType intakeConfig = intake.getMotorType().clone();
        intakeConfig.setAchieveableMaxRPMFraction(1.0);
        intake.setMotorType(intakeConfig);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Our first driver can also activate intake when needed.
    public void update(Gamepad gamepad1, Gamepad gamepad2)
    {
        if(gamepad1.aWasPressed() || gamepad2.aWasPressed())
        {
            start();
        }

        if(gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            stop();
        }

        if(gamepad1.xWasPressed() || gamepad2.xWasPressed())
        {
            reverse();
        }
    }

    public void start()
    {
        intake.setPower(-1.0);
    }

    public void reverse() { intake.setPower(1.0);}

    public void stop()
    {
        intake.setPower(0.0);
    }

    public InstantCommand startCommand()
    {
        return new InstantCommand(this::start);
    }

    public InstantCommand stopCommand()
    {
        return new InstantCommand(this::stop);
    }
}

