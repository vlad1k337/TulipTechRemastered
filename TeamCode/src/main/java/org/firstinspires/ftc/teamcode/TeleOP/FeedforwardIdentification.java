package org.firstinspires.ftc.teamcode.TeleOP;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import kotlin.collections.ArrayDeque;

@TeleOp(name = "FeedforwardID")
public class
FeedforwardIdentification extends OpMode {
    private TelemetryManager telemetryM;

    private DcMotorEx flywheel;

    private double targetPower = 0.0;
    private double deltaPower = 0.01;

    private ArrayList<Double> powers;
    private ArrayList<Double> velocities;

    private ElapsedTime spinupTimer;

    @Override
    public void init()
    {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        flywheel = hardwareMap.get(DcMotorEx.class, "Shooter");
        MotorConfigurationType flywheelConfig = flywheel.getMotorType().clone();
        flywheelConfig.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(flywheelConfig);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        powers = new ArrayList<>();
        velocities = new ArrayList<>();

        spinupTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        spinupTimer.reset();
    }

    @Override
    public void loop()
    {
        for(int i = 0; i < powers.toArray().length; ++i)
        {
            telemetryM.debug(powers.get(i));
            telemetryM.debug(velocities.get(i));
            telemetryM.debug(" ");
        }

        telemetryM.update(telemetry);

        if(targetPower >= 1.0)
        {
            deltaPower = 0.0;
        }

        flywheel.setPower(targetPower);
        if(spinupTimer.milliseconds() > 2000)
        {
            powers.add(targetPower);
            velocities.add(flywheel.getVelocity());
            targetPower += deltaPower;

            spinupTimer.reset();
        }
    }
}