package org.firstinspires.ftc.teamcode.TeleOP;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Shooter;

@TeleOp
public class FlywheelTuner extends OpMode {
    private TelemetryManager telemetryM;
    private Shooter shooter;

    // 0 for kS, 1 for kV, 2 for kP
    int chosenCoefficient = 0;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void start()
    {
        telemetryM.debug("This is shooter feedforward coefficients tuner");
        telemetryM.debug("Press dpad right to cycle between kS, kV and kP");
        telemetryM.debug("Use left/right bumpers to decrease/increase coefficients");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        shooter.update(gamepad1);
        shooter.updateTelemetry(telemetryM);

        telemetryM.addData("kS", shooter.kS);
        telemetryM.addData("kV", shooter.kV);
        telemetryM.addData("kP", shooter.kP);

        telemetryM.update(telemetry);

        if(gamepad1.dpadRightWasPressed())
        {
            chosenCoefficient += 1;
            chosenCoefficient %= 3;
        }

        if(gamepad1.rightBumperWasPressed())
        {
            switch(chosenCoefficient)
            {
                case 0: shooter.kS += 0.001; break;
                case 1: shooter.kV += 0.001; break;
                case 2: shooter.kP += 0.001; break;
            }
        }

        if(gamepad1.leftBumperWasPressed())
        {
            switch(chosenCoefficient)
            {
                case 0: shooter.kS -= 0.001; break;
                case 1: shooter.kV -= 0.001; break;
                case 2: shooter.kP -= 0.001; break;
            }
        }
    }
}