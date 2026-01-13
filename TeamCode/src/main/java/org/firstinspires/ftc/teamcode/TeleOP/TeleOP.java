package org.firstinspires.ftc.teamcode.TeleOP;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "TeleOP")
public class TeleOP extends OpMode {
    List<LynxModule> allHubs;

    private Follower follower;

    private Shooter shooter;
    private Intake intake;

    @Override
    public void init()
    {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    private void updateDrive(Gamepad gamepad)
    {
        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x,
                true
        );
    }

    @Override
    public void loop()
    {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        updateDrive(gamepad1);

        shooter.update(gamepad1);
        intake.update(gamepad1, gamepad2);

        follower.update();
    }
}
