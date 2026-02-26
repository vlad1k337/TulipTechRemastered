package org.firstinspires.ftc.teamcode.Autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Paths.FarPathsRed;
import org.firstinspires.ftc.teamcode.Paths.PathsBlue;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseHolder;

import java.time.Instant;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "RedFar9")
public class RedFar9 extends NextFTCOpMode {
    private final double TIME_TO_SHOOT_PRELOAD = 3.0;

    private FarPathsRed paths;
    private Shooter shooter;
    private Intake intake;

    public RedFar9()
    {
        addComponents(
                new PedroComponent(Constants::createFollower));
    }

    private SequentialGroup autonomousRoutine()
    {
        PedroComponent.follower().setStartingPose(FarPathsRed.startPose);

        paths = new FarPathsRed(PedroComponent.follower());

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        InstantCommand setHood = new InstantCommand(() -> {
            shooter.hoodFar();
        });

        InstantCommand prepareShooters = new InstantCommand(() -> {
            shooter.gateClose();
            shooter.setVelocity(Shooter.FAR_VELOCITY);
        });

        InstantCommand stopShooter = new InstantCommand(() -> {
            shooter.gateClose();
            shooter.setVelocity(0);
            intake.stop();
        });

        SequentialGroup shoot = new SequentialGroup(
                shooter.gateOpenCommand(),
                new Delay(0.5),

                intake.startCommand(),
                new Delay(0.15),
                shooter.gateCloseCommand(),
                intake.stopCommand(),
                new Delay(0.5),
                shooter.gateOpenCommand(),
                new Delay(0.4),

                intake.startCommand(),
                new Delay(0.15),
                shooter.gateCloseCommand(),
                intake.stopCommand(),
                new Delay(0.5),
                shooter.gateOpenCommand(),
                new Delay(0.4),

                intake.startCommand(),
                new Delay(0.15),
                shooter.gateCloseCommand(),
                intake.stopCommand()
        );

        InstantCommand stopIntake = new InstantCommand(() -> {
            intake.stop();
        });

        return new SequentialGroup(
                // Start shooters, set the hood
                setHood,
                prepareShooters.then(
                        new Delay(1.0)
                ),
                // Turn
                new FollowPath(paths.turnToShoot),
                // Shoot
                new ParallelGroup(
                        shoot,
                        new Delay(TIME_TO_SHOOT_PRELOAD)
                ),
                stopShooter,

                // Start intake and move to the pose
                intake.startCommand(),
                new FollowPath(paths.intakePGP),

                // Go back to shooting pose
                new FollowPath(paths.shootPGP),
                prepareShooters.then(
                        stopIntake,
                        new Delay(2.0)
                ),

                // Shoot
                new ParallelGroup(
                        shoot,
                        new Delay(TIME_TO_SHOOT_PRELOAD)
                ),
                stopShooter,

                // Start intake and get GPP
                intake.startCommand(),
                new FollowPath(paths.intakeGPP),

                new FollowPath(paths.shootGPP),
                prepareShooters.then(
                        stopIntake,
                        new Delay(2.0)
                ),

                // Shoot
                new ParallelGroup(
                        shoot,
                        new Delay(TIME_TO_SHOOT_PRELOAD)
                ),
                stopShooter,

                // Park
                new Delay(5),
                new FollowPath(paths.leave)
        );
    }

    @Override
    public void onStartButtonPressed()
    {
        SequentialGroup autoCommands = autonomousRoutine();
        autoCommands.schedule();
    }

    @Override
    public void onUpdate()
    {
        CommandManager.INSTANCE.run();
        PoseHolder.position = PedroComponent.follower().getPose();

        shooter.updateFeedforward();
    }

    @Override
    public void onStop()
    {
        PoseHolder.position = PedroComponent.follower().getPose();
    }
}