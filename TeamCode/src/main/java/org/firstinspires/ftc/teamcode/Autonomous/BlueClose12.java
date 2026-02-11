package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Paths.PathsBlue;
import org.firstinspires.ftc.teamcode.Paths.PathsRed;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseHolder;

import java.nio.file.Paths;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "BlueClose12")
public class BlueClose12 extends NextFTCOpMode {
    // Pretty self-explanatory, mess around with this values if the robot takes too much time shooting
    // Delay is always in seconds.
    private static final double TIME_TO_SHOOT_PRELOAD = 2;
    private static final double TIME_TO_SHOOT_PPG = 2;
    private static final double TIME_TO_SHOOT_PGP = 2;
    private static final double TIME_TO_SHOOT_GPP = 2;

    private PathsBlue paths;

    private Shooter shooter;
    private Intake intake;

    // Let NextFTC know about Pedro
    public BlueClose12()
    {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    private SequentialGroup autonomousRoutine()
    {
        PedroComponent.follower().setStartingPose(PathsBlue.startPose);

        paths = new PathsBlue(PedroComponent.follower());

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        InstantCommand setHood = new InstantCommand(() -> {
            shooter.hoodNear();
        });

        InstantCommand prepareShooters = new InstantCommand(() -> {
            shooter.gateClose();
            shooter.setVelocity(Shooter.MID_LINE_VELOCITY);
        });

        InstantCommand stopShooter = new InstantCommand(() -> {
            shooter.gateClose();
            shooter.setVelocity(0);
            intake.stop();
        });

        SequentialGroup shoot = new SequentialGroup(
                intake.stopCommand(),
                shooter.gateOpenCommand(),
                new Delay(0.5),
                intake.startCommand()
        );

        InstantCommand stopIntake = new InstantCommand(() -> {
            intake.stop();
        });

        return new SequentialGroup(
                // Score preloads
                setHood,
                prepareShooters,
                new FollowPath(paths.startToShoot).then(
                        new Delay(0.5)
                ),
                new ParallelGroup(
                        shoot,
                        new Delay(TIME_TO_SHOOT_PRELOAD)
                ),
                stopShooter,

                // Intake and score PPG
                new FollowPath(paths.moveToPPG).then(
                        intake.startCommand()
                ),
                new FollowPath(paths.moveToIntakePPG).then(
                        prepareShooters
                ),

                new FollowPath((paths.shootPPG)).then(
                        new Delay(0.5)
                ),
                new ParallelGroup(
                        shoot,
                        new Delay(TIME_TO_SHOOT_PPG)
                ),
                stopShooter,

                // Intake and score PGP
                new FollowPath(paths.moveToPGP).then(
                        intake.startCommand()
                ),
                new FollowPath(paths.moveToIntakePGP).then(
                        prepareShooters
                ),

                new FollowPath((paths.shootPGP)).then(
                        new Delay(0.5)
                ),
                new ParallelGroup(
                        shoot,
                        new Delay(TIME_TO_SHOOT_PGP)
                ),
                stopShooter,

                // Intake and score GPP
                new FollowPath(paths.moveToGPP).then(
                        intake.startCommand()
                ),
                new FollowPath(paths.moveToIntakeGPP).then(
                        prepareShooters
                ),

                new FollowPath((paths.shootGPP)).then(
                        new Delay(0.5)
                ),
                new ParallelGroup(
                        shoot,
                        new Delay(TIME_TO_SHOOT_GPP)
                ),
                stopShooter,

                // Park
                new FollowPath(paths.moveToPGP)
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
