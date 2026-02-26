package org.firstinspires.ftc.teamcode.TeleOP;

import static org.firstinspires.ftc.teamcode.Subsystem.LimelightSubsystem.fusionLocalizer;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystem.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.MathUtilities;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.SlewRateLimiter;

@TeleOp(name = "Tulip1P")
public class Tulip1P extends OpMode {
    private Shooter shooter;
    private Intake intake;
    private LimelightSubsystem limelight;

    private SlewRateLimiter forwardLimiter, strafeLimiter, headingLimiter;
    private Follower follower;
    private TelemetryManager telemetryM;

    private PIDFController headingController;
    private boolean headingLock = false;
    private double targetHeading;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        forwardLimiter = new SlewRateLimiter(0.5);
        strafeLimiter = new SlewRateLimiter(0.5);
        headingLimiter = new SlewRateLimiter(0.3);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();

        limelight = new LimelightSubsystem(hardwareMap);

        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        shooter   = new Shooter(hardwareMap);
        intake    = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    private double getHeadingError()
    {
        if(follower.getCurrentPath() != null)
        {
            return 0;
        }

        targetHeading = Math.atan2(134 - follower.getPose().getY(), 11 - follower.getPose().getX());

        return MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading)
                * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
    }

    private void updateDrive(Gamepad gamepad) {
        headingController.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        headingController.updateError(getHeadingError());

        if(gamepad.leftStickButtonWasPressed())
        {
            Pose limelightPose = limelight.positionFromTag();
            if(limelightPose != null)
            {
                follower.setPose(limelightPose);
            }
        }

        if (gamepad.yWasPressed()) {
            headingLock = !headingLock;
        }

        if (headingLock) {
            follower.setTeleOpDrive(
                    MathUtilities.expo(-gamepad.left_stick_y),
                    MathUtilities.expo(-gamepad.left_stick_x),
                    headingController.run()
            );
        } else {
            follower.setTeleOpDrive(
                    MathUtilities.expo(forwardLimiter.calculate(-gamepad.left_stick_y)),
                    MathUtilities.expo(strafeLimiter.calculate(-gamepad.left_stick_x)),
                    MathUtilities.expo(headingLimiter.calculate(-gamepad.right_stick_x))
            );
        }

        follower.update();
    }

    private void updateTelemetry()
    {
        shooter.updateTelemetry(telemetryM);
        telemetryM.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetryM.addData("Target Heading", Math.toDegrees(targetHeading));
        telemetryM.addData("X", follower.getPose().getX());
        telemetryM.addData("Y", follower.getPose().getY());
        telemetryM.addData("Angular Velocity", Math.toDegrees(follower.getAngularVelocity()));
        telemetryM.addData("Velocity", follower.getVelocity().getMagnitude());
        telemetryM.addData("Distance to (144, 144)", MathUtilities.calculateDistance(11, 134, follower.getPose().getX(), follower.getPose().getY()));
        telemetryM.addData("Angle from tag", limelight.angleFromTag());
        telemetryM.update(telemetry);
    }

    @Override
    public void loop()
    {
        updateDrive(gamepad1);

        double distance = MathUtilities.calculateDistance(11, 134, follower.getPose().getX(), follower.getPose().getY());
        shooter.update(gamepad1, distance);
        shooter.hoodRegression(distance);
        intake.update(gamepad1, gamepad2);

        updateTelemetry();
    }
}