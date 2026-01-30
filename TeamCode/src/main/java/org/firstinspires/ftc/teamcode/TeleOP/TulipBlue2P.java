package org.firstinspires.ftc.teamcode.TeleOP;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystem.MathUtilities;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;

// Test OpMode for one driver to have all the controls
@TeleOp(name = "TulipBlue2P")
public class TulipBlue2P extends OpMode {
    private Shooter shooter;
    private Intake intake;

    private Follower follower;
    private TelemetryManager telemetryM;

    private final Pose startingPose = new Pose(89, 59, Math.toRadians(0)).mirror();

    private PIDFController headingController;
    private boolean headingLock = false;
    private double targetHeading;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

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

        targetHeading = Math.atan2(follower.getPose().getX(), follower.getPose().getY()) + Math.toRadians(90);

        return MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading)
                * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);
    }

    private void updateDrive(Gamepad gamepad) {
        headingController.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        headingController.updateError(getHeadingError());

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
                    MathUtilities.expo(-gamepad.left_stick_y),
                    MathUtilities.expo(-gamepad.left_stick_x),
                    MathUtilities.expo(-gamepad.right_stick_x)
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
        telemetryM.addData("Distance to (0, 144)", MathUtilities.calculateDistance(0, 144, follower.getPose().getX(), follower.getPose().getY()));
        telemetryM.update(telemetry);
    }

    @Override
    public void loop()
    {
        updateDrive(gamepad1);

        shooter.update(gamepad2, MathUtilities.calculateDistance(0, 135, follower.getPose().getX(), follower.getPose().getY()));
        shooter.hoodRegression(MathUtilities.calculateDistance(0 , 135, follower.getPose().getX(), follower.getPose().getY()));
        intake.update(gamepad1, gamepad2);

        updateTelemetry();
    }
}