package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystem.LimelightSubsystem;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            //.predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.2, 0.08641, 0.002158))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0.0, 0.1, 0.02))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0.0, 0.01, 0.015))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0.0, 0.0025,0.6, 0.01))
            .centripetalScaling(0.001)
            .forwardZeroPowerAcceleration(-35.2984560)
            .lateralZeroPowerAcceleration(-47.7576);

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1.5, 1.0);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .useBrakeModeInTeleOp(true)
            .maxPower(1.0)
            .xVelocity(78.42)
            .yVelocity(64.836)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("FL")
            .leftFrontMotorName("BL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants localizerPinpointConstants = new PinpointConstants()
            .forwardPodY(-3.3)
            .strafePodX(-6.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        PinpointLocalizer pinpointLocalizer = new PinpointLocalizer(hardwareMap, localizerPinpointConstants);

        LimelightSubsystem.fusionLocalizer = new FusionLocalizer(
                pinpointLocalizer,
                new Pose(0.5, 0.5, 0.05),
                new Pose(1.0, 1.0, 0.1),
                new Pose(4.0, 4.0, 0.04),
                100
        );

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerPinpointConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}