package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PathsBlue {
    public static final Pose startPose = new Pose(20, 121, Math.toRadians(90));
    final Pose shootingPose = new Pose(43, 100, Math.toRadians(138));

    final Pose PPG = new Pose(89, 83, Math.toRadians(0)).mirror();
    final Pose PGP = new Pose(89, 59, Math.toRadians(0)).mirror();
    final Pose GPP = new Pose(89, 36, Math.toRadians(0)).mirror();

    final Pose IntakePPG = new Pose(120, 83, Math.toRadians(0)).mirror();
    final Pose IntakePGP = new Pose(125, 59, Math.toRadians(0)).mirror();
    final Pose IntakeGPP = new Pose(125, 36, Math.toRadians(0)).mirror();
    final Pose Gate      = new Pose(130, 70, Math.toRadians(0)).mirror();

    public PathChain startToShoot;
    public PathChain moveToPPG, moveToIntakePPG, shootPPG;
    public PathChain moveToPGP, moveToIntakePGP, shootPGP;
    public PathChain moveToGPP, moveToIntakeGPP, shootGPP;

    public PathsBlue(Follower follower)
    {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        moveToPPG = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, PPG))
                .setConstantHeadingInterpolation(PPG.getHeading())
                .build();

        moveToIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine(PPG, IntakePPG))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootPPG = follower.pathBuilder()
                .addPath(new BezierLine(IntakePPG, shootingPose))
                .setLinearHeadingInterpolation(IntakePPG.getHeading(), shootingPose.getHeading())
                .build();

        moveToPGP = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, PGP))
                .setConstantHeadingInterpolation(PGP.getHeading())
                .build();

        moveToIntakePGP = follower.pathBuilder()
                .addPath(new BezierLine(PGP, IntakePGP))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootPGP = follower.pathBuilder()
                .addPath(new BezierCurve(IntakePGP, GPP, shootingPose))
                .setLinearHeadingInterpolation(IntakePGP.getHeading(), shootingPose.getHeading())
                .build();

        moveToGPP = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, GPP))
                .setConstantHeadingInterpolation(GPP.getHeading())
                .build();

        moveToIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPP, IntakeGPP))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootGPP = follower.pathBuilder()
                .addPath(new BezierLine(IntakeGPP, shootingPose))
                .setLinearHeadingInterpolation(IntakeGPP.getHeading(), shootingPose.getHeading())
                .build();
    }
}
