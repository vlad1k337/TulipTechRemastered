package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PathsRed {
    public static final Pose startPose = new Pose(124, 121, Math.toRadians(90));
    final Pose shootingPose = new Pose(101, 100, Math.toRadians(47));

    final Pose PPG = new Pose(100, 83, Math.toRadians(0));
    final Pose PGP = new Pose(100, 59, Math.toRadians(0));
    final Pose GPP = new Pose(100, 36, Math.toRadians(0));

    final Pose IntakePPG = new Pose(120, 83, Math.toRadians(0));
    final Pose IntakePGP = new Pose(128, 59, Math.toRadians(0));
    final Pose IntakeGPP = new Pose(125, 36, Math.toRadians(0));
    final Pose Gate      = new Pose(133, 71, Math.toRadians(0));
    final Pose GateFarm  = new Pose(132, 60, Math.toRadians(30));

    public PathChain startToShoot;
    public PathChain moveToPPG, moveToIntakePPG, shootPPG;
    public PathChain moveToPGP, moveToIntakePGP, shootPGP;
    public PathChain moveToGPP, moveToIntakeGPP, shootGPP;
    public PathChain openGate, farmGate, shootGate;

    public PathsRed(Follower follower)
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
                .build();

        shootPGP = follower.pathBuilder()
                .addPath(new BezierLine(IntakePGP, shootingPose))
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

        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(IntakePGP, new Pose(95, 65), Gate))
                .setConstantHeadingInterpolation(0)
                .build();

        farmGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose, new Pose(106, 65), GateFarm))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), GateFarm.getHeading())
                .build();

        shootGate = follower.pathBuilder()
                .addPath(new BezierCurve(GateFarm, new Pose(106, 65), shootingPose))
                .setLinearHeadingInterpolation(GateFarm.getHeading(), shootingPose.getHeading())
                .build();
    }
}
