package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarPathsBlue {
    public static final Pose startPose = new Pose(80, 9, Math.toRadians(90)).mirror();
    final Pose shootingPose = new Pose(85, 21, Math.toRadians(68)).mirror();

    final Pose startPGP  = new Pose(132, 21,  Math.toRadians(300)).mirror();
    final Pose endPGP = new Pose(133, 12, Math.toRadians(320)).mirror();
    final Pose parkPose  = new Pose(85, 37, Math.toRadians(90)).mirror();

    public PathChain turnToShoot;
    public PathChain intakePGP;
    public PathChain shootPGP;
    public PathChain leave;

    public FarPathsBlue(Follower follower)
    {
        turnToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .build();

        intakePGP = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, startPGP))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), startPGP.getHeading())
                .addPath(new BezierLine(startPGP, endPGP))
                .setLinearHeadingInterpolation(startPGP.getHeading(), endPGP.getHeading())
                .build();

        shootPGP = follower.pathBuilder()
                .addPath(new BezierLine(endPGP, shootingPose))
                .setLinearHeadingInterpolation(endPGP.getHeading(), shootingPose.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();
    }
}
