package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.FusionLocalizer;

import java.util.List;

public class LimelightSubsystem {
    private final Limelight3A limelight;
    private Pose lastPose = new Pose(0, 0, 0);

    public static FusionLocalizer fusionLocalizer = null;

    public LimelightSubsystem(HardwareMap hardwareMap)
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(20);
        limelight.start();
    }

    public Pose lowPassFilter(Pose newPose, double smoothingFactor)
    {
        if(lastPose.equals(new Pose(0, 0, 0)))
        {
            return newPose;
        }

        double newX = MathUtilities.lowPass(newPose.getX(), lastPose.getX(), smoothingFactor);
        double newY = MathUtilities.lowPass(newPose.getY(), lastPose.getY(), smoothingFactor);
        double newHeading = MathUtilities.lowPass(newPose.getHeading(), lastPose.getHeading(), smoothingFactor);

        return new Pose(newX, newY, newHeading);
    }

    public Pose positionFromTag()
    {
        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid())
        {
            Pose3D robotPos = result.getBotpose();

            double angle = robotPos.getOrientation().getYaw(AngleUnit.DEGREES) - 90;

            return new Pose(
                    robotPos.getPosition().y / 0.0254 + 70.625,
                    -robotPos.getPosition().x / 0.0254 + 70.625,
                    Math.toRadians(angle));
        }

        return null;
    }

    public double angleFromTag()
    {
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();

        LLResultTypes.FiducialResult target = null;
        for(LLResultTypes.FiducialResult result : results)
        {
            if(result != null && result.getFiducialId() == 20)
            {
                target = result;
                break;
            }
        }

        if(target == null)
        {
            return 0;
        }

        return target.getTargetXDegrees();
    }
}
