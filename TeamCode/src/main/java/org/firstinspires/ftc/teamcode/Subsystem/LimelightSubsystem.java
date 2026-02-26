package org.firstinspires.ftc.teamcode.Subsystem;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.FusionLocalizer;

import java.util.List;

public class LimelightSubsystem {
    private final Limelight3A limelight;

    public static FusionLocalizer fusionLocalizer = null;

    public LimelightSubsystem(HardwareMap hardwareMap)
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(20);
        limelight.start();
    }

    // Props to team Gyrobotic Droids for helping with relocalization code!
    // We appreciate your desire to help!
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
