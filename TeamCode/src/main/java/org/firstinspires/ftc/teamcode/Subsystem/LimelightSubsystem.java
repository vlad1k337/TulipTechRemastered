package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
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
    private Limelight3A limelight;
    private Pose lastPose = new Pose(0, 0, 0);

    public static FusionLocalizer fusionLocalizer = null;

    public LimelightSubsystem(HardwareMap hardwareMap)
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(20);
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public Pose lowPassFilter(Pose newPose, double smoothingFactor)
    {
        double newX = smoothingFactor * newPose.getX() + (1 - smoothingFactor) * lastPose.getX();
        double newY = smoothingFactor * newPose.getY() + (1 - smoothingFactor) * lastPose.getY();
        double newHeading = smoothingFactor * newPose.getHeading() + (1 - smoothingFactor) * lastPose.getHeading();

        return new Pose(newX, newY, newHeading);
    }

    public Pose positionFromTag(double heading, TelemetryManager telemetry)
    {
        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                Pose2D botpose2D = new Pose2D(
                        DistanceUnit.INCH,
                        botpose.getPosition().toUnit(DistanceUnit.INCH).x,
                        botpose.getPosition().toUnit(DistanceUnit.INCH).y,
                        AngleUnit.DEGREES,
                        botpose.getOrientation().getYaw()
                );

                Pose botposeFTC   = PoseConverter.pose2DToPose(botpose2D, InvertedFTCCoordinates.INSTANCE);
                Pose botposePedro = botposeFTC.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                telemetry.addData("MT2 Pedro X", botposePedro.getX());
                telemetry.addData("MT2 Pedro Y", botposePedro.getY());

                Pose filteredPose = lowPassFilter(botposePedro, 0.5);
                telemetry.addData("Filtered Lime X", filteredPose.getX());
                telemetry.addData("Filtered Lime Y", filteredPose.getY());

                lastPose = filteredPose;
                return filteredPose;
            }
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
