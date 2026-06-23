package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.lib.math.LinearFilter;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Vision extends WSubsystem {
    public static final int NO_FIDUCIAL_ID_FILTER = -1;
    public Limelight3A LL3;
    public double tx;
    public double ty;
    public boolean tv;
    public double tl;
    LLResult results;
    Pose2d TargetCamera = new Pose2d();
    public int ApriltagID = NO_FIDUCIAL_ID_FILTER;
    private int targetFiducialId = NO_FIDUCIAL_ID_FILTER;
    private double targetDistanceInches = Double.NaN;
    LinearFilter filter;

    public void setTargetFiducialId(int fiducialId) {
        targetFiducialId = fiducialId;
    }

    public void clearTargetFiducialId() {
        targetFiducialId = NO_FIDUCIAL_ID_FILTER;
    }

    public int getTargetFiducialId() {
        return targetFiducialId;
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        LL3 = hardwareMap.get(Limelight3A.class,"limelight");
//        if(Superstructure.IsBlue){
//            LL3.pipelineSwitch(0);
//        }else{
//            LL3.pipelineSwitch(1);
//        }
        LL3.start();
        filter = new LinearFilter(3);
    }

    @Override
    public void periodic() {}

    @Override
    public void read() {
        this.results=LL3.getLatestResult();
        this.tv = false;
        this.ApriltagID = NO_FIDUCIAL_ID_FILTER;
        this.targetDistanceInches = Double.NaN;

        if(results!=null){
            this.tl =results.getTargetingLatency()/100;
            if(results.isValid()){
                if (targetFiducialId == NO_FIDUCIAL_ID_FILTER) {
                    this.tx = results.getTx();
                    this.ty = results.getTy();
                    this.tv = true;
                }

                LLResultTypes.FiducialResult tag = getTargetFiducial(results);

                if (tag != null) {
                    if (targetFiducialId != NO_FIDUCIAL_ID_FILTER) {
                        this.tx = tag.getTargetXDegrees();
                        this.ty = tag.getTargetYDegrees();
                    }
                    this.tv = true;
                    this.ApriltagID = tag.getFiducialId();

                    Pose3D TargetPoseCamera = tag.getTargetPoseCameraSpace();
                    if (TargetPoseCamera != null) {
                        double targetXInches = TargetPoseCamera.getPosition().toUnit(DistanceUnit.INCH).x;
                        double targetYInches = TargetPoseCamera.getPosition().toUnit(DistanceUnit.INCH).y;
                        double targetZInches = TargetPoseCamera.getPosition().toUnit(DistanceUnit.INCH).z;
                        targetDistanceInches = Math.sqrt(
                                targetXInches * targetXInches
                                        + targetYInches * targetYInches
                                        + targetZInches * targetZInches
                        );
                        TargetCamera = new Pose2d(
                                new Translation2d(
                                        TargetPoseCamera.getPosition().toUnit(DistanceUnit.METER).x,
                                        TargetPoseCamera.getPosition().toUnit(DistanceUnit.METER).y
                                ),
                                new Rotation2d(TargetPoseCamera.getOrientation().getYaw(AngleUnit.RADIANS))
                        );
                    }
                }
            }
        }
    }

    public double getTargetDistanceInches() {
        return targetDistanceInches;
    }

    @Override
    public void write() {}

    @Override
    public void reset() {
        tv = false;
        ApriltagID = NO_FIDUCIAL_ID_FILTER;
        targetFiducialId = NO_FIDUCIAL_ID_FILTER;
        targetDistanceInches = Double.NaN;
    }

    private LLResultTypes.FiducialResult getTargetFiducial(LLResult result) {
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            return null;
        }

        if (targetFiducialId == NO_FIDUCIAL_ID_FILTER) {
            return tags.get(0);
        }

        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == targetFiducialId) {
                return tag;
            }
        }

        return null;
    }
}
