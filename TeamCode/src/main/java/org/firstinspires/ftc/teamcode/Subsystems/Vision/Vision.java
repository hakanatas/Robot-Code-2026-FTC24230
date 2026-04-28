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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Vision extends WSubsystem {
    public Limelight3A LL3;
    public double tx;
    public double ty;
    public boolean tv;
    public double tl;
    LLResult results;
    Pose2d TargetCamera = new Pose2d();
    int ApriltagID;
    LinearFilter filter;

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
        if(results!=null){
            this.tx=results.getTx();
            this.ty= results.getTy();
            this.tv =results.isValid();
            this.tl =results.getTargetingLatency()/100;
            if(this.tv){
                LLResultTypes.FiducialResult tag =results.getFiducialResults().get(0);
                if(tag.getFiducialId()==20){
                    Pose3D TargetPoseCamera =tag.getTargetPoseCameraSpace();
                    TargetCamera = new Pose2d(new Translation2d(TargetPoseCamera.getPosition().toUnit(DistanceUnit.METER).x, TargetPoseCamera.getPosition().toUnit(DistanceUnit.METER).y),new Rotation2d(TargetPoseCamera.getOrientation().getYaw(AngleUnit.RADIANS)));
                }
            }
        }
    }

    @Override
    public void write() {}

    @Override
    public void reset() {}
}
