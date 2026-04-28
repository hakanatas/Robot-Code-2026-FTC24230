package edu.wpi.first.math.estimator;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.kinematics.GoBildaPinpointDriver;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PinPointPoseEstimator extends PoseEstimator {
    public PinPointPoseEstimator(
            GoBildaPinpointDriver pinpointDriver,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super(
                pinpointDriver,
                stateStdDevs,
                visionMeasurementStdDevs);
    }
}
