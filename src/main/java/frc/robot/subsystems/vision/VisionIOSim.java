package frc.robot.subsystems.vision;

import java.util.Random;

import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionCamera.PoseSupplier;

public class VisionIOSim implements VisionIO{
    private PoseSupplier supplier;
    private final TimeInterpolatableBuffer<Pose2d> m_odometryPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(1.5);

    public VisionIOSim(PoseSupplier supplier) {
        this.supplier = supplier;
    }

    /**
     * Get a "noisy" fake global pose reading.
     *
     * @param estimatedRobotPose The robot pose.
     */
    public static Pose2d getEstimatedGlobalPose(Pose2d estimatedRobotPose) {
        var rand =
            StateSpaceUtil.makeWhiteNoiseVector(VecBuilder.fill(0.001, 0.001, Units.degreesToRadians(5)));
        return new Pose2d(
            estimatedRobotPose.getX() + rand.get(0, 0),
            estimatedRobotPose.getY() + rand.get(1, 0),
            estimatedRobotPose.getRotation().plus(new Rotation2d(rand.get(2, 0))));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        m_odometryPoseBuffer.addSample(Timer.getFPGATimestamp(), supplier.getPose());

        VisionDetection detection = new VisionDetection(null, 0, 0);

        var odometry_pose_sample = m_odometryPoseBuffer.getSample(Timer.getFPGATimestamp() - 0.1);
        if (odometry_pose_sample.isPresent()) {
            detection = new VisionDetection(getEstimatedGlobalPose(odometry_pose_sample.get()), new Random().nextInt(1, 33), Timer.getFPGATimestamp() * 1_000_000);
        }

        inputs.detections = new VisionDetection[] {detection};
        inputs.visionDelay = 0.1;
    }
}

