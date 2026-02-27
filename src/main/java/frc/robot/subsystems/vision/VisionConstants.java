package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    public static final String frontRightCameraName = "Cam0";
    public static final String frontLeftCameraName = "Cam2";
    public static final String backCameraName = "Cam4";

    public static final double backCameraLinearStdDevs = 0.03;
    public static final double frontLeftCameraLinearStdDevs = 0.03;
    public static final double frontRightCameraLinearStdDevs = 0.03;

    public static final double backCameraAngularStdDevs = Units.degreesToRadians(100_000);
    public static final double frontLeftCameraAngularStdDevs = Units.degreesToRadians(100_000);
    public static final double frontRightCameraAngularStdDevs = Units.degreesToRadians(100_000);

    public static final double DELAY_OFFSET = 0.10; // One loop cycle
}