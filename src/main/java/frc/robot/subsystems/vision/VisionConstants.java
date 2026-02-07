package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    public static final String frontRightCameraName = "cam0";
    public static final String frontLeftCameraName = "cam1";
    public static final String backCameraName = "cam2";

    public static final double backCameraLinearStdDevs = 0.05;
    public static final double frontLeftCameraLinearStdDevs = 0.05;
    public static final double frontRightCameraLinearStdDevs = 0.05;

    public static final double backCameraAngularStdDevs = Units.degreesToRadians(15);
    public static final double frontLeftCameraAngularStdDevs = Units.degreesToRadians(15);
    public static final double frontRightCameraAngularStdDevs = Units.degreesToRadians(15);

    public static final double DELAY_OFFSET = 0.0; // One loop cycle
}