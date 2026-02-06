package frc.robot.subsystems.vision;

public final class VisionConstants {
    public static final String frontLeftCameraTable = "Cam0";
    public static final String frontRightCameraTable = "Cam1";
    public static final String backLeftCameraTable = "Cam2";
    public static final String backRightCameraTable = "Cam3";

    public static final double frontLeftCameraLinearStdDevs = 0.05;
    public static final double frontRightCameraLinearStdDevs = 0.05;
    public static final double backLeftCameraLinearStdDevs = 0.05;
    public static final double backRightCameraLinearStdDevs = 0.05;

    public static final double frontLeftCameraAngularStdDevs = 100_000;
    public static final double frontRightCameraAngularStdDevs = 100_000;
    public static final double backLeftCameraAngularStdDevs = 100_000;
    public static final double backRightCameraAngularStdDevs = 100_000;

    public static final double DELAY_OFFSET = 0.10; // One loop cycle
}