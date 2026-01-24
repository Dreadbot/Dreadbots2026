package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    private final CommandXboxController primaryController = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL: 

            // drive = 
            // new Drive(
            //     new GyroIONavX(),
            //     new ModuleIOSpark(0),
            //     new ModuleIOSpark(1),
            //     new ModuleIOSpark(2),
            //     new ModuleIOSpark(3));
            // turret = new Turret(new TurretIOSparkFlex());
            CameraServer.startAutomaticCapture(0);
            break;
            case SIM:

            // drive = 
            // new Drive(
            //     new GyroIO() {},
            //     new ModuleIOSim(),
            //     new ModuleIOSim(),
            //     new ModuleIOSim(),
            //     new ModuleIOSim());
            // turret = new Turret(new TurretIOSim());
            // break;

            default:

            // drive = 
            //     new Drive(
            //         new GyroIO() {},
            //         new ModuleIO() {}, 
            //         new ModuleIO() {}, 
            //         new ModuleIO() {}, 
            //         new ModuleIO() {});
            // turret = new Turret(new TurretIO() {});
            break;
        }
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        //VisionUtil.getAprilTagPose(1);
        // drive.setDefaulComand(
        //     DriveCommands.joystickDrive(
        //         drive, 
        //             () -> -primaryController.getLeftY(),
        //             () -> -primaryController.getLeftX(),
        //             () -> -primaryController.getRightX()));

        // primaryController.start().onTrue(
        //     Commands.runOnce(
        //         () -> drive.setPose(
        //                 new Pose2d(vision.getLastVisionPose().getTranslation(), new Rotation2d())),
        //                 drive).ignoringDisable(true));
        }

    public void autonomousInit() {

    }

    public void teleopInit() {

    }
}