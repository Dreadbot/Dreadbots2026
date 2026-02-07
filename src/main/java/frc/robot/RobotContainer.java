package frc.robot;

import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.DriveCommands;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.util.vision.VisionUtil;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOCamera;

import frc.robot.subsystems.flywheel.*;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkFlex;

public class RobotContainer {

    private final CommandXboxController primaryController = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);
    //private final Drive drive;
    //private final Vision vision;
    //private final List<VisionCamera> cameras;
    private final Flywheel flywheel;
    private Indexer indexer;

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:

                // drive = new Drive(
                //     new GyroIONavX(),
                //     new ModuleIOSpark(0),
                //     new ModuleIOSpark(1),
                //     new ModuleIOSpark(2),
                //     new ModuleIOSpark(3));
                // drive = new Drive(
                //     new GyroIO() {
                //     },
                //     new ModuleIOSim(),
                //     new ModuleIOSim(),
                //     new ModuleIOSim(),
                //     new ModuleIOSim());
                // cameras = List.of(
                //     new VisionCamera(
                //         new VisionIOCamera(VisionConstants.frontRightCameraName),
                //         0),
                //     new VisionCamera(
                //         new VisionIOCamera(VisionConstants.frontLeftCameraName),
                //         1),
                //     new VisionCamera(
                //         new VisionIOCamera(VisionConstants.backCameraName),
                //         2));
                // vision = new Vision(
                //     cameras,
                //     drive::addVisionMeasurement,
                //     drive::getPose);

                flywheel = new Flywheel(new FlywheelIOSim());
                // turret = new Turret(new TurretIOSim());

                indexer = new Indexer(new IndexerIOSparkFlex());
                // // turret = new Turret(new TurretIOSparkMax());
                // cameras = List.of(
                //     new VisionCamera(
                //         new VisionIOCamera(VisionConstants.frontRightCameraName),
                //         0),
                //     new VisionCamera(
                //         new VisionIOCamera(VisionConstants.frontLeftCameraName),
                //         1),
                //     new VisionCamera(
                //         new VisionIOCamera(VisionConstants.backCameraName),
                //         2));
                // vision = new Vision(
                //     cameras,
                //     drive::addVisionMeasurement,
                //     drive::getPose);
                // CameraServer.startAutomaticCapture(0);

                flywheel = new Flywheel(new FlywheelIOSparkFlex());
                break;










            case SIM:
                // drive = new Drive(
                //     new GyroIO() {
                //     },
                //     new ModuleIOSim(),
                //     new ModuleIOSim(),
                //     new ModuleIOSim(),
                //     new ModuleIOSim());
                // indexer = new Indexer(new IndexerIOSim());
                // cameras = List.of(
                //     new VisionCamera(
                //         new VisionIOCamera(VisionConstants.frontRightCameraName),
                //         0),
                //     new VisionCamera(
                //         new VisionIOCamera(VisionConstants.frontLeftCameraName),
                //         1),
                //     new VisionCamera(
                //         new VisionIOCamera(VisionConstants.backCameraName),
                //         2));
                // vision = new Vision(
                //     cameras,
                //     drive::addVisionMeasurement,
                //     drive::getPose);

                flywheel = new Flywheel(new FlywheelIOSim());
                // turret = new Turret(new TurretIOSim());
                break;

            default:

                // drive = new Drive(
                //     new GyroIO() {
                //     },
                //     new ModuleIO() {
                //     },
                //     new ModuleIO() {
                //     },
                //     new ModuleIO() {
                //     },
                //     new ModuleIO() {
                //     });
                // indexer = new Indexer(new IndexerIO() {
                // });
                // cameras = List.of(
                //     new VisionCamera(
                //         new VisionIO() {
                //         },
                //         0),
                //     new VisionCamera(
                //         new VisionIO() {
                //         },
                //         1),
                //     new VisionCamera(
                //         new VisionIO() {
                //         },
                //         2));
                // vision = new Vision(
                //     cameras,
                //     drive::addVisionMeasurement,
                //     drive::getPose);

                flywheel = new Flywheel(new FlywheelIO());
                // turret = new Turret(new TurretIO() {});
                break;
        }
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // VisionUtil.getApriltagPose(1);
        // drive.setDefaultCommand(
        //     DriveCommands.joystickDrive(
        //         drive,
        //         () -> -primaryController.getLeftY(),
        //         () -> -primaryController.getLeftX(),
        //         () -> -primaryController.getRightX()));

        primaryController.start().onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(vision.getLastVisionPose().getTranslation(), new Rotation2d())),
                drive).ignoringDisable(true));
        primaryController.leftTrigger().whileTrue(indexer.intake());
        primaryController.rightTrigger().whileTrue(indexer.outtake());
        primaryController.a().onTrue(flywheel.setRPM(400));
        primaryController.b().onTrue(flywheel.setRPM(0));
        primaryController.y().onTrue(flywheel.changeRPM(100));
        primaryController.x().onTrue(flywheel.changeRPM(-100));
    }

    public Command getAutonomousCommand() {
        return null; // choreoAutoChooser.selectedCommand();
    }

    public void autonomousInit() {

    }

    public void teleopInit() {

    }
}