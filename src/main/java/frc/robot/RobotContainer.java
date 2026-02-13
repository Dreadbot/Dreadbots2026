package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.DriveCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AutoAim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSparkMax;
import frc.robot.util.vision.VisionUtil;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOCamera;
import frc.robot.subsystems.vision.VisionIOSim;

import frc.robot.subsystems.slapdown.Slapdown;
import frc.robot.subsystems.slapdown.SlapdownIOSim;
import frc.robot.subsystems.slapdown.SlapdownIOSparkMax;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.flywheel.*;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.indexer.*;

public class RobotContainer {

    private final CommandXboxController primaryController = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);
    private final Drive drive;
    private final Vision vision;
    private final List<VisionCamera> cameras;
    private final Turret turret;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Indexer indexer;

    private final AutoAim autoAim;

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                    new GyroIONavX(),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3)
                );
                cameras = List.of(
                    new VisionCamera(new VisionIOCamera(VisionConstants.frontRightCameraName), 0),
                    new VisionCamera(new VisionIOCamera(VisionConstants.frontLeftCameraName), 1),
                    new VisionCamera(new VisionIOCamera(VisionConstants.backCameraName), 2)
                );
                vision = new Vision(cameras, drive::addVisionMeasurement, drive::getPose);
                turret = new Turret(new TurretIOSparkMax(), drive);
                CameraServer.startAutomaticCapture(0);
                flywheel = new Flywheel(new FlywheelIOSparkFlex());
                hood = new Hood(new HoodIOSparkMax());
                indexer = new Indexer(new IndexerIOSparkFlex());
                break;
            case SIM:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );
                cameras = List.of(
                    new VisionCamera(new VisionIOSim(drive::getPose), 0),
                    new VisionCamera(new VisionIOSim(drive::getPose), 1),
                    new VisionCamera(new VisionIOSim(drive::getPose), 2)
                );
                vision = new Vision(cameras, drive::addVisionMeasurement, drive::getPose);
                turret = new Turret(new TurretIOSim(), drive);
                flywheel = new Flywheel(new FlywheelIOSim());
                hood = new Hood(new HoodIOSim());
                indexer = new Indexer(new IndexerIOSim());
                break;
            default:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );
                cameras = List.of(
                    new VisionCamera(new VisionIOCamera(VisionConstants.frontRightCameraName), 0),
                    new VisionCamera(new VisionIOCamera(VisionConstants.frontLeftCameraName), 1),
                    new VisionCamera(new VisionIOCamera(VisionConstants.backCameraName), 2)
                );
                vision = new Vision(cameras, drive::addVisionMeasurement, drive::getPose);
                turret = new Turret(new TurretIOSparkMax(), drive);
                flywheel = new Flywheel(new FlywheelIOSim());
                hood = new Hood(new HoodIOSim());
                indexer = new Indexer(new IndexerIOSim());
                break;
        }
        autoAim = new AutoAim(turret, hood, flywheel, indexer, drive);
        configureButtonBindings();
    }

    // This configures the button's bindings for the controller with the system for the robot
    private void configureButtonBindings() {
        VisionUtil.getApriltagPose(1);

        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -primaryController.getLeftY(),
                () -> -primaryController.getLeftX(),
                () -> -primaryController.getRightX()
            )
        );

        primaryController.start().onTrue(
            Commands.runOnce(
                () -> drive.setPose(
                    new Pose2d(
                        vision.getLastVisionPose().getTranslation(),
                        new Rotation2d()
                    )
                ),
                drive
            ).ignoringDisable(true)
        );

        primaryController.x().onTrue(turret.setAngleRad(0.5 * Math.PI));
        primaryController.y().onTrue(turret.setAngleRad(0 * Math.PI));
        primaryController.b().onTrue(turret.setAngleRad(-0.5 * Math.PI));
        primaryController.a().onTrue(turret.setAngleRad(1 * Math.PI));

        primaryController.leftBumper().whileTrue(autoAim.trackTarget());

        // Subsystem button bindings used for testing
        // Can be changed or refit for actual use
        // These were all used seperately so buttons may be used more than once

        // primaryController.a().onTrue(flywheel.setRPM(3000));
        // primaryController.b().onTrue(flywheel.setRPM(0));
        // primaryController.y().onTrue(flywheel.changeRPM(100));
        // primaryController.x().onTrue(flywheel.changeRPM(-100));

        // primaryController.leftTrigger().whileTrue(indexer.intake());
        // primaryController.rightTrigger().whileTrue(indexer.outtake());
        
        // primaryController.rightBumper().onTrue(hood.setAngle(2));
        // primaryController.leftBumper().onTrue(hood.setAngle(0));
        // primaryController.a().onTrue(hood.calibrate());
    }

    public Command getAutonomousCommand() {
        return null; // choreoAutoChooser.selectedCommand();
    }

    public void autonomousInit() {

    }

    public void teleopInit() {
        
    }
}