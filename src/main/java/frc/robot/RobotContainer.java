package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkFlex;
import frc.robot.subsystems.LEDs.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOCamera;
import frc.robot.util.vision.VisionUtil;

public class RobotContainer {

    private final CommandXboxController primaryController = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);
    private final Drive drive;
    private final Vision vision;
    private final List<VisionCamera> cameras;
    private final Flywheel flywheel;
    private final Led leds;

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL: 

                drive = 
                new Drive(
                    new GyroIONavX(),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3));
                // turret = new Turret(new TurretIOSparkMax());
                cameras = List.of(
                    new VisionCamera(
                        new VisionIOCamera(VisionConstants.frontRightCameraName), 
                        0),
                    new VisionCamera(
                        new VisionIOCamera(VisionConstants.frontLeftCameraName),
                        1),
                    new VisionCamera(
                        new VisionIOCamera(VisionConstants.backCameraName),
                        2));
                vision = new Vision(
                    cameras,
                    drive::addVisionMeasurement,
                    drive::getPose);
                CameraServer.startAutomaticCapture(0);

                flywheel = new Flywheel(new FlywheelIOSparkFlex());
                leds = new Led(new LedIOAddressableLED());
            break;
            
            case SIM:

                drive = 
                new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim());
                cameras = List.of(
                new VisionCamera(
                    new VisionIOCamera(VisionConstants.frontRightCameraName), 
                    0),
                new VisionCamera(
                    new VisionIOCamera(VisionConstants.frontLeftCameraName),
                    1),
                new VisionCamera(
                    new VisionIOCamera(VisionConstants.backCameraName),
                    2));
                vision = new Vision(
                    cameras,
                    drive::addVisionMeasurement,
                    drive::getPose);

                flywheel = new Flywheel(new FlywheelIOSim());
                // turret = new Turret(new TurretIOSim());
                leds = new Led(new LedIO() {});
                break;

            default:

                drive = 
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {}, 
                        new ModuleIO() {}, 
                        new ModuleIO() {}, 
                        new ModuleIO() {});

                cameras = List.of(
                    new VisionCamera(
                        new VisionIO() {},
                        0),
                    new VisionCamera(
                        new VisionIO() {},
                        1),
                    new VisionCamera(
                        new VisionIO() {},
                        2));
                vision = new Vision(
                    cameras,
                    drive::addVisionMeasurement,
                    drive::getPose);

                flywheel = new Flywheel(new FlywheelIOSim());
                // turret = new Turret(new TurretIO() {});
                leds = new Led(new LedIO() {});
                break;
            }
            configureButtonBindings();
        }

    private void configureButtonBindings() {
        
        VisionUtil.getApriltagPose(1);
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive, 
                    () -> -primaryController.getLeftY(),
                    () -> -primaryController.getLeftX(),
                    () -> -primaryController.getRightX()));

        primaryController.start().onTrue(
            Commands.runOnce(
                () -> drive.setPose(
                        new Pose2d(vision.getLastVisionPose().getTranslation(), new Rotation2d())),
                        drive).ignoringDisable(true));
        disabled().onTrue(
            new InstantCommand(() -> {
                leds.setToPattern(LEDPattern.solid(leds.getAllianceColor())
                .breathe(Seconds.of(1)));})
                .ignoringDisable(true));
    }
    public Command getAutonomousCommand() {
        return null; //choreoAutoChooser.selectedCommand();
    }

    public void autonomousInit() {

    }

    public void teleopInit() {

    }

    private static Trigger disabled() {
        return new Trigger(DriverStation::isDisabled);
    }
}