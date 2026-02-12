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
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSparkMax;
import frc.robot.util.vision.VisionUtil;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOCamera;
import frc.robot.subsystems.vision.VisionIOSim;

import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkFlex;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkFlex;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOSparkMax;

public class RobotContainer {

    private final CommandXboxController primaryController = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);
    private final Drive drive;
    private final Vision vision;
    private final List<VisionCamera> cameras;
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Indexer indexer;
    
    private final AutoAim autoAim;

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
          turret = new Turret(
            new TurretIOSparkMax(),
            drive);
          hood = new Hood(new HoodIOSparkMax());
          flywheel = new Flywheel(new FlywheelIOSparkFlex());
          indexer = new Indexer(new IndexerIOSparkFlex());

          autoAim = new AutoAim(turret, hood, flywheel, indexer, drive);
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
              new VisionIOSim(drive::getPose), 
              0),
            new VisionCamera(
              new VisionIOSim(drive::getPose), 
              1),
            new VisionCamera(
              new VisionIOSim(drive::getPose), 
              2));
          vision = new Vision(
                cameras,
                drive::addVisionMeasurement,
                drive::getPose);
          turret = new Turret(
            new TurretIOSim(),
            drive);
          hood = new Hood(new HoodIOSim());
          flywheel = new Flywheel(new FlywheelIOSim());
          indexer = new Indexer(new IndexerIOSim());

          autoAim = new AutoAim(turret, hood, flywheel, indexer, drive);
          break;
        default:
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
          turret = new Turret(
            new TurretIOSparkMax(), 
            drive);
          hood = new Hood(new HoodIOSim());
          flywheel = new Flywheel(new FlywheelIOSim());
          indexer = new Indexer(new IndexerIOSim());

          autoAim = new AutoAim(turret, hood, flywheel, indexer, drive);
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
          // primaryController.axisGreaterThan(0, 0).onTrue(
          //   turret.setAngleRad(0.5 * Math.PI));
          // primaryController.axisLessThan(0, 0).onTrue(
          //   turret.setAngleRad(-0.5 * Math.PI));
          // primaryController.axisGreaterThan(1, 0).onTrue(
          //   turret.setAngleRad(0 * Math.PI)); 
          primaryController.x().whileTrue(
            autoAim.shoot()
          );
         }

    public Command getAutonomousCommand() {
        return null; // choreoAutoChooser.selectedCommand();
    }

    public void autonomousInit() {

    }

    public void teleopInit() {

    }
}