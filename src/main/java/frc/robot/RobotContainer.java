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
import frc.robot.subsystems.slapdown.*;
import frc.robot.subsystems.climb.*;

public class RobotContainer {

    private final CommandXboxController primaryController = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);
    private final Drive drive;
    private final Vision vision;
    private final List<VisionCamera> cameras;
    private final Flywheel flywheel;
    private final Climb climb;
    private final Slapdown slapdown;

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


                     drive = 
                new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim());

                // turret = new Turret(new TurretIOSparkMax());
        //         cameras = List.of(
        //     new VisionCamera(
        //     new VisionIOCamera(VisionConstants.frontRightCameraName), 
        //     0),
        //     new VisionCamera(
        //     new VisionIOCamera(VisionConstants.frontLeftCameraName),
        //     1),
        //     new VisionCamera(
        //     new VisionIOCamera(VisionConstants.backCameraName),
        //     2));
        // vision = new Vision(
        //     cameras,
        //     drive::addVisionMeasurement,
        //     drive::getPose);
                // CameraServer.startAutomaticCapture(0);

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
                //flywheel = new Flywheel(new FlywheelIOSparkFlex());
                climb = new Climb(new ClimbIOSparkFlex());
                slapdown = new Slapdown(new SlapdownIOSparkMax());
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
                slapdown = new Slapdown(new SlapdownIOSim());

                climb = new Climb(new ClimbIOSim());
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

                climb = new Climb(new ClimbIO() {});
                flywheel = new Flywheel(new FlywheelIOSim());
                slapdown = new Slapdown(new SlapdownIOSparkMax());
                // turret = new Turret(new TurretIO() {});
                break;
            }
            configureButtonBindings();
        }









//This Configures the Button's bindings for the controller with the system for the robot
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



        // //Slapdown Algae Buttons (Left Trigger Intakes wheels/ Right Trigger Outakes wheels) (D-pad Up will pull in the intake system while D-pad down will push the intake system out to grab Algae) 
        primaryController.rightTrigger().whileTrue(climb.doClimbSequence());
        primaryController.leftTrigger().whileTrue(climb.unClimbSequence());

        primaryController.rightTrigger().whileTrue(slapdown.intakeSequence());
        primaryController.leftTrigger().whileTrue(slapdown.outtakeSequence());
         }
         public Command getAutonomousCommand() {
        return null; //choreoAutoChooser.selectedCommand();
    }

    public void autonomousInit() {

    }

    public void teleopInit() {

    }
}