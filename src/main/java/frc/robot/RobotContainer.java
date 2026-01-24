// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoChooser;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOCamera;
import frc.robot.util.vision.VisionUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final List<VisionCamera> cameras;

  private final AutoCommands autos;

  // Controller
  private final CommandXboxController primaryController = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);
  
  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoChooser choreoAutoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Sim robot
        drive =
        new Drive(
            new GyroIONavX(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));
    //   endEffector = new EndEffector(new EndEffectorIOSparkFlex());
    //   wrist = new Wrist(new WristIOSparkMax());
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
   
      //Boot up camera server
      CameraServer.startAutomaticCapture(0);
      break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
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
        
        break;

      default:
        // Replayed robot, disable IO implementations
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
        
        break;
    }
    
    autos = new AutoCommands(drive);
    
    choreoAutoChooser = new AutoChooser();
  
    choreoAutoChooser.addCmd("Wheel Radius Calibration", () -> DriveCommands.wheelRadiusCharacterization(drive));
    
    choreoAutoChooser.addCmd("StraightLine", autos::StraightLine);
    choreoAutoChooser.addCmd("StraightLineBack", autos::StraightLineBack);

    SmartDashboard.putData("Auto Chooser", choreoAutoChooser);
    // Configure the button bindings
    configureButtonBindings();
  }

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Cache April Tag Map
    VisionUtil.getApriltagPose(1);

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -primaryController.getLeftY(),
            () -> -primaryController.getLeftX(),
            () -> -primaryController.getRightX()));

    
    primaryController
      .start()
        .onTrue(Commands.runOnce(
          () ->
              drive.setPose(
                  new Pose2d(vision.getLastVisionPose().getTranslation(), new Rotation2d())),
          drive)
      .ignoringDisable(true));
    
    primaryController
      .back()
        .onTrue(Commands.runOnce(
          () ->
              drive.setPose(
                  new Pose2d(vision.getLastVisionPose().getTranslation(), drive.getRotation())),
          drive)
      .ignoringDisable(true));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return choreoAutoChooser.selectedCommand();
  }

  public void autonomousInit(){
    // elevator.init();
  }

  public void teleopInit() {
    // elevator.init();
    // climb.init().schedule();
    // AutoAlignUtil.createPOIListCommand().schedule();
  }
}