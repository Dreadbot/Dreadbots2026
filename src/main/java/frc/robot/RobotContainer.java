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
import frc.robot.util.vision.VisionUtil;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOCamera;
import frc.robot.subsystems.vision.VisionIOSim;

import frc.robot.subsystems.flywheel.*;
import frc.robot.subsystems.turret.*;
import frc.robot.subsystems.slapdown.*;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.vision.*;

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
    // private final AutoAim autoAim;
    private final Climb climb;
    private final Slapdown slapdown;

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
                slapdown = new Slapdown(new SlapdownIOSparkMax());
                climb = new Climb(new ClimbIOSparkFlex());
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
                slapdown = new Slapdown(new SlapdownIOSim());
                climb = new Climb(new ClimbIOSim());
                break;
            default:
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
                flywheel = new Flywheel(new FlywheelIOSparkFlex());
                hood = new Hood(new HoodIOSparkMax());
                indexer = new Indexer(new IndexerIOSparkFlex());
                slapdown = new Slapdown(new SlapdownIOSparkMax());
                climb = new Climb(new ClimbIOSparkFlex());
                break;
        }
    }

    // This configures the button's bindings for the controller with the system for the robot
    private void configureButtonBindings() {
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

        // autoAim.setDefaultCommand(
        //     autoAim.trackTarget()
        // );

        // Subsystem button bindings used for testing
        // Can be changed or refit for actual use
        // These were all used seperately so buttons may be used more than once



        // Turret Angle Controls
        primaryController.x().onTrue(turret.setAngleRad(0.5 * Math.PI));
        primaryController.y().onTrue(turret.setAngleRad(0 * Math.PI));
        primaryController.b().onTrue(turret.setAngleRad(-0.5 * Math.PI));
        //primaryController.a().onTrue(turret.setAngleRad(1 * Math.PI));

        //primaryController.leftBumper().whileTrue(autoAim.trackTarget());

        // Climb controls
        //primaryController.rightTrigger().whileTrue(climb.doClimbSequence());
        //primaryController.leftTrigger().whileTrue(climb.unClimbSequence());

       
        // Flywheel controls
        secondaryController.a().onTrue(flywheel.setRPM(3000));
        // secondaryController.a().onTrue(flywheel.start()); // Start method used for testing, 1 press for 500 rpm, another press for 3000 rpm
        
        // secondaryController.b().onTrue(flywheel.setRPM(0));
        secondaryController.b().onTrue(flywheel.stop()); // Doesn't stop immediately, just sends 0 voltage
        
        secondaryController.y().onTrue(flywheel.changeRPM(100));
        secondaryController.x().onTrue(flywheel.changeRPM(-100));

        // PID and FF tuning controls, wouldn't recommend using since it's been tuned already
        // primaryController.y().onTrue(flywheel.changeRPM(100));
        // primaryController.x().onTrue(flywheel.changeRPM(-100));
        // primaryController.leftTrigger().onTrue(flywheel.changeOrderOfMagnitude(1));
        // primaryController.rightTrigger().onTrue(flywheel.changeOrderOfMagnitude(-1));
        // primaryController.leftBumper().onTrue(flywheel.changeNumber(-1));
        // primaryController.rightBumper().onTrue(flywheel.changeNumber(1));
        // primaryController.povLeft().onTrue(flywheel.changeTarget("left"));
        // primaryController.povUp().onTrue(flywheel.changeTarget("up"));
        // primaryController.povRight().onTrue(flywheel.changeTarget("right"));
        // primaryController.povDown().onTrue(flywheel.changeSystem());

            // Indexer and Kicker motors controlled by the second controller
            //secondaryController.getLeftX().onTrue(indexer.startIndexer());
            //secondaryController.getRightX().onTrue(indexer.stopIndexer());
        
        // primaryController.rightBumper().onTrue(hood.setAngle(4));
        // primaryController.leftBumper().onTrue(hood.setAngle(0));
        // primaryController.povLeft().onTrue(hood.calibrate());
        // primaryController.povUp().onTrue(hood.changeAngle(0.1));
        // primaryController.povDown().onTrue(hood.changeAngle(-0.1));
        //Slapdown
        secondaryController.rightBumper().onTrue(slapdown.goToIntakeCommand());
        secondaryController.leftBumper().onTrue(slapdown.goToHomeCommand());
        primaryController.leftTrigger().whileTrue(slapdown.intakeCommand());
        primaryController.leftTrigger().onFalse(slapdown.stopIntakeCommand());

        // Double check with test
        secondaryController.rightTrigger().whileTrue(slapdown.agitateCommand());
        secondaryController.rightTrigger().onFalse(slapdown.stopIntakeCommand());

        // Hood
        secondaryController.leftTrigger().whileTrue(hood.changeRotations(-0.1));
        //secondaryController.leftTrigger().whileFalse(hood.changeRotations(1));
    
        // Turret Presets
        secondaryController.povRight().onTrue(turret.setAtZero().andThen(turret.setAngleRad(0)));
        secondaryController.povDown().onTrue(turret.setAngleRad(Constants.TurretConstants.TURRET_PRESET_ANGLE1));
        secondaryController.povLeft().onTrue(turret.setAngleRad(Constants.TurretConstants.TURRET_PRESET_ANGLE2));
        secondaryController.povUp().onTrue(turret.setAngleRad(Constants.TurretConstants.TURRET_PRESET_ANGLE3));
        // Pass Track
        secondaryController.b().onTrue(turret.setAngleRad(Constants.TurretConstants.PASS_ANGLE));
        // Hub Track
        secondaryController.a().onTrue(turret.setAngleRad(Constants.TurretConstants.HUB_ANGLE));
    }

    public Command getAutonomousCommand() {
        return null; // choreoAutoChooser.selectedCommand();

    }

    public void autonomousInit() {

    }

    public void teleopInit() {
        
    }
}