package frc.robot;

import java.util.List;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
import choreo.auto.AutoChooser.*;

public class RobotContainer {

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final Drive drive;
    private final Vision vision;
    private final List<VisionCamera> cameras;
    private final Turret turret;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Indexer indexer;
    private final AutoAim autoAim;
    private final Climb climb;
    private final Slapdown slapdown;
    private final AutoChooser choreoAutoChooser;
    private final AutoCommands autos;


    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                        new GyroIONavX(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));
                cameras = List.of(
                        new VisionCamera(new VisionIOCamera(VisionConstants.frontRightCameraName), 0),
                        new VisionCamera(new VisionIOCamera(VisionConstants.frontLeftCameraName), 1),
                        new VisionCamera(new VisionIOCamera(VisionConstants.backCameraName), 2));
                vision = new Vision(cameras, drive::addVisionMeasurement, drive::getPose);
                turret = new Turret(new TurretIOSparkMax(), drive);

                flywheel = new Flywheel(new FlywheelIOSparkFlex());
                hood = new Hood(new HoodIOSparkMax());
                indexer = new Indexer(new IndexerIOSparkFlex());
                slapdown = new Slapdown(new SlapdownIOSparkMax());
                climb = new Climb(new ClimbIOSparkFlex(), slapdown);
                break;
            case SIM:
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                cameras = List.of(
                        new VisionCamera(new VisionIOSim(drive::getPose), 0),
                        new VisionCamera(new VisionIOSim(drive::getPose), 1),
                        new VisionCamera(new VisionIOSim(drive::getPose), 2));

                vision = new Vision(cameras, drive::addVisionMeasurement, drive::getPose);
                turret = new Turret(new TurretIOSim(), drive);
                flywheel = new Flywheel(new FlywheelIOSim());
                hood = new Hood(new HoodIOSim());
                indexer = new Indexer(new IndexerIOSim());
                slapdown = new Slapdown(new SlapdownIOSim());
                climb = new Climb(new ClimbIOSim(), slapdown);
                

                break;
            default:
                drive = new Drive(
                        new GyroIONavX(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));
                cameras = List.of(
                        new VisionCamera(new VisionIOCamera(VisionConstants.frontRightCameraName), 0),
                        new VisionCamera(new VisionIOCamera(VisionConstants.frontLeftCameraName), 1),
                        new VisionCamera(new VisionIOCamera(VisionConstants.backCameraName), 2));
                vision = new Vision(cameras, drive::addVisionMeasurement, drive::getPose);
                turret = new Turret(new TurretIOSparkMax(), drive);
                flywheel = new Flywheel(new FlywheelIOSparkFlex());
                hood = new Hood(new HoodIOSparkMax());
                indexer = new Indexer(new IndexerIOSparkFlex());
                slapdown = new Slapdown(new SlapdownIOSparkMax());
                climb = new Climb(new ClimbIOSparkFlex(), slapdown);
             

                break;
        }
        autoAim = new AutoAim(turret, hood, flywheel, indexer, drive, operator::getLeftX, operator::getLeftY);

        autos = new AutoCommands(drive, slapdown, indexer, climb, flywheel, autoAim);

        // Set up auto routines
        choreoAutoChooser = new AutoChooser();
        
        choreoAutoChooser.addRoutine("Left Double", autos::leftDouble);
        // 1
        //choreoAutoChooser.addRoutine("Center Center Climb", autos::centerCenterClimb);
        // // 2
        // choreoAutoChooser.addRoutine("Left Center Sweep", autos::leftCenter);
        // 3
        //choreoAutoChooser.addCmd("Outpost Climb", autos::outpostClimb);
        // 4
        //choreoAutoChooser.addCmd("Center Outpost Climb", autos::centerOutpostClimb);
        // 5
        //choreoAutoChooser.addCmd("Center Fire", autos::centerFire);
        // 6 needs to be done
        //choreoAutoChooser.addCmd("", autos::);
        SmartDashboard.putData("Auto Chooser", choreoAutoChooser);

        configureButtonBindings();
    }
 
    // This configures the button's bindings for the controller with the system for the robot
    private void configureButtonBindings() {
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX()));

        driver.start().onTrue(
                Commands.runOnce(
                        () -> drive.setPose(
                                new Pose2d(
                                        vision.getLastVisionPose().getTranslation(),
                                        new Rotation2d())),
                        drive).ignoringDisable(true));
        
        // autoAim.setDefaultCommand(autoAim.trackTarget());
        
        driver.b().onTrue(turret.setAngleRad(0 * Math.PI));
        driver.a().whileTrue(drive.brace());

        driver.leftTrigger().whileTrue(slapdown.intakeCommand());
        driver.leftBumper().whileTrue(autoAim.trackTarget());
        // driver.leftBumper().onTrue(climb.climb());
        
        driver.rightTrigger().whileTrue(autoAim.shoot());
        driver.rightBumper().whileTrue(autoAim.prepShot());

        hood.setDefaultCommand(hood.run(() -> hood.setSetpoint(0)));

        // Final Operator Controls
        operator.rightBumper().onTrue(slapdown.goToIntakeCommand());
        operator.leftBumper().onTrue(slapdown.goToHomeCommand());
        operator.leftTrigger().whileTrue(hood.run(() -> hood.setSetpoint(0)));
        operator.rightTrigger().whileTrue(slapdown.agitateCommand());

        operator.axisGreaterThan(4, IndexerConstants.DEAD_BAND)
                .onTrue(Commands.runOnce(() -> indexer.startIndexing()));
        operator.axisLessThan(4, -IndexerConstants.DEAD_BAND)
                .onTrue(Commands.runOnce(() -> indexer.startReverseIndexing()));
        operator.axisMagnitudeGreaterThan(4, IndexerConstants.DEAD_BAND)
                .onFalse(Commands.runOnce(() -> indexer.stopIndexing()));
        operator.start().onTrue(Commands.runOnce(() -> hood.calibrate()));

        operator.a().onTrue(autoAim.targetPassing());
        operator.b().onTrue(autoAim.targetHub());
        // operator.y().onTrue(climb.prep);
        // operator.x().onTrue(climb.stow);

        // Turret Presets
        // secondaryController.povDown().onTrue(turret.setAngleRad(Constants.TurretConstants.TURRET_PRESET_ANGLE1));
        // secondaryController.povRight().onTrue(turret.setAngleRad(Constants.TurretConstants.TURRET_PRESET_ANGLE2));
        // secondaryController.povUp().onTrue(turret.setAngleRad(Constants.TurretConstants.TURRET_PRESET_ANGLE3));

        // Tuning Operator Controls
        // operator.povUp().onTrue(hood.changeRotations(0.5));
        // operator.povDown().onTrue(hood.changeRotations(-0.5));
        // operator.y().onTrue(Commands.runOnce(() -> flywheel.setRPM(3000)));
        // operator.a().onTrue(Commands.runOnce(() -> flywheel.setRPM(0)));
        // operator.b().onTrue(flywheel.changeRPM(100));
        // operator.x().onTrue(flywheel.changeRPM(-100));
    }

    public Command getAutonomousCommand() {
        return choreoAutoChooser.selectedCommand();
        //return RobotModeTriggers.autonomous.whileTrue(choreoAutoChooser.selectedCommand());
    }

    public void autonomousInit() {

    }

    public void teleopInit() {

    }
}