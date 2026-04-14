package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.misc.AimUtil;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoAim extends SubsystemBase {
    private final InterpolatingMatrixTreeMap<Double, N3, N1> firingTable = new InterpolatingMatrixTreeMap<Double, N3, N1>();
    private final Hood hood;
    private final Flywheel flywheel;
    private final Turret turret;
    private final Indexer indexer;
    private final Drive drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    //private final ChassisSpeeds speeds;

    private boolean passing = false;

    public AutoAim(Turret turret, Hood hood, Flywheel flywheel, Indexer indexer, Drive drive, DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        // Distance (m)
        double flywheel_tuning = 0; //-125;
        firingTable.put(1.30, getMatrix(0.0, 2650 + flywheel_tuning, 1.05 - AutoAimConstants.PHASE_DELAY));
        //firingTable.put(3.23, getMatrix(4.39, 3215 + flywheel_tuning, 1.2)); // Home
        firingTable.put(3.23, getMatrix(4.39, 3130 + flywheel_tuning, 1.2 - AutoAimConstants.PHASE_DELAY)); // Chelsea //1.2
        firingTable.put(5.87, getMatrix(8.5, 3950 + flywheel_tuning, 1.4 - AutoAimConstants.PHASE_DELAY)); //1.4
        firingTable.put(7.0, getMatrix(10, 4050 + flywheel_tuning, 1.5 - AutoAimConstants.PHASE_DELAY));
        firingTable.put(12.0, getMatrix(10.5, 6300 + flywheel_tuning, 2.0 - AutoAimConstants.PHASE_DELAY));
    }

    public Command targetPassing() {
        return Commands.runOnce(
                () -> {
                    passing = true;
                });
    }

    public Command targetHub() {
        return Commands.runOnce(
                () -> {
                    passing = false;
                });
    }

    public Command shoot() {
        return Commands.sequence(
                Commands.runOnce(() -> setSetpoints(true),
                        flywheel, turret, hood, this),

                prepShot().alongWith(
                    // Commands.deadline(
                        Commands.waitUntil(this::isReady)
                        // Commands.run(() -> indexer.startReverseIndexing()))
                    .andThen(indexer.conditionalFeed(turret::atSetpoint)))

        ).finallyDo(interrupted -> stopShooting());
    }

    public boolean isReady() {
        return flywheel.atRPM() && turret.atSetpoint() && hood.atSetpoint();
    }

    public Command prepShot() {
        // Boolean should_prime_shot;
        // if (Constants.AutoAimConstants.PREPSHOT_OVERRIDE || trenchApproachTimeListTraversal() > (2 * AutoAimConstants.HOOD_LOWER_TIME)) {
        //     should_prime_shot = true;
        // } else {
        //     should_prime_shot = false;
        // }
        return Commands.run(
                () -> setSetpoints(true), //should_prime_shot
                turret,
                hood,
                flywheel,
                this);

    }

    public void startFeeding() {
        indexer.startIndexing();
    }

    public void stopShooting() {
        flywheel.setRPM(0);
        indexer.stopIndexing();
    }

    public Command trackTarget() {
        return Commands.run(
                () -> setSetpoints(false),
                turret,
                this);
    }

    public Translation2d getTargetTranslation() {
        if (!passing) {
            return AimUtil.getHubTranslation().plus(AimUtil.getFieldShiftFromJoystick(xSupplier, ySupplier));
        }
        return AimUtil.getPassTranslation(drive.getPose())
                .plus(AimUtil.getFieldShiftFromJoystick(xSupplier, ySupplier));
    }

    public double getDistanceToTargetFromRobotPose(Pose2d robotPose) {
        Translation2d turretTranslation = AimUtil.getTurretTranslationFromRobotPose(robotPose);
        return turretTranslation.getDistance(getTargetTranslation());
    }

    public void setSetpoints(boolean primingShot) {
        double phaseDelay = AutoAimConstants.PHASE_DELAY;

        Pose2d estimatedPose = drive.getPose();
        Twist2d fieldRelativeVelocity = drive.getFieldVelocity();

        double dtheta = drive.getChassisSpeeds().omegaRadiansPerSecond;

        estimatedPose = estimatedPose.exp(
                new Twist2d(
                        fieldRelativeVelocity.dx * phaseDelay,
                        fieldRelativeVelocity.dy * phaseDelay,
                        dtheta * phaseDelay));

        Translation2d target = getTargetTranslation();

        double timeOfFlight = 0;
        Pose2d lookaheadPose = drive.getPose();
        double lookaheadTurretToTargetDistance = getDistanceToTargetFromRobotPose(estimatedPose);

        for (int i = 0; i < 20; i++) {
            timeOfFlight = firingTable.get(lookaheadTurretToTargetDistance).get(2, 0);

            Translation2d newTranslation = 
                new Translation2d(
                        fieldRelativeVelocity.dx,
                        fieldRelativeVelocity.dy)
                    .times(timeOfFlight)
                    .plus(estimatedPose.getTranslation());

            Rotation2d newRotation = 
                new Rotation2d(
                        dtheta)
                    .times(timeOfFlight)
                    .plus(estimatedPose.getRotation());
            
            lookaheadPose = new Pose2d(newTranslation, newRotation);
            // ChassisSpeeds robotDelta = drive.getChassisSpeeds().times(timeOfFlight);
            // lookaheadPose = estimatedPose.plus(new Transform2d(robotDelta.vxMetersPerSecond,
            //         robotDelta.vyMetersPerSecond, new Rotation2d(robotDelta.omegaRadiansPerSecond)));
            lookaheadTurretToTargetDistance = getDistanceToTargetFromRobotPose(lookaheadPose);
        }

        Matrix<N3, N1> firingValues = getFiringTableValues(lookaheadTurretToTargetDistance);

        Pose2d robotPoseForTurret = new Pose2d(lookaheadPose.getX(), lookaheadPose.getY(), estimatedPose.getRotation());

        Pose2d turretPose = new Pose2d(AimUtil.getTurretTranslationFromRobotPose(robotPoseForTurret),
                robotPoseForTurret.getRotation());

        turret.setSetpointFromTurretPose(turretPose, target);
        if (primingShot) {
            hood.setSetpoint(firingValues.get(0, 0));
            flywheel.setRPM(firingValues.get(1, 0));
        } else {
            indexer.stopIndexing();
            //hood.setSetpoint(0.0);
        }

        Logger.recordOutput("AutoAim/DistanceToTarget", lookaheadTurretToTargetDistance);
        Logger.recordOutput("AutoAim/LookaheadPose", lookaheadPose);
        Logger.recordOutput("AutoAim/TurretPose", turretPose);
        Logger.recordOutput("AutoAim/TimeOfFlight", timeOfFlight);
    }

    public Matrix<N3, N1> getFiringTableValues(double distance) {
        return firingTable.get(distance);
    }

    public static Matrix<N3, N1> getMatrix(double hoodSetpoint, double flywheelSetpoint, double flightTimeSeconds) {
        return new Matrix<>(
                new SimpleMatrix(3, 1, true, new double[] { hoodSetpoint, flywheelSetpoint, flightTimeSeconds }));
    }

    // public ArrayList<Double> trenchApproachTimeList() {
    //     ArrayList<Double> timeArray = new ArrayList<>();
    //     //id for each trench tag
    //     int[] idArray = {12, 1, 7, 6};
        
    //     double xVelo = speeds.vxMetersPerSecond;
    //     double yVelo = speeds.vyMetersPerSecond;
    //     double resultant = Math.sqrt(Math.pow(xVelo, 2) + Math.pow(yVelo, 2));

    //     Pose2d drivePose = drive.getPose();
    //         Translation2d driveTranslation = drivePose.getTranslation();
    //         Double driveDouble = (Double) driveTranslation.getDistance(driveTranslation);

    //     for (int i = 0; i < idArray.length; i++) {
    //         Pose3d currentPose = VisionUtil.getApriltagPose(idArray[i]);
    //         Translation2d translation = currentPose.getTranslation().toTranslation2d();
    //         Double value = (Double) translation.getDistance(driveTranslation);
    //         Double time = Math.abs(driveDouble - value) / resultant;
    //         timeArray.add(time);
    //     }
    //     return timeArray;
    // }

    // public double trenchApproachTimeListTraversal() {
    //     ArrayList<Double> list = trenchApproachTimeList();
    //     double lowestTime = 2 * (AutoAimConstants.HOOD_LOWER_TIME) + 0.01;
    //     for (int i = 0; i < list.size(); i++){
    //         if (lowestTime > list.get(i)){
    //             lowestTime = list.get(i);
    //         }
    //     }
    //     return lowestTime;
    // }

    // //Override for the hood that sets the hood safety to be on or off
    // public void overridePrepshotTrue() {
    //     Constants.AutoAimConstants.PREPSHOT_OVERRIDE = true;
    // }

    // public void overridePrepshotFalse() {
    //     Constants.AutoAimConstants.PREPSHOT_OVERRIDE = false;
    // }
}