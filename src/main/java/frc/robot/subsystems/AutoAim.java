package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.TurretConstants;
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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    public AutoAim(Turret turret, Hood hood, Flywheel flywheel, Indexer indexer, Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        // Distance (m)
        firingTable.put(1.20, getMatrix(0.0, 2900, 1.24));
        firingTable.put(2.16, getMatrix(2.92, 2960, 1.24));
        firingTable.put(2.64, getMatrix(6.3, 3173, 1.3));
        firingTable.put(2.98, getMatrix(5.4, 3161, 1.35));
        firingTable.put(3.80, getMatrix(5.83, 3450, 1.31));
        firingTable.put(4.43, getMatrix(6.0, 3600, 1.40));
        firingTable.put(5.08, getMatrix(9.67, 3867, 1.40));
        firingTable.put(5.67, getMatrix(10.0, 3921, 1.45));
    }

    public Command shoot() {
        return prepShot()
                .alongWith(Commands.waitUntil(turret::atSetpoint)
                .andThen(Commands.runOnce(
                        () -> startFeeding(), 
                        indexer))
                .andThen(Commands.runEnd(
                        () -> setSetpoints(true),
                        () -> stopShooting()
                        )));
    }

    public Command prepShot() {
        return Commands.run(
                () -> setSetpoints(true),
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
        return AimUtil.getHubTranslation().plus(AimUtil.getFieldShiftFromJoystick(xSupplier, ySupplier));
    }

    public double getDistanceToTargetFromRobotPose(Pose2d robotPose) {
        Translation2d turretTranslation = AimUtil.getTurretTranslationFromRobotPose(robotPose);
        return turretTranslation.getDistance(getTargetTranslation());
    }

    public void setSetpoints(boolean primingShot) {
        double phaseDelay = AutoAimConstants.PHASE_DELAY;

        Pose2d estimatedPose = drive.getPose();
        ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

        estimatedPose = estimatedPose.exp(
                new Twist2d(
                        robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                        robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                        robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        Translation2d target = getTargetTranslation();

        double timeOfFlight = 0;
        Pose2d lookaheadPose = drive.getPose();
        double lookaheadTurretToTargetDistance = getDistanceToTargetFromRobotPose(estimatedPose);

        for (int i = 0; i < 20; i++) {
            timeOfFlight = firingTable.get(lookaheadTurretToTargetDistance).get(2, 0);
            ChassisSpeeds robotDelta = robotRelativeVelocity.times(timeOfFlight);
            lookaheadPose = estimatedPose.plus(new Transform2d(robotDelta.vxMetersPerSecond,
                    robotDelta.vyMetersPerSecond, new Rotation2d(robotDelta.omegaRadiansPerSecond)));
            lookaheadTurretToTargetDistance = getDistanceToTargetFromRobotPose(lookaheadPose);
        }

        Matrix<N3, N1> firingValues = getFiringTableValues(lookaheadTurretToTargetDistance);

        Pose2d robotPoseForTurret = new Pose2d(lookaheadPose.getX(), lookaheadPose.getY(), estimatedPose.getRotation());

        Pose2d turretPose = new Pose2d(AimUtil.getTurretTranslationFromRobotPose(robotPoseForTurret),
                robotPoseForTurret.getRotation());

        turret.setSetpointFromTurretPose(turretPose, target);
        if (primingShot) {
            hood.setRotations(firingValues.get(0, 0));
            flywheel.setRPM(firingValues.get(1, 0));
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
}