package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoAim {
    private final InterpolatingMatrixTreeMap<Double, N3, N1> firingTable = new InterpolatingMatrixTreeMap<Double, N3, N1>();
    private final Hood hood;
    private final Flywheel flywheel;
    private final Turret turret;
    private final Indexer indexer;
    private final Drive drive;

    public AutoAim(Turret turret, Hood hood, Flywheel flywheel, Indexer indexer, Drive drive) {
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.drive = drive;
        //              Distance (m)
        firingTable.put(0.0,    getMatrix(0.0, 0.0, 0.0));
        firingTable.put(10.0,   getMatrix(5.0, 10.0, 1.0));
    }

    public Command shoot() {
        return prepShot().andThen(() -> startFeeding()).andThen(Commands.runEnd(() -> setSetpoints(true), () -> stopShooting()));
    }

    public Command prepShot() {
        return Commands.runOnce(() -> setSetpoints(true)).andThen(Commands.waitUntil(flywheel::atRPM));
    }

    public void startFeeding() {
        indexer.startIndexer();
        indexer.startKicker();
    }

    public void stopShooting() {
        flywheel.setRPM(0);
        indexer.stopIndexer();
        indexer.stopKicker();
    }

    public Command trackTarget() {
        return Commands.run(
            () -> setSetpoints(false)
        );
    }

    public Translation2d getTargetTranslation() {
        // Add joystick manipulation
        return AimUtil.getHubTranslation();
    }

    public double getDistanceToTargetFromRobotPose(Pose2d robotPose) {
        Translation2d turretTranslation = AimUtil.getTurretTranslationFromRobotPose(robotPose);
        return turretTranslation.getDistance(getTargetTranslation());
    }

    public void setSetpoints(boolean setFlywheel) {
        double phaseDelay = AutoAimConstants.PHASE_DELAY;

        Pose2d estimatedPose = drive.getPose();
        ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

        estimatedPose =
            estimatedPose.exp(
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
            lookaheadPose =
                estimatedPose.plus(new Transform2d(robotDelta.vxMetersPerSecond, robotDelta.vyMetersPerSecond, new Rotation2d(robotDelta.omegaRadiansPerSecond)));
            lookaheadTurretToTargetDistance = getDistanceToTargetFromRobotPose(lookaheadPose);
        }

        Matrix<N3, N1> firingValues = getFiringTableValues(lookaheadTurretToTargetDistance);

        Pose2d robotPoseForTurret = new Pose2d(lookaheadPose.getX(), lookaheadPose.getY(), estimatedPose.getRotation());

        turret.setSetpointFromTurretPose(new Pose2d(AimUtil.getTurretTranslationFromRobotPose(robotPoseForTurret), estimatedPose.getRotation()), target);
        hood.setAngle(firingValues.get(0, 0));
        if (setFlywheel) {
            flywheel.setRPM(firingValues.get(1, 0));
        }

        Logger.recordOutput("AutoAim/LookaheadPose", lookaheadPose);
        Logger.recordOutput("AutoAim/TurretPose", new Pose2d(AimUtil.getTurretTranslationFromRobotPose(robotPoseForTurret), estimatedPose.getRotation()));
        Logger.recordOutput("AutoAim/TimeOfFlight", timeOfFlight);
    }   

    public Matrix<N3, N1> getFiringTableValues(double distance) {
        return firingTable.get(distance);
    }

    public static Matrix<N3, N1> getMatrix(double hoodSetpoint, double flywheelSetpoint, double flightTimeSeconds) {
        return new Matrix<>(new SimpleMatrix(3, 1, true, new double[]{hoodSetpoint, flywheelSetpoint, flightTimeSeconds}));
    }
}