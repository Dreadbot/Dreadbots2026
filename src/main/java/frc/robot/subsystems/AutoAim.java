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
        return prepShot().andThen(Commands.runEnd(() -> shootSequence(), () -> stopShooting()));
    }

    public Command prepShot() {
        return Commands.runOnce(() -> startShootSequence()).andThen(Commands.waitUntil(flywheel::atRPM));
    }

    public void startShootSequence() {
        setSetpoints();
        indexer.startIndexer();
        indexer.startKicker();
    }

    public void shootSequence() {
        setSetpoints();
    }

    public void stopShooting() {
        flywheel.setRPM(0);
        indexer.stopIndexer();
        indexer.stopKicker();
    }

    public Translation2d getTargetTranslation() {
        // Add joystick manipulation
        return AimUtil.getHubTranslation();
    }

    public double getDistanceToTargetFromRobotPose(Pose2d robotPose) {
        Translation2d turretTranslation = AimUtil.getTurretTranslationFromRobotPose(robotPose);
        return turretTranslation.getDistance(getTargetTranslation());
    }

    public void setSetpoints() {
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
        Pose2d turretPosition = estimatedPose.plus(new Transform2d(TurretConstants.TURRET_OFFSET, Rotation2d.kZero));
        double turretToTargetDistance = turretPosition.getTranslation().getDistance(getTargetTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, drive.getRotation());
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX =
            robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (TurretConstants.TURRET_OFFSET.getY() * Math.cos(robotAngle)
                        - TurretConstants.TURRET_OFFSET.getX() * Math.sin(robotAngle));
        
        double turretVelocityY =
            robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (TurretConstants.TURRET_OFFSET.getX() * Math.cos(robotAngle)
                        - TurretConstants.TURRET_OFFSET.getY() * Math.sin(robotAngle));

        double timeOfFlight = 0;
        Pose2d lookaheadPose = drive.getPose();
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        
        for (int i = 0; i < 20; i++) {
            timeOfFlight = firingTable.get(lookaheadTurretToTargetDistance).get(2, 0);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose =
                new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        Matrix<N3, N1> firingValues = getFiringTableValues(lookaheadTurretToTargetDistance);
        turret.setSetpointFromTurretPose(lookaheadPose, target);
        hood.setAngle(firingValues.get(0, 0));
        flywheel.setRPM(firingValues.get(1, 0));

        Logger.recordOutput("AutoAim/LookaheadPose", lookaheadPose);
        Logger.recordOutput("AutoAim/TimeOfFlight", timeOfFlight);
    }   

    public Matrix<N3, N1> getFiringTableValues(double distance) {
        return firingTable.get(distance);
    }

    public static Matrix<N3, N1> getMatrix(double hoodSetpoint, double flywheelSetpoint, double flightTimeSeconds) {
        return new Matrix<>(new SimpleMatrix(3, 1, true, new double[]{hoodSetpoint, flywheelSetpoint, flightTimeSeconds}));
    }
}