package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.misc.TurretUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Turret extends SubsystemBase {
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretIO io;
    private final PIDController pid = new PIDController(6, 0, 0.2);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.5135);
    private final Drive drive;

    private double setpointRelativeRad;
    public double voltage;
    

    public Turret(TurretIO io, Drive drive) {
        this.io = io;
        this.voltage = 0;
        io.updateInputs(inputs);
        pid.setTolerance(Units.degreesToRadians(5));
        this.drive = drive;
    }

    @Override
    public void periodic() {
        //setCorrectAngleRad(setpointRelativeRad + Units.degreesToRadians(0.1));
        aimAtHub();
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        double wrappedSetpoint = wrapToLimits(setpointRelativeRad);
        Twist2d partialTwist = new Twist2d(
            drive.getFieldVelocity().dx     * TurretConstants.TURRET_LOOKAHEAD,
            drive.getFieldVelocity().dy     * TurretConstants.TURRET_LOOKAHEAD,
            drive.getFieldVelocity().dtheta * TurretConstants.TURRET_LOOKAHEAD);

        Pose2d estimatedFutureRobotPose = drive.getPose().exp(partialTwist);
        double estimatedAngleChange = MathUtil.angleModulus(getSetpointFromRobotPose(estimatedFutureRobotPose) - getSetpointFromRobotPose(drive.getPose()));

        voltage = pid.calculate(inputs.turretRotationRad, wrappedSetpoint); 
        double feedforward_voltage = feedforward.calculate(estimatedAngleChange * (1.0 / TurretConstants.TURRET_LOOKAHEAD)); //feedforward.calculate(Units.degreesToRadians(5));// // delta robot rad/s

        if (Math.signum(feedforward_voltage) == Math.signum(voltage)) {
            voltage += feedforward_voltage;
        }
        
        if (inputs.turretRotationRad >= TurretConstants.MAX_ANGLE_RAD && voltage > 0) {
            voltage = 0.0;
        } else if (inputs.turretRotationRad <= TurretConstants.MIN_ANGLE_RAD && voltage < 0) {
            voltage = 0.0;
        }
        voltage = MathUtil.clamp(voltage, -TurretConstants.MAX_VOLTAGE, TurretConstants.MAX_VOLTAGE);
        io.runTurretVoltage(voltage);

        Logger.recordOutput("Turret/SetpointRelativeRadians", wrappedSetpoint);
        Logger.recordOutput("Turret/CurrentPositionRadians", inputs.turretRotationRad);
        Logger.recordOutput("Turret/Delta", MathUtil.angleModulus(wrappedSetpoint - inputs.turretRotationRad));
        Logger.recordOutput("Turret/AtSetpoint", atSetpoint());
        Logger.recordOutput("Turret/Voltage", voltage);
        Logger.recordOutput("Turret/EstimatedAngleChange", estimatedAngleChange);
        Logger.recordOutput("Turret/FieldPose", 
            new Pose2d(TurretUtil.getTurretTranslationFromRobotPose(
                drive.getPose()), 
                new Rotation2d(inputs.turretRotationRad).plus(drive.getPose().getRotation())));
    }   
    
    public Command setAngleRad(double angleRad) {
        return runOnce(
            () -> {
                setCorrectAngleRad(angleRad); 
            } );
    }

    public void setCorrectAngleRad(double angleRad) {
        double currentPositionWrapped = MathUtil.angleModulus(inputs.turretRotationRad);
        double delta = MathUtil.angleModulus(MathUtil.angleModulus(angleRad) - currentPositionWrapped);
        setpointRelativeRad = wrapToLimits(inputs.turretRotationRad + delta); 
    }

    public Command setAtZero() {
        return runOnce(
            () -> {
                setpointRelativeRad = 0.0;
            } );
    }

    public static double wrapToLimits(double angleRad) {
        while (angleRad > TurretConstants.MAX_ANGLE_RAD) angleRad -= 2 * Math.PI;
        while (angleRad < TurretConstants.MIN_ANGLE_RAD) angleRad += 2 * Math.PI;
        return angleRad;
    }

    public double getAngle() {
        return inputs.turretRotationRad;
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    public Command trackHub() {
        return Commands.run(
            () -> {
                aimAtHub();
            },
            this);
    }

    public void aimAtHub() {
        setCorrectAngleRad(getSetpointFromRobotPose(drive.getPose()));
    }

    public double getSetpointFromRobotPose(Pose2d robotPose) {
        Translation2d hubTranslation = TurretUtil.getHubTranslation();
        Translation2d turretPosition = TurretUtil.getTurretTranslationFromRobotPose(robotPose);

        return hubTranslation.minus(turretPosition).getAngle().getRadians() - robotPose.getRotation().getRadians();
    }
}