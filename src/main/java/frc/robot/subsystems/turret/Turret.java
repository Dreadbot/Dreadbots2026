package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.misc.AimUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class Turret extends SubsystemBase {
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretIO io;
    private final PIDController pid = new PIDController(TurretConstants.TURRET_Kp, 0, TurretConstants.TURRET_Kd);
    private final Drive drive;

    private double setpointRelativeRad;
    public double voltage;
    

    public Turret(TurretIO io, Drive drive) {
        this.io = io;
        this.voltage = 0;
        io.updateInputs(inputs);
        pid.setTolerance(Units.degreesToRadians(1));
        this.drive = drive;
        setCorrectAngleRad(0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        double wrappedSetpoint = wrapToLimits(setpointRelativeRad);
        double turretRotationRelative = inputs.turretRotationRad - TurretConstants.TURRET_ZERO_ROBOT_RELATIVE;
        
        voltage = pid.calculate(turretRotationRelative, wrappedSetpoint);
        if (Math.abs(voltage) > 1e-2) {
            voltage += Math.copySign(TurretConstants.TURRET_Ks, voltage);
        }
        
        if (turretRotationRelative >= TurretConstants.MAX_ANGLE_RAD && voltage > 0) {
            voltage = 0.0;
        } else if (turretRotationRelative <= TurretConstants.MIN_ANGLE_RAD && voltage < 0) {
            voltage = 0.0;
        }

        voltage = MathUtil.clamp(voltage, -TurretConstants.MAX_VOLTAGE, TurretConstants.MAX_VOLTAGE);
        io.runTurretVoltage(voltage);

        Logger.recordOutput("Turret/SetpointRelativeRadians", wrappedSetpoint);
        Logger.recordOutput("Turret/CurrentFieldRotationRadians", inputs.turretRotationRad);
        Logger.recordOutput("Turret/CurrentTurretRotationRadians", turretRotationRelative);
        Logger.recordOutput("Turret/Delta", MathUtil.angleModulus(wrappedSetpoint - turretRotationRelative));
        Logger.recordOutput("Turret/AtSetpoint", atSetpoint());
        Logger.recordOutput("Turret/Voltage", voltage);
        Logger.recordOutput("Turret/FieldPose", 
            new Pose2d(AimUtil.getTurretTranslationFromRobotPose(
                drive.getPose()), 
                new Rotation2d(inputs.turretRotationRad).plus(drive.getPose().getRotation())));
        Logger.recordOutput("Turret/SetpointPose", 
            new Pose2d(AimUtil.getTurretTranslationFromRobotPose(
                drive.getPose()), 
                new Rotation2d(wrappedSetpoint).plus(drive.getPose().getRotation())));
        Logger.recordOutput("turretRelativeRotation", robotRelativeToTurretRelative(inputs.turretRotationRad));
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
        setpointRelativeRad = wrapToLimits(robotRelativeToTurretRelative(inputs.turretRotationRad + delta)); 
    }

    public double robotRelativeToTurretRelative(double angleRad) {
        return MathUtil.angleModulus(angleRad - TurretConstants.TURRET_ZERO_ROBOT_RELATIVE);
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

    public void setSetpointFromTurretPose(Pose2d turretPose, Translation2d target) {
        double setpointRad = target.minus(turretPose.getTranslation()).getAngle().getRadians() - turretPose.getRotation().getRadians();
        setCorrectAngleRad(setpointRad);
    }
}