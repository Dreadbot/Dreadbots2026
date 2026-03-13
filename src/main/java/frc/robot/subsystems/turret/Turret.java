package frc.robot.subsystems.turret;

import java.util.function.BooleanSupplier;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Turret extends SubsystemBase {
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretIO io;
    private final PIDController pid = new PIDController(TurretConstants.TURRET_Kp, 0, TurretConstants.TURRET_Kd);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(TurretConstants.MAX_VELOCITY, TurretConstants.MAX_ACCELERATION));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private final Drive drive;

    private double setpointRelativeRad;
    public double voltage;
    private boolean lock = false;
    

    public Turret(TurretIO io, Drive drive) {
        SmartDashboard.putData("TurretPID", pid);
        this.io = io;
        this.voltage = 0;
        io.updateInputs(inputs);
        pid.setTolerance(Units.degreesToRadians(3));
        this.drive = drive;
        //setCorrectAngleRad(0.0);
        setpointRelativeRad = 0;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        double turretRotationRelative = inputs.turretRotationRad - TurretConstants.TURRET_ZERO_ROBOT_RELATIVE;
        Logger.recordOutput("Turret/CurrentTurretRotationRadians", -(turretRotationRelative + TurretConstants.TURRET_ZERO_ROBOT_RELATIVE));

        if (DriverStation.isDisabled()) {
            setpoint = new TrapezoidProfile.State(inputs.turretRotationRad - TurretConstants.TURRET_ZERO_ROBOT_RELATIVE, 0);
            goal = setpoint;
            io.runTurretVoltage(0.0);
            return;
        }

        setpoint = profile.calculate(0.02, setpoint, goal);
        double wrappedSetpoint = wrapToLimits(setpoint.position);
        
        
        voltage = pid.calculate(turretRotationRelative, wrappedSetpoint);
        if (Math.abs(voltage) > 1e-1) {
            voltage += Math.copySign(TurretConstants.TURRET_Ks, voltage);
        }
        
        if (turretRotationRelative >= TurretConstants.MAX_ANGLE_RAD && voltage > 0) {
            voltage = 0.0;
        } else if (turretRotationRelative <= TurretConstants.MIN_ANGLE_RAD && voltage < 0) {
            voltage = 0.0;
        }

        voltage = MathUtil.clamp(voltage, -TurretConstants.MAX_VOLTAGE, TurretConstants.MAX_VOLTAGE);

        if (!lock) {
            io.runTurretVoltage(voltage);
        }

        Logger.recordOutput("Turret/SetpointRelativeRadians", wrappedSetpoint);
        // Logger.recordOutput("Turret/CurrentFieldRotationRadians", inputs.turretRotationRad);
        // Logger.recordOutput("Turret/Delta", MathUtil.angleModulus(wrappedSetpoint - turretRotationRelative));
        Logger.recordOutput("Turret/AtSetpoint", atSetpoint());
        Logger.recordOutput("Turret/Voltage", voltage);
        Logger.recordOutput("Turret/FieldPose", 
            new Pose2d(AimUtil.getTurretTranslationFromRobotPose(
                drive.getPose()), 
                new Rotation2d(inputs.turretRotationRad).plus(drive.getPose().getRotation())));
        // Logger.recordOutput("Turret/SetpointPose", 
        //     new Pose2d(AimUtil.getTurretTranslationFromRobotPose(
        //         drive.getPose()), 
        //         new Rotation2d(wrappedSetpoint).plus(drive.getPose().getRotation())));
        // Logger.recordOutput("turretRelativeRotation", robotRelativeToTurretRelative(inputs.turretRotationRad));
    }   
    
    public Command setAngleRad(double angleRad) {
        return run(
            () -> {
                setCorrectAngleRad(angleRad); 
            } );
    }

    public Command toggleLock() {
        return Commands.runOnce(
            () -> {
                lock = !lock;
            }, this);
    }

    public void setCorrectAngleRad(double angleRad) {
        double currentPositionWrapped = MathUtil.angleModulus(inputs.turretRotationRad);
        double delta = MathUtil.angleModulus(MathUtil.angleModulus(angleRad) - currentPositionWrapped);
        setpointRelativeRad = wrapToLimits(robotRelativeToTurretRelative(inputs.turretRotationRad + delta)); 
        goal = new TrapezoidProfile.State(setpointRelativeRad, 0);
    }

    public double robotRelativeToTurretRelative(double angleRad) {
        return MathUtil.angleModulus(angleRad - TurretConstants.TURRET_ZERO_ROBOT_RELATIVE);
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

    public double getVoltage() {
        return inputs.turretAppliedVolts;
    }
}