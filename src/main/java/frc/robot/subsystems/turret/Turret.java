package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.vision.VisionCamera.PoseSupplier;
import frc.robot.util.misc.TurretUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Turret extends SubsystemBase {
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretIO io;
    private final PIDController pid = new PIDController(0.2, 0, 0.003);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.00896);
    private final PoseSupplier poseSupplier;

    private double setpointRelativeRad;
    public double voltage;
    

    public Turret(TurretIO io, PoseSupplier poseSupplier) {
        this.io = io;
        this.voltage = 0;
        io.updateInputs(inputs);
        pid.setTolerance(Units.degreesToRadians(1));
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void periodic() {
        //setCorrectAngleRad(setpointRelativeRad + Units.degreesToRadians(1));
        aimAtHub();
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        double wrappedSetpoint = wrapToLimits(setpointRelativeRad);
        
        voltage = pid.calculate(inputs.turretRotationRad, wrappedSetpoint);
        // + feedforward.calculate(Units.degreesToRadians(50));

        if (inputs.turretRotationRad >= TurretConstants.MAX_ANGLE_RAD && voltage > 0) {
            voltage = 0.0;
        } else if (inputs.turretRotationRad <= TurretConstants.MIN_ANGLE_RAD && voltage < 0) {
            voltage = 0.0;
        }
        
        io.runTurretVoltage(voltage);

        Logger.recordOutput("Turret/SetpointRelativeRadians", wrappedSetpoint);
        Logger.recordOutput("Turret/CurrentPositionRadians", inputs.turretRotationRad);
        Logger.recordOutput("Turret/Delta", MathUtil.angleModulus(wrappedSetpoint - inputs.turretRotationRad));
        Logger.recordOutput("Turret/AtSetpoint", atSetpoint());
        Logger.recordOutput("Turret/Voltage", voltage);
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
        return MathUtil.isNear(setpointRelativeRad, inputs.turretRotationRad, Units.degreesToRadians(1));
    }

    public Command trackHub() {
        return Commands.run(
            () -> {
                aimAtHub();
            },
            this);
    }

    public void aimAtHub() {
        Translation2d hubTranslation = TurretUtil.getHubTranslation();
        Translation2d turretPosition = TurretUtil.getTurretTranslationFromRobotPose(poseSupplier.getPose());

        setCorrectAngleRad(hubTranslation.minus(turretPosition).getAngle().getRadians() - poseSupplier.getPose().getRotation().getRadians());
    }
}