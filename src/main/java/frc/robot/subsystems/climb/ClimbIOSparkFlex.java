package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

import frc.robot.Constants.ClimbConstants;

//Redone Code from 2024/2025 Slapdown Algae I wrote to get voltage from a motor needs work so commented out for now -Landon

// I think issue is that the Sparkflex Motor has not been defined nad named yet 
public class ClimbIOSparkFlex implements ClimbIO {
    private SparkFlex screwMotor;
        
        
        public void ClimbIOSparkFlex() {
            this.screwMotor = new SparkFlex(ClimbConstants.MOTOR_ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        screwMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(ClimbIOInputs inputs) {
        inputs.appliedVolts = screwMotor.getAppliedOutput() * screwMotor.getBusVoltage();
        inputs.currentAmps = screwMotor.getOutputCurrent();
        inputs.RPM = screwMotor.getEncoder().getVelocity();
    }

    public void runVoltage(double volts) {
        screwMotor.setVoltage(volts);
    }
}

//  public ClimbIOSim() {
        
//          ScrewMotorVolts = 0.0;

//     private double ScrewMotorVolts;
 
//     @Override
//     public void updateInputs(ClimbIOInputs inputs) {
      

//         inputs.screwMotorAppliedVolts = ScrewMotorVolts;
        
       
//         inputs.screwMotorCurrentAmps = ScrewMotor.getCurrentDrawAmps();

//        }  
//     }