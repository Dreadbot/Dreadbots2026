package frc.robot.subsystems.hookprep;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import frc.robot.Constants.HookprepConstants;

public class HookprepIOServo implements HookprepIO {
    // Initialize the servo hub
    private ServoHub servoHub;
    private ServoChannel channel0;
    
    public HookprepIOServo() {
        servoHub = new ServoHub(HookprepConstants.Servo_ID);
        channel0 = servoHub.getServoChannel(ChannelId.kChannelId0);
    }

    // suggested maybe?
    @Override
    public void updateInputs(HookprepIOInputs inputs) {
        
    }

    @Override
    public void runPulse(double volts) {
        int pulseWidth = (int)(volts / 12.0 * 2000) + 500; // Assuming 0-12V maps to 500-2500 microseconds
        channel0.setPulseWidth(pulseWidth);
    }
}
