package frc.robot.subsystems.hookprep;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import frc.robot.Constants.HookprepConstants;

public class HookprepIOServo implements HookprepIO {
    // Initialize the servo hub
    private ServoHub servoHub;

    // Get the servo channel (assuming channel 1 for this example)
    private ServoChannel channel1;
    
    // Constructor to set up the servo hub and channels
    public HookprepIOServo() {
        // Initialize the servo hub with the specified ID
        servoHub = new ServoHub(HookprepConstants.SERVO_ID);

        // Get the servo channel (assuming channel 0 for this example)
        channel1 = servoHub.getServoChannel(ChannelId.kChannelId0);
    }

    // Method to update inputs (if needed)
    @Override
    public void updateInputs(HookprepIOInputs inputs) {
        
    }

    // Method to run pulse
    @Override
    public void runPulse(double pulseWidthUs) {
        // Log the pulse width for debugging
        System.out.println("HookprepIOServo.runPulse pulseWidthUs=" + pulseWidthUs);

        // Convert pulse width in microseconds to the appropriate value for the servo channel
        int pw = (int)(pulseWidthUs / 12.0 * 2000) + 500; // pw = pulse width in microseconds, scaled to the servo's expected range (500-2500 microseconds)

        // Set the pulse width on the servo channel
        channel1.setPulseWidth(pw);
    }
}
