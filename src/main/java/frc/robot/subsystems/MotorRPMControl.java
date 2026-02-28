package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.controller.PIDController; 
import edu.wpi.first.wpilibj.Encoder; 

public class MotorRPMControl {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final PIDController pidController;

    public MotorRPMControl(int motorCANID, double tkP, double tkI, double tkD) {
        // Create motor object
        motor = new SparkMax(motorCANID, MotorType.kBrushless);

        // Get encoder and PID controller
        encoder = motor.getEncoder();
        pidController = new PIDController(tkP, tkI, tkD);

        // Link PID controller to encoder
        // pidController.setFeedbackDevice(encoder);

        // Set PID coefficients
        // pidController.setP(DriveConstants.kPShooter);
        // pidController.setI(DriveConstants.kIShooter);
        // pidController.setD(DriveConstants.kDShooter);
        // pidController.setFF(DriveConstants.kFFShooter);
        // pidController.setOutputRange(DriveConstants.kMinOutputShooter, DriveConstants.kMaxOutputShooter);
    }

    /**
     * Set the target RPM for the motor.
     * @param targetRPM Desired motor speed in RPM.
     */
    public void setTargetRPM(double targetRPM) {
        // Get the current RPM from the encoder
        double currentRPM = encoder.getVelocity();

        // Calculate the PID output
        double output = pidController.calculate(currentRPM, targetRPM);

        // Set the motor speed (clamp output to [-1.0, 1.0])
        motor.set(Math.max(-1.0, Math.min(1.0, output)));
    }

    /**
     * Get the current motor RPM.
     */
    public double getCurrentRPM() {
        return encoder.getVelocity();
    }

    /**
     * Stop the motor.
     */
    public void stop() {
        motor.set(0);
    }
}
