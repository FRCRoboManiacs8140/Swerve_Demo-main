package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.MAXConfigure;
// import frc.robot.subsystems.MotorRPMControl;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.PIDController; 
import java.util.Arrays;


public class ShooterSubsystem extends SubsystemBase {
    // Define the leader and follower motors
    // private final MAXConfigure m_shooterLeaderLeftMotor = new MAXConfigure(
    //   DriveConstants.kShooterLeaderLeftMotorCanId);

    // private final MAXConfigure m_shooterFollowerRightMotor = new MAXConfigure(
    //   DriveConstants.kShooterFollowerRightMotorCanId);

    public SparkMax m_shooterFollowerMotor; 
    public SparkMax m_shooterLeaderMotor; 
    public PIDController shooterController;
    public RelativeEncoder m_shooterLeaderEncoder;
    private PIDController pidController;


    // public MotorRPMControl m_shooterFollowerMotorControl;
    // public MotorRPMControl m_shooterLeaderMotorControl;
    
    // private final RelativeEncoder shooter_encoder = m_shooterLeaderMotor.getEncoder();
    // PIDController shooterController = new PIDController(.1, 0, 0);
//    }


    public ShooterSubsystem() {
        // Initialize the motors
        // m_shooterFollowerMotor = new SparkMax(DriveConstants.kShooterFollowerRightMotorCanId, MotorType.kBrushless);
        m_shooterLeaderMotor = new SparkMax(DriveConstants.kShooterLeaderLeftMotorCanId, MotorType.kBrushless);
        m_shooterLeaderEncoder = m_shooterLeaderMotor.getEncoder();
        //shooterController = new PIDController(.1, 0, 0);
        // m_shooterFollowerMotorControl = new MotorRPMControl(DriveConstants.kShooterFollowerRightMotorCanId, DriveConstants.kPShooter, 0, 0); 
        // m_shooterLeaderMotorControl = new MotorRPMControl(DriveConstants.kShooterLeaderLeftMotorCanId, DriveConstants.kPShooter, 0, 0);

        // Invert motors if needed
        // m_shooterFollowerMotor.setInverted(true); 
        // m_shooterFollowerMotor.setInverted(true);


    // // Example data: Distance from target (x) vs. motor power in TargetRPMs (y)
    double[] xData = {7.5, 9.5, 10.5, 12, 13.5};
    double[] yData = {2600, 2700, 3100, 3400, 3800};

    // Get slope (m) and intercept (b) for y = m*x + b
    double slope = 200; //300 per 3/2 feet
    double intercept = 2400;

    double apriltagheight = 3.6875 // In feet. NEEDS TO BE CODED WITH LIMELIGHT
    double limelightheight = 0.5; // In feet. NEEDS TO BE CODED WITH LIMELIGHT

    // System.out.printf("Slope: %.4f, Intercept: %.4f%n", slope, intercept);

    // Predict distance for a new encoder reading
    double targetDistance = (apriltagheight-limelightheight)/Math.tan(VisionConstants.ty); // In feet. NEEDS TO BE CODED WITH LIMELIGHT

    double predictedRPM = slope * targetDistance + intercept;

    // System.out.printf("Predicted distance for %.0f ticks: %.3f meters%n",
    //newEncoderTicks, predictedDistance);
    }

    public void setTargetRPM(double targetRPM) {
        // Get the current RPM from the encoder
        double currentRPM = m_shooterLeaderEncoder.getVelocity();

        // Calculate the PID output
        double output = pidController.calculate(currentRPM, targetRPM);

        // Set the motor speed (clamp output to [-1.0, 1.0])
        m_shooterLeaderMotor.set(Math.max(-1.0, Math.min(1.0, output)));
    }

    public double getCurrentRPM() {
        return m_shooterLeaderEncoder.getVelocity();
    }

    // Method to set the speed of both motors

    public void shoot(double setpoint, double tkP, double tkI, double tkD) {
            // Set PID coefficients
        // PID Controller for shooter
        pidController = new PIDController(tkP, tkI, tkD);

            // Apply PID output to motor
        setTargetRPM(setpoint/5676);
        SmartDashboard.putNumber("Shooter Velocity", m_shooterLeaderEncoder.getVelocity());
        

       // m_shooterFollowerMotorControl.setTargetRPM(speed);
       // m_shooterLeaderMotorControl.setTargetRPM(speed);
    }

    // Method to stop both motors
    public void stop() {
        m_shooterLeaderMotor.stopMotor();
       m_shooterFollowerMotor.stopMotor();
    }
}
    