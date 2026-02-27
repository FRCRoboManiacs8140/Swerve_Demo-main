package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.MAXConfigure;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ShooterSubsystem extends SubsystemBase {
    // Define the leader and follower motors
    private final MAXConfigure m_shooterLeaderLeftMotor = new MAXConfigure(
      DriveConstants.kShooterLeaderLeftMotorCanId);

    private final MAXConfigure m_shooterFollowerRightMotor = new MAXConfigure(
      DriveConstants.kShooterFollowerRightMotorCanId);

    public SparkMax m_shooterFollowerMotor; 
    public SparkMax m_shooterLeaderMotor; 

    public ShooterSubsystem() {
        // Initialize the motors
        m_shooterFollowerMotor = new SparkMax(DriveConstants.kShooterFollowerRightMotorCanId, MotorType.kBrushless);
        m_shooterLeaderMotor = new SparkMax(DriveConstants.kShooterLeaderLeftMotorCanId, MotorType.kBrushless);

        // Invert motors if needed
        // m_shooterFollowerMotor.setInverted(true); 
        // m_shooterFollowerMotor.setInverted(true);
    }

    // Method to set the speed of both motors
    public void shoot(double speed) {
        m_shooterLeaderMotor.set(speed);
        m_shooterFollowerMotor.set(speed);
    }

    // Method to stop both motors
    public void stop() {
        m_shooterLeaderMotor.stopMotor();
        m_shooterFollowerMotor.stopMotor();
    }
}
