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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

<<<<<<< HEAD
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
=======
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;


public class ShooterSubsystem extends SubsystemBase {
    // Define the leader and follower motors
    private final MAXSwerveShooterModule m_shooter = new MAXSwerveShooterModule(
      DriveConstants.kShooterFollowerRightMotorCanId,
      DriveConstants.kShooterLeaderLeftMotorCanId);

    public SparkMax m_shooterFollowerRightMotor; 
    public SparkMax m_shooterLeaderLeftMotor; 

    public ShooterSubsystem() {
        // Initialize the motors
        m_shooterFollowerRightMotor = new SparkMax(DriveConstants.kShooterFollowerRightMotorCanId, MotorType.kBrushless);
        m_shooterLeaderLeftMotor = new SparkMax(DriveConstants.kShooterLeaderLeftMotorCanId, MotorType.kBrushless);

        // Optionally, invert one motor if needed
        // m_shooterFollowerMotor.setInverted(true); // Example: invert the follower motor
>>>>>>> 7700cc8 (Build Succeeds, successfully implemented ShootCommand ShooterSubsystem and MAXSwerveSHooterModule)
    }

    // Method to set the speed of both motors
    public void shoot(double speed) {
<<<<<<< HEAD
        m_shooterLeaderMotor.set(speed);
        m_shooterFollowerMotor.set(speed);
=======
        //m_shooter.set(speed);
        m_shooterLeaderLeftMotor.set(speed);
        m_shooterFollowerRightMotor.set(speed);
>>>>>>> 7700cc8 (Build Succeeds, successfully implemented ShootCommand ShooterSubsystem and MAXSwerveSHooterModule)
    }

    // Method to stop both motors
    public void stop() {
<<<<<<< HEAD
        m_shooterLeaderMotor.stopMotor();
        m_shooterFollowerMotor.stopMotor();
    }
=======
        m_shooterLeaderLeftMotor.stopMotor();
        m_shooterFollowerRightMotor.stopMotor();
    }
    // private final MAXSwerveShooterModule m_shooter = new MAXSwerveShooterModule(
    //     DriveConstants.kShooterLeaderLeftMotorCanId,
    //     DriveConstants.kShooterFollowerRightMotorCanId);
    
    // public void shoot(double kShooterSpeed) {
    //     m_shooter.set(kShooterSpeed);
    //   }
>>>>>>> 7700cc8 (Build Succeeds, successfully implemented ShootCommand ShooterSubsystem and MAXSwerveSHooterModule)
}
