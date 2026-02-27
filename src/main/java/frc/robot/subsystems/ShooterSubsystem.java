package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.MAXConfigure;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

        // Optionally, invert one motor if needed
        // m_shooterFollowerMotor.setInverted(true); // Example: invert the follower motor
    }

    // Method to set the speed of both motors
    public void shoot(double speed) {
        //m_shooter.set(speed);
        m_shooterLeaderMotor.set(speed);
        m_shooterFollowerMotor.set(speed);
    }

    // Method to stop both motors
    public void stop() {
        m_shooterLeaderMotor.stopMotor();
        m_shooterFollowerMotor.stopMotor();
    }
}
