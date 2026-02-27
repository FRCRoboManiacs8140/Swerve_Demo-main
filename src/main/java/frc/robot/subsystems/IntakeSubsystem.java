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


public class IntakeSubsystem extends SubsystemBase {
    // Define the leader and follower motors
    private final MAXConfigure m_intake = new MAXConfigure(
      DriveConstants.kIntakeMotorCanId);

    public SparkMax m_intakeMotor; 

    public IntakeSubsystem() {
        // Initialize the motors
        m_intakeMotor = new SparkMax(DriveConstants.kIntakeMotorCanId, MotorType.kBrushless);

        // Invert Motor if needed
        // m_intakeMotor.setInverted(true);
    }

    // Method to set the speed of both motors
    public void intake(double speed) {
        //m_shooter.set(speed);
        m_intakeMotor.set(speed);
    }

    // Method to stop both motors
    public void stop() {
        m_intakeMotor.stopMotor();
    }
}
