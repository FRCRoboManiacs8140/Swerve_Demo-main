// package frc.robot.subsystems;

// import frc.robot.Constants.DriveConstants;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.pathplanner.lib.config.PIDConstants;
// import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.subsystems.Limelight.LimelightHelpers;
// import frc.robot.subsystems.MAXConfigure;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;


// public class IndexSubsystem extends SubsystemBase {
//     // Define the leader and follower motors
//     // private final MAXConfigure m_index = new MAXConfigure(
//     //   DriveConstants.kIndexMotorCanId);

//     public SparkMax m_indexMotor; 

//     public IndexSubsystem() {
//         // Initialize the motors
//         m_indexMotor = new SparkMax(DriveConstants.kIndexMotorCanId, MotorType.kBrushless);

//         // Invert Motor if needed
//         // m_indexMotor.setInverted(true); 
//     }

//     // Method to set the speed of both motors
//     public void index(double speed) {
//         m_indexMotor.set(speed);
//     }

//     // Method to stop both motors
//     public void stop() {
//         m_indexMotor.stopMotor();
//     }
// }