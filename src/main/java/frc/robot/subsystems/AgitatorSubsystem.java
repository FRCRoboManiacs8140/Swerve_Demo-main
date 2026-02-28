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


// public class AgitatorSubsystem extends SubsystemBase {
//     // Define the leader and follower motors
//     // private final MAXConfigure m_agitator = new MAXConfigure(
//     //   DriveConstants.kAgitatorMotorCanId);

//     public SparkMax m_agitatorMotor; 

//     public AgitatorSubsystem() {
//         // Initialize the motors
//         m_agitatorMotor = new SparkMax(DriveConstants.kAgitatorMotorCanId, MotorType.kBrushless);

//         // Invert Motor if needed
//         // m_agitatorMotor.setInverted(true); 
//     }

//     // Method to set the speed of both motors
//     public void agitate(double speed) {
//         m_agitatorMotor.set(speed);
//     }

//     // Method to stop both motors
//     public void stop() {
//         m_agitatorMotor.stopMotor();
//     }
// }