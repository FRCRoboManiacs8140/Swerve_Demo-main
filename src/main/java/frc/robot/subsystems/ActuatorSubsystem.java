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


// public class ActuatorSubsystem extends SubsystemBase {
//     // Define the leader and follower motors

//     // private final MAXSwerveModule m_actuatorLeaderLeft = new MAXSwerveModule(
//     //   0,
//     //   DriveConstants.kActuatorLeaderMotorCanId, 
//     //   0);

//     //   private final MAXSwerveModule m_actuatorFollowerRight = new MAXSwerveModule(
//     //   0, 
//     //   DriveConstants.kActuatorFollowerMotorCanId, 
//     //   0);

//     public SparkMax m_actuatorFollowerRightMotor; 
//     public SparkMax m_actuatorLeaderLeftMotor; 

//     public ActuatorSubsystem() {
//         // Initialize the motors
//         m_actuatorFollowerRightMotor = new SparkMax(DriveConstants.kActuatorFollowerMotorCanId, MotorType.kBrushless);
//         m_actuatorLeaderLeftMotor = new SparkMax(DriveConstants.kActuatorLeaderMotorCanId, MotorType.kBrushless);

//         // Invert motors if needed
//         // m_actuatorFollowerRightMotor.setInverted(true); 
//         // m_actuatorLeaderLeftMotor.setInverted(true);
//     }

//     // Method to set the speed of both motors
//     public void deploy(double speed) {
//         // Get the current position of the leader motor
//         double currentPosition = m_actuatorLeaderLeftMotor.getEncoder().getPosition();
    
//         // Calculate the target position for two rotations
//         double targetPosition = DriveConstants.kActuatorExtendedPosition; 
    
//         // Run the motors until the target position is reached
//         while (m_actuatorLeaderLeftMotor.getEncoder().getPosition() < targetPosition) {
//             m_actuatorFollowerRightMotor.set(speed);
//             m_actuatorLeaderLeftMotor.set(speed);
//         }
    
//         // Stop the motors once the target position is reached
//         stop();
//     }

//     public void retract(double speed) { 
//         // Get the current position of the leader motor
//         double currentPosition = m_actuatorLeaderLeftMotor.getEncoder().getPosition();

//         // Calculate the target position for two rotations in the opposite direction
//         double targetPosition = DriveConstants.kActuatorRetractedPosition;

//         // Run the motors until the target position is reached
//         while (m_actuatorLeaderLeftMotor.getEncoder().getPosition() > targetPosition) {
//             m_actuatorFollowerRightMotor.set(-speed);
//             m_actuatorLeaderLeftMotor.set(-speed);
//         }

//         // Stop the motors once the target position is reached
//         stop();
//     }

//     public double getPosition() {
//         double position = m_actuatorLeaderLeftMotor.getEncoder().getPosition();
//         SmartDashboard.putNumber("Actuator Position", position);
//         return position;
//     }

//     // Method to stop both motors
//     public void stop() {
//         m_actuatorFollowerRightMotor.stopMotor();
//         m_actuatorLeaderLeftMotor.stopMotor();
//     }
// }
