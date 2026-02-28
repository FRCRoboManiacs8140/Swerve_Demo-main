// package frc.robot.Commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.subsystems.ActuatorSubsystem;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase;



// public class DeployActuatorCommand extends Command {
    
//   // Instantiate Stuff
// public ActuatorSubsystem m_robotActuator;
//     double m_initialDistance;
//     double m_distance;
    
// // Make the Agitate Command
// public DeployActuatorCommand(ActuatorSubsystem robotActuator, double distance) {
//     m_robotActuator = robotActuator;
//     m_distance = distance;
//     addRequirements(m_robotActuator);
// }

// @Override
// public void initialize() {
//     m_initialDistance = m_robotActuator.getPosition();
// }

// // Run the Agitate Command
// @Override
// public void execute() {
//     m_robotActuator.deploy(DriveConstants.kActuatorSpeed);
// }

// // If command is interrupted or ends, stop the agitator
// @Override
// public void end(boolean interrupted) {
//     m_robotActuator.stop();
// }

// @Override
// public boolean isFinished() {
//     if (m_robotActuator.getPosition() - m_initialDistance >= m_distance) {
//         return true;
//     } else {
//         return false;
//     }
// }
// }