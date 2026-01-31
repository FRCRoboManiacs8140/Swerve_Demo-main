// public class MyPathfinder implements Pathfinder{
//     EncoderFollower flFollower = new EncoderFollower(modifier.getFrontLeftTrajectory());   // Front Left wheel
//     EncoderFollower frFollower = new EncoderFollower(modifier.getFrontRightTrajectory());   // Front Right wheel
//     EncoderFollower blFollower = new EncoderFollower(modifier.getBackLeftTrajectory());   // Back Left wheel
//     EncoderFollower brFollower = new EncoderFollower(modifier.getBackRightTrajectory());   // Back Right wheel

//     // Encoder Position is the current, cumulative position of your encoder. If you're using an SRX, this will be the
// // 'getEncPosition' function.
// // 1000 is the amount of encoder ticks per full revolution
// // Wheel Diameter is the diameter of your wheel in meters
// flFollower.configureEncoder(fl_encoder_position, 1000, wheel_diameter);
// frFollower.configureEncoder(fr_encoder_position, 1000, wheel_diameter);
// blFollower.configureEncoder(bl_encoder_position, 1000, wheel_diameter);
// brFollower.configureEncoder(br_encoder_position, 1000, wheel_diameter);

//     // The first argument is the proportional gain. Usually this will be quite high
// // The second argument is the integral gain. This is unused for motion profiling
// // The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
// // The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the 
// //      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
// // The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
// flFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);
// frFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);
// blFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);
// brFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);

// @Override
// public void initialize() {
//     m_initialDistance = m_robotDrive.getPose().getX();
// }

// @Override
// public void execute() { 
//     double output = flFollower.calculate(fl_encoder_position);
// double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(flFollower.getHeading()));    // Bound to -180..180 degrees
// frontLeftWheel.setDirection(desiredHeading);
// frontLeftWheel.setSpeed(output);
// //

// double output = frFollower.calculate(fr_encoder_position);
// double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(frFollower.getHeading()));    // Bound to -180..180 degrees
// frontRightWheel.setDirection(desiredHeading);
// frontRightWheel.setSpeed(output);
// //

// double output = blFollower.calculate(bl_encoder_position);
// double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(blFollower.getHeading()));    // Bound to -180..180 degrees
// backLeftWheel.setDirection(desiredHeading);
// backLeftWheel.setSpeed(output);
// //

// double output = brFollower.calculate(br_encoder_position);
// double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(brFollower.getHeading()));    // Bound to -180..180 degrees
// backRightWheel.setDirection(desiredHeading);
// backRightWheel.setSpeed(output);

// }
//   }