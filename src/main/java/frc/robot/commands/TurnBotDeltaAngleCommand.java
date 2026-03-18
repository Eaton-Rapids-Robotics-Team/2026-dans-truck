// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.SwerveSubsystem;

// /**
//  * Command to turn the robot by a specified delta angle. Positive angles result in clockwise
//  * rotation, negative angles in counter-clockwise rotation.
//  */
// public class TurnBotDeltaAngleCommand extends Command {
//   private final SwerveSubsystem m_swerveSubsystem;
//   private final double m_deltaDegrees;
//   private double m_targetAngleDegrees;

//   // PID controller for rotation
//   private final PIDController m_rotationController;

//   // Tolerance for angle completion (degrees)
//   private static final double ANGLE_TOLERANCE = 5.0;

//   // Maximum rotation speed (radians per second)
//   private static final double MAX_ROTATION_SPEED = 1.0;

//   /**
//    * Creates a new TurnBotDeltaAngleCommand.
//    *
//    * @param swerveSubsystem The swerve subsystem to control
//    * @param deltaDegrees The angle to turn in degrees. Positive = CW, Negative = CCW
//    */
//   public TurnBotDeltaAngleCommand(SwerveSubsystem swerveSubsystem, double deltaDegrees) {
//     m_swerveSubsystem = swerveSubsystem;
//     m_deltaDegrees = deltaDegrees;

//     // Create PID controller for rotation
//     // Using similar values to the path planner rotation PID
//     m_rotationController = new PIDController(0.04, 0.0, 0.002);

//     // Enable continuous input for the PID controller (-180 to 180 degrees)
//     m_rotationController.enableContinuousInput(-180, 180);

//     // Set tolerance
//     m_rotationController.setTolerance(ANGLE_TOLERANCE);

//     addRequirements(swerveSubsystem);
//   }

//   @Override
//   public void initialize() {
//     // Get current angle from gyro
//     double currentAngle = m_swerveSubsystem.getHeading();

//     // Calculate target angle
//     m_targetAngleDegrees = currentAngle + m_deltaDegrees;

//     // Normalize to -180 to 180 range
//     m_targetAngleDegrees = normalizeAngle(m_targetAngleDegrees);

//     // Set the setpoint for the PID controller
//     m_rotationController.setSetpoint(m_targetAngleDegrees);
//   }

//   @Override
//   public void execute() {
//     // Get current heading
//     double currentHeading = m_swerveSubsystem.getHeading();

//     // Calculate rotation speed using PID controller
//     double rotationSpeed = m_rotationController.calculate(currentHeading);

//     // Clamp rotation speed to maximum
//     rotationSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationSpeed));

//     // Drive the robot with only rotation (no translation)
//     // Note: The rotation is negated to match the expected CW/CCW behavior
//     m_swerveSubsystem.drive(0, 0, rotationSpeed, 0.02, false);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     // Stop the robot
//     m_swerveSubsystem.stopModules();

//     if (interrupted) {
//       System.out.println("TurnBotDeltaAngleCommand interrupted");
//     } else {
//       System.out.println("TurnBotDeltaAngleCommand completed");
//       System.out.println("  Final angle: " + m_swerveSubsystem.getHeading() + " degrees");
//     }
//   }

//   @Override
//   public boolean isFinished() {
//     // Command is finished when the robot is at the target angle within tolerance
//     return m_rotationController.atSetpoint();
//   }

//   /**
//    * Normalizes an angle to the range [-180, 180].
//    *
//    * @param angle The angle to normalize in degrees
//    * @return The normalized angle in degrees
//    */
//   private double normalizeAngle(double angle) {
//     while (angle > 180) {
//       angle -= 360;
//     }
//     while (angle <= -180) {
//       angle += 360;
//     }
//     return angle;
//   }
// }
