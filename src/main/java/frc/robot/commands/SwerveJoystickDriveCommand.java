// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ControlConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.SwerveSubsystem;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// public class SwerveJoystickDriveCommand extends Command {
//   private final SwerveSubsystem m_swerveSubsystem;

//   // We use slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
//   private final SlewRateLimiter m_xSpeedLimiter;
//   private final SlewRateLimiter m_ySpeedLimiter;
//   private final SlewRateLimiter m_rotLimiter;

//   private final DoubleSupplier m_xSpdSupplier;
//   private final DoubleSupplier m_ySpdSupplier;
//   private final DoubleSupplier m_turningSpdSupplier;
//   private final BooleanSupplier m_fieldOrientedSupplier;

//   /*
//    * 1.   Constructor - Might have parameters for this command such as target positions of
// devices. Should also set the name of the command for debugging purposes.
//    *  This will be used if the status is viewed in the dashboard. And the command should require
// (reserve) any devices is might use.
//    */
//   public SwerveJoystickDriveCommand(
//       SwerveSubsystem swerveSubsystem,
//       DoubleSupplier xSpdSupplier,
//       DoubleSupplier ySpdSupplier,
//       DoubleSupplier turningSpdSupplier,
//       BooleanSupplier fieldOrientedSUpplier) {
//     // The rate limiter means the rate-of-change limit, in units per second.
//     m_xSpeedLimiter = new SlewRateLimiter(ControlConstants.kSlewRateLimit);
//     m_ySpeedLimiter = new SlewRateLimiter(ControlConstants.kSlewRateLimit);
//     m_rotLimiter = new SlewRateLimiter(ControlConstants.kSlewRateLimit);

//     m_xSpdSupplier = xSpdSupplier;
//     m_ySpdSupplier = ySpdSupplier;
//     m_turningSpdSupplier = turningSpdSupplier;
//     m_fieldOrientedSupplier = fieldOrientedSUpplier;

//     m_swerveSubsystem = swerveSubsystem;

//     addRequirements(swerveSubsystem);
//   }

//   //    initialize() - This method sets up the command and is called immediately before the
// command
//   // is executed for the first time and every subsequent time it is started .
//   //  Any initialization code should be here.
//   @Override
//   public void initialize() {}

//   /*
//    *   execute() - This method is called periodically (about every 20ms) and does the work of the
// command. Sometimes, if there is a position a
//    *  subsystem is moving to, the command might set the target position for the subsystem in
// initialize() and have an empty execute() method.
//    */
//   @Override
//   public void execute() {
//     m_swerveSubsystem.drive(
//         m_xSpeedLimiter.calculate(
//                 MathUtil.applyDeadband(
//                     m_xSpdSupplier.getAsDouble(), ControlConstants.kJoystickDeadband))
//             * DriveConstants.kMaxSpeedMetersPerSecond,
//         m_ySpeedLimiter.calculate(
//                 MathUtil.applyDeadband(
//                     m_ySpdSupplier.getAsDouble(), ControlConstants.kJoystickDeadband))
//             * DriveConstants.kMaxSpeedMetersPerSecond,
//         m_rotLimiter.calculate(
//                 MathUtil.applyDeadband(
//                     m_turningSpdSupplier.getAsDouble(), ControlConstants.kJoystickDeadband))
//             * DriveConstants.kMaxTurnSpeedRadPerSecond,
//         .02,
//         m_fieldOrientedSupplier.getAsBoolean());
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   @Override
//   public void end(boolean interrupted) {}
// }
