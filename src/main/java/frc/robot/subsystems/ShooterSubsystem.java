package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex m_leftShooter =
      new SparkFlex(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
  private final SparkFlex m_rightShooter =
      new SparkFlex(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

  private final SparkClosedLoopController m_shooterController;

  // private final SparkMax m_dev = new SparkMax(37, MotorType.kBrushless);

  private double m_desiredVelocityRPM; // Target velocity in RPM
  private double m_variableSpeed = 0.5; // Starting at 50%

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Shooter");

  public ShooterSubsystem() {
    m_desiredVelocityRPM = 0;

    m_leftShooter.configure(
        Configs.Shooter.leftShooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_rightShooter.configure(
        Configs.Shooter.rightShooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Get the closed-loop controller for velocity control
    m_shooterController = m_rightShooter.getClosedLoopController();

    updateDashboard();
  }

  /** Updates NetworkTable with current values and reads settable parameters. */
  private void updateDashboard() {
    m_table.getEntry("Desired Velocity RPM").setDouble(m_desiredVelocityRPM);
    m_table.getEntry("Shooter Running").setBoolean(m_desiredVelocityRPM > 0);
    m_table.getEntry("Variable Shoot Speed").setDouble(m_variableSpeed);
    m_table.getEntry("Current RPM").setDouble(m_rightShooter.getEncoder().getVelocity());
  }

  public boolean isShooterRunning() {
    return m_desiredVelocityRPM > 0;
  }

  /**
   * Set the desired shooter velocity in RPM.
   *
   * @param velocityRPM The target velocity in RPM
   */
  public void setShooterVelocity(double velocityRPM) {
    m_desiredVelocityRPM = velocityRPM;
  }

  /**
   * Get the current desired shooter velocity in RPM.
   *
   * @return The desired velocity in RPM
   */
  public double getShooterVelocity() {
    return m_desiredVelocityRPM;
  }

  /** Increment the variable speed by 5%, capped at 1.0. */
  public void incrementVariableSpeed() {
    m_variableSpeed = Math.min(1.0, m_variableSpeed + 0.05);
  }

  /** Decrement the variable speed by 5%, minimum at 0.0. */
  public void decrementVariableSpeed() {
    m_variableSpeed = Math.max(0.0, m_variableSpeed - 0.05);
  }

  /**
   * Get the current variable speed setting.
   *
   * @return The current variable speed (0.0 to 1.0).
   */
  public double getVariableSpeed() {
    return m_variableSpeed;
  }

  /**
   * Set the variable speed to a specific value.
   *
   * @param speed The speed to set (will be clamped between 0.0 and 1.0).
   */
  public void setVariableSpeed(double speed) {
    m_variableSpeed = Math.max(0.0, Math.min(1.0, speed));
  }

  /**
   * A factory function that creates a command to set the shooter velocity.
   *
   * @param velocityRPM The velocity in RPM to set the shooter to.
   * @return A Command that sets the shooter velocity.
   */
  public Command getSetShooterVelocityCommand(double velocityRPM) {
    return Commands.runOnce(() -> m_desiredVelocityRPM = velocityRPM, this);
  }

  /**
   * A factory function that creates a command to rev the shooter at a specified velocity. Will stop
   * the motor when the command ends.
   *
   * @param velocityRPM The velocity in RPM to run the shooter at.
   * @return A Command that revs the shooter at the specified velocity.
   */
  public Command getRevShooterCommand(double velocityRPM) {
    return Commands.startEnd(
        () -> m_desiredVelocityRPM = velocityRPM, () -> m_desiredVelocityRPM = 0, this);
  }

  /**
   * A factory function that creates a command to rev the shooter at the variable speed percentage
   * of max RPM. Will continuously update to match variable speed changes and stop when the command
   * ends.
   *
   * @return A Command that revs the shooter at the current variable speed.
   */
  public Command getRevShooterVariableCommand() {
    // Assuming max RPM is around 5000 - adjust as needed
    return Commands.run(() -> m_desiredVelocityRPM = m_variableSpeed * 5000.0, this)
        .finallyDo(() -> m_desiredVelocityRPM = 0);
  }

  /**
   * A factory function that creates a command to increment the variable speed. Does not require the
   * subsystem, so it won't interrupt the shooter.
   *
   * @return A Command that increments the variable speed by 5%.
   */
  public Command getIncrementSpeedCommand() {
    return Commands.runOnce(() -> incrementVariableSpeed());
  }

  /**
   * A factory function that creates a command to decrement the variable speed. Does not require the
   * subsystem, so it won't interrupt the shooter.
   *
   * @return A Command that decrements the variable speed by 5%.
   */
  public Command getDecrementSpeedCommand() {
    return Commands.runOnce(() -> decrementVariableSpeed());
  }

  /**
   * A factory function that creates a command to reset the variable speed to 50%. Does not require
   * the subsystem, so it won't interrupt the shooter.
   *
   * @return A Command that resets the variable speed to 0.5 (50%).
   */
  public Command getResetVariableSpeedCommand() {
    return Commands.runOnce(() -> setVariableSpeed(0.5));
  }

  /**
   * A factory function that creates a command to stop the shooter.
   *
   * @return A Command that stops the shooter.
   */
  public Command getStopShooterCommand() {
    return Commands.runOnce(
        () -> {
          m_desiredVelocityRPM = 0;
        },
        this);
  }

  /**
   * A factory function that creates a command to run the shooter in reverse to unclog. Will stop
   * the motor when the command ends. Note: Running in reverse uses duty cycle control, not
   * velocity.
   *
   * @param speed The duty cycle speed to run the shooter in reverse (positive value 0-1, will be
   *     negated).
   * @return A Command that runs the shooter in reverse.
   */
  public Command getUnclogShooterCommand(double speed) {
    return Commands.startEnd(
        () -> {
          // Use duty cycle control for reverse (set velocity to 0 to disable PID)
          m_desiredVelocityRPM = 0;
          m_rightShooter.set(-Math.abs(speed));
        },
        () -> m_desiredVelocityRPM = 0,
        this);
  }

  /**
   * Sets the shooter to run at the specified velocity using built-in PID control. This is the
   * recommended method for normal shooting operations.
   *
   * @param desiredVelocityRPM The desired velocity in RPM.
   */
  public void setVelocityControl(double desiredVelocityRPM) {
    m_desiredVelocityRPM = desiredVelocityRPM;
  }

  /**
   * A factory function that creates a command to run the shooter at a specific velocity. Uses the
   * built-in SparkFlex PID velocity control for smooth, consistent operation.
   *
   * @param desiredVelocityRPM A supplier for the desired velocity in RPM.
   * @return A Command that runs the shooter at the specified velocity.
   */
  public Command getRunPIDcommand(DoubleSupplier desiredVelocityRPM) {
    return run(() -> setVelocityControl(desiredVelocityRPM.getAsDouble()));
  }

  @Override
  public void periodic() {
    // Use velocity control when desired velocity is set, otherwise stop
    if (m_desiredVelocityRPM != 0) {
      // Using setReference with ControlType.kVelocity for closed-loop velocity control
      m_shooterController.setReference(m_desiredVelocityRPM, ControlType.kVelocity);
    } else {
      // Stop the motor
      m_rightShooter.set(0);
    }
    updateDashboard();
  }
}
