package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
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

  private final SparkMax m_dev = new SparkMax(37, MotorType.kBrushless);

  private double m_shooterSpeed;
  private double m_variableSpeed = 0.5; // Starting at 50%

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Shooter");

  private final PIDController m_shooterFeedback =
      new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

  public ShooterSubsystem() {
    m_shooterSpeed = 0;

    m_leftShooter.configure(
        Configs.Shooter.leftShooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rightShooter.configure(
        Configs.Shooter.rightShooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    updateDashboard();
  }

  /** Updates NetworkTable with current values and reads settable parameters. */
  private void updateDashboard() {
    m_table.getEntry("Shooter Speed").setDouble(m_shooterSpeed);
    m_table.getEntry("Shooter Running").setBoolean(m_shooterSpeed > 0);
    m_table.getEntry("Variable Shoot Speed").setDouble(m_variableSpeed);
    m_table.getEntry("Current Rpm").setDouble(m_dev.getEncoder().getVelocity());
  }

  public boolean isShooterRunning() {
    return m_shooterSpeed > 0;
  }

  public void setShooterSpeed(double speed) {
    m_shooterSpeed = speed;
  }

  public double getShooterSpeed() {
    return m_shooterSpeed;
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
   * A factory function that creates a command to set the shooter speed.
   *
   * @param speed The speed to set the shooter to.
   * @return A Command that sets the shooter speed.
   */
  public Command getSetShooterSpeedCommand(double speed) {
    return Commands.runOnce(() -> m_shooterSpeed = speed, this);
  }

  /**
   * A factory function that creates a command to rev the shooter at a specified speed. Will stop
   * the motor when the command ends.
   *
   * @param speed The speed to run the shooter at (0.0 to 1.0).
   * @return A Command that revs the shooter at the specified speed.
   */
  public Command getRevShooterCommand(double speed) {
    return Commands.startEnd(() -> m_shooterSpeed = speed, () -> m_shooterSpeed = 0, this);
  }

  /**
   * A factory function that creates a command to rev the shooter at the variable speed. Will
   * continuously update to match variable speed changes and stop when the command ends.
   *
   * @return A Command that revs the shooter at the current variable speed.
   */
  public Command getRevShooterVariableCommand() {
    return Commands.run(() -> m_shooterSpeed = m_variableSpeed, this)
        .finallyDo(() -> m_shooterSpeed = 0);
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
          m_shooterSpeed = 0;
        },
        this);
  }

  /**
   * A factory function that creates a command to run the shooter in reverse to unclog. Will stop
   * the motor when the command ends.
   *
   * @param speed The speed to run the shooter in reverse (positive value, will be negated).
   * @return A Command that runs the shooter in reverse.
   */
  public Command getUnclogShooterCommand(double speed) {
    return Commands.startEnd(
        () -> m_shooterSpeed = -Math.abs(speed), () -> m_shooterSpeed = 0, this);
  }

  public void runPIDcommand(double DesiredSpeed) {
    setShooterSpeed(m_shooterFeedback.calculate(m_dev.getEncoder().getVelocity(), DesiredSpeed));
  }

  public Command getRunPIDcommand(DoubleSupplier DesiredSpeed) {
    return run(() -> runPIDcommand(DesiredSpeed.getAsDouble()));
  }

  @Override
  public void periodic() {
    m_rightShooter.set(m_shooterSpeed);
    m_dev.set(m_shooterSpeed);
    updateDashboard();
  }
}
