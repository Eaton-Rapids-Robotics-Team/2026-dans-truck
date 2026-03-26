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
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex m_intake =
      new SparkFlex(IntakeConstants.kIntakeCANId, MotorType.kBrushless);
  private final SparkClosedLoopController m_intakeController;

  private double m_desiredVelocityRPM; // Target velocity in RPM

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Intake");

  public IntakeSubsystem() {
    m_intake.configure(
        Configs.Intake.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Get the closed-loop controller for velocity control
    m_intakeController = m_intake.getClosedLoopController();

    m_desiredVelocityRPM = 0;
    updateDashboard();
  }

  private void updateDashboard() {
    m_table.getEntry("Desired Velocity RPM").setDouble(m_desiredVelocityRPM);
    m_table.getEntry("Intake Running Forward").setBoolean(m_desiredVelocityRPM > 0);
    m_table.getEntry("Intake Running Backward").setBoolean(m_desiredVelocityRPM < 0);
    m_table.getEntry("Intake Current").setDouble(m_intake.getOutputCurrent());
    m_table.getEntry("Current Velocity RPM").setDouble(m_intake.getEncoder().getVelocity());
  }

  /**
   * Set the desired intake velocity in RPM.
   *
   * @param velocityRPM The target velocity in RPM (positive for intake, negative for reverse)
   */
  public void setIntakeVelocity(double velocityRPM) {
    m_desiredVelocityRPM = velocityRPM;
  }

  /**
   * Get the current desired intake velocity in RPM.
   *
   * @return The desired velocity in RPM
   */
  public double getIntakeVelocity() {
    return m_desiredVelocityRPM;
  }

  private void setIntakeOn() {
    m_desiredVelocityRPM = IntakeConstants.kDefaultTargetRPM;
  }

  private void setIntakeOff() {
    m_desiredVelocityRPM = 0;
  }

  private void setIntakeReversed() {
    m_desiredVelocityRPM = -IntakeConstants.kDefaultTargetRPM;
  }

  public Command getSetIntakeOnCommand() {
    return Commands.runOnce(() -> this.setIntakeOn(), this);
  }

  public Command getSetIntakeOffCommand() {
    return Commands.runOnce(() -> this.setIntakeOff(), this);
  }

  public Command getSetIntakeReversedCommand() {
    return Commands.runOnce(() -> this.setIntakeReversed(), this);
  }

  /**
   * A factory function that creates a command to run the intake at a specified velocity. Will stop
   * the motor when the command ends.
   *
   * @param velocityRPM The velocity in RPM to run the intake at.
   * @return A Command that runs the intake at the specified velocity.
   */
  public Command getRunIntakeCommand(double velocityRPM) {
    return Commands.startEnd(
        () -> m_desiredVelocityRPM = velocityRPM, () -> m_desiredVelocityRPM = 0, this);
  }

  /**
   * A factory function that creates a command to stop the intake.
   *
   * @return A Command that stops the intake.
   */
  public Command getStopIntakeCommand() {
    return Commands.runOnce(() -> m_desiredVelocityRPM = 0, this);
  }

  public void periodic() {
    // Use velocity control when desired velocity is set, otherwise stop
    if (m_desiredVelocityRPM != 0) {
      // Using setSetpoint with ControlType.kVelocity for closed-loop velocity control
      m_intakeController.setSetpoint(m_desiredVelocityRPM, ControlType.kVelocity);
    } else {
      // Stop the motor
      m_intake.set(0);
    }
    updateDashboard();
  }
}
