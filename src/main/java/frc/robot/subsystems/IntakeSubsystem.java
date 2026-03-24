package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex m_intake =
    new SparkFlex(IntakeConstants.kIntakeCANId, MotorType.kBrushless);
  private double m_intakeSpeed;

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Intake");

  public IntakeSubsystem() {
    m_intake.configure(
        Configs.Intake.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_intakeSpeed = 0;
    updateDashboard();
  }

  private void updateDashboard() {
    m_table.getEntry("Intake Speed").setDouble(m_intakeSpeed);
    m_table.getEntry("Intake Running Forward").setBoolean(m_intakeSpeed > 0);
    m_table.getEntry("Intake Running Backward").setBoolean(m_intakeSpeed < 0);
    m_table.getEntry("Intake Current").setDouble(m_intake.getOutputCurrent());
    m_table.getEntry("Intake Velocity").setDouble(m_intake.getEncoder().getVelocity());
  }

  private void setIntakeOn() {
    m_intakeSpeed = IntakeConstants.kDefaultIntakeSpeed;
  }

  private void setIntakeOff() {
    m_intakeSpeed = 0;
  }

  private void setIntakeReversed() {
    m_intakeSpeed = -IntakeConstants.kDefaultIntakeSpeed;
  }

  public Command getSetIntakeOnCommand() {
    return new InstantCommand(() -> this.setIntakeOn());
  }

  public Command getSetIntakeOffCommand() {
    return new InstantCommand(() -> this.setIntakeOff());
  }

  public Command getSetIntakeReversedCommand() {
    return new InstantCommand(() -> this.setIntakeReversed());
  }

  public void periodic() {
    m_intake.set(m_intakeSpeed);
    updateDashboard();
  }
}