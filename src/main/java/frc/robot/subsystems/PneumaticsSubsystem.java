package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
  private final Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);

  private final DoubleSolenoid m_climbSolenoid =
      new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 7, 5);
  private final DoubleSolenoid m_intakSolenoid =
      new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 9, 8);

  // Member variables to store desired solenoid states
  private Value m_desiredClimbState = Value.kOff;
  private Value m_desiredIntakeState = Value.kOff;

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Pneumatics");

  // Assuming port 0 on a REV Pneumatic Hub with default CAN ID 1

  public PneumaticsSubsystem() {
    compressor.enableAnalog(90, 120);
  }

  public void setIntakeOff() {
    m_desiredIntakeState = Value.kOff;
  }

  public void setClimbOff() {
    m_desiredClimbState = Value.kOff;
  }

  private void updateDashboard() {
    m_table.getEntry("Compressor Enabled").setBoolean(compressor.isEnabled());
    m_table.getEntry("Compressor Pressure (psi)").setDouble(compressor.getPressure());
    m_table.getEntry("Climb Extended").setBoolean(m_climbSolenoid.get() == Value.kForward);
    m_table.getEntry("Intake Extended").setBoolean(m_intakSolenoid.get() == Value.kForward);
  }

  // Command factory methods for climb
  private void extendClimb() {
    m_desiredClimbState = Value.kForward;
  }

  private void retractClimb() {
    m_desiredClimbState = Value.kReverse;
  }

  private void turnOffClimb() {
    m_desiredClimbState = Value.kOff;
  }

  public Command getClimbExtendCommand() {
    return new StartEndCommand(() -> this.extendClimb(), () -> this.turnOffClimb(), this);
  }

  public Command getClimbRetractCommand() {
    return new StartEndCommand(() -> this.retractClimb(), () -> this.turnOffClimb(), this);
  }

  public Command getInstantClimbExtendCommand() {
    return new InstantCommand(() -> this.extendClimb());
  }

  public Command getInstantClimbRetractCommand() {
    return new InstantCommand(() -> this.retractClimb());
  }

  public Command getClimbOffCommand() {
    return new InstantCommand(() -> this.turnOffClimb());
  }

  // Command factory methods for intake pneumatics
  private void extendIntake() {
    m_desiredIntakeState = Value.kForward;
  }

  private void retractIntake() {
    m_desiredIntakeState = Value.kReverse;
  }

  private void turnOffIntake() {
    m_desiredIntakeState = Value.kOff;
  }

  public Command getIntakeExtendCommand() {
    return new StartEndCommand(() -> this.extendIntake(), () -> this.turnOffIntake(), this);
  }

  public Command getIntakeRetractCommand() {
    return new StartEndCommand(() -> this.retractIntake(), () -> this.turnOffIntake(), this);
  }

  public Command getIntakeOffCommand() {
    return new InstantCommand(() -> this.turnOffIntake());
  }

  public void periodic() {
    // Set solenoids based on desired states
    m_climbSolenoid.set(m_desiredClimbState);
    m_intakSolenoid.set(m_desiredIntakeState);

    updateDashboard();
  }
}
