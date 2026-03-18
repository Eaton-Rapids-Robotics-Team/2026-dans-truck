package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorConstants;

@SuppressWarnings({"removal"}) // NOTE THAT THE PHEONIX 5 CANDLE LIBS ARE DEPRECATED
public class LEDSubsystem extends SubsystemBase {
  CANdle lights = new CANdle(10);
  CANdleConfiguration lightConfig = new CANdleConfiguration();

  private Color m_currentColor;

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("LED");

  // Constructor, runs on start up
  public LEDSubsystem() {
    // light configuration
    lightConfig.stripType = LEDStripType.GRB;
    lightConfig.statusLedOffWhenActive = false;
    lightConfig.disableWhenLOS = false;
    lightConfig.vBatOutputMode = VBatOutputMode.Modulated;
    lightConfig.v5Enabled = true;
    lights.configAllSettings(lightConfig);

    this.changeLightColor(ColorConstants.OFF);
    updateDashboard();
  }

  private void setLEDSInternal() {
    lights.setLEDs(
        (int) m_currentColor.red, (int) m_currentColor.green, (int) m_currentColor.blue, 0, 0, 13);
  }

  private void changeLightColor(Color newColor) {
    m_currentColor = newColor;
    this.setLEDSInternal();
  }

  public Command getChangeLightColorCommand(Color newColor) {
    return new InstantCommand(() -> this.changeLightColor(newColor), this);
  }

  private void updateDashboard() {
    m_table.getEntry("Current Color").setString(m_currentColor.toString());
  }

  public void periodic() {
    updateDashboard();
  }
}
