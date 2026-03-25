package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FeedConstants;

public class FeedSubsystem extends SubsystemBase {
  private final SparkFlex m_trigger =
      new SparkFlex(FeedConstants.kTriggerCANId, MotorType.kBrushless);
  private final SparkMax m_indexerLeft =
      new SparkMax(FeedConstants.kIndexerLeftCANId, MotorType.kBrushless);
  private final SparkMax m_indexerRight =
      new SparkMax(FeedConstants.kIndexerRightCANId, MotorType.kBrushless);
  private final SparkMax m_belt = new SparkMax(FeedConstants.kBeltCANId, MotorType.kBrushless);

  private double m_beltSpeed = 0;
  private double m_indexerSpeed = 0;
  private double m_triggerSpeed = 0;

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Feed");

  public FeedSubsystem() {
    m_belt.configure(
        Configs.Feed.beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_indexerLeft.configure(
        Configs.Feed.indexerLeftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_indexerRight.configure(
        Configs.Feed.indexerRightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_trigger.configure(
        Configs.Feed.triggerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void updateDashboard() {
    m_table.getEntry("belt Speed").setDouble(m_beltSpeed);
    m_table.getEntry("indexer Speed").setDouble(m_indexerSpeed);
    m_table.getEntry("indexer Speed").setDouble(m_triggerSpeed);

    m_table.getEntry("Real belt Speed").setDouble(m_belt.get());
    m_table.getEntry("Real right indexer Speed").setDouble(m_indexerRight.get());
    m_table.getEntry("Real left indexer Speed").setDouble(m_indexerLeft.get());
    m_table.getEntry("Real Trigger Speed").setDouble(m_trigger.get());

    m_table.getEntry("Feed Running").setBoolean(m_beltSpeed > 0);
    m_table.getEntry("Finger Running").setBoolean(m_indexerSpeed > 0);
    m_table.getEntry("Trigger Running").setBoolean(m_triggerSpeed > 0);
  }

  public void setBeltSpeed(double newSpeed) {
    m_beltSpeed = newSpeed;
  }

  public void setIndexerSpeed(double newSpeed) {
    m_indexerSpeed = newSpeed;
  }

  public void setTriggerSpeed(double newSpeed) {
    m_triggerSpeed = newSpeed;
  }

  public Command getBeltCommand() {
    return runOnce(() -> setBeltSpeed(FeedConstants.kBeltSpeed));
  }

  public Command getIndexerCommand() {
    return runOnce(() -> setIndexerSpeed(FeedConstants.kIndexerSpeed));
  }

  public Command getTriggerCommand() {
    return runOnce(() -> setTriggerSpeed(FeedConstants.kTriggerSpeed));
  }

  public void getStopFeed() {
    m_beltSpeed = 0;
    m_indexerSpeed = 0;
    m_triggerSpeed = 0;
  }

  public void setSpeeds() {
    m_beltSpeed = FeedConstants.kBeltSpeed;
    m_indexerSpeed = FeedConstants.kIndexerSpeed;
    m_triggerSpeed = FeedConstants.kTriggerSpeed;
  }

  /**
   * Returns a command that runs the feed system while held. The feed stops automatically when the
   * command ends (button released).
   */
  public Command getFeedCommand() {
    return startEnd(
        () -> setSpeeds(), // Start: Set speeds when button pressed
        () -> getStopFeed() // End: Stop when button released
        );
  }

  /**
   * Returns a default command that keeps the feed system stopped. This ensures the feed doesn't run
   * unless commanded.
   */
  public Command getDefaultCommand() {
    return run(() -> getStopFeed());
  }

  public void periodic() {
    m_belt.set(m_beltSpeed);
    m_indexerRight.set(m_indexerSpeed);
    m_trigger.set(m_triggerSpeed);

    updateDashboard();
  }
}
