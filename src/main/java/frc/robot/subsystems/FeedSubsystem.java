package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FeedConstants;

public class FeedSubsystem extends SubsystemBase {
  private final SparkFlex m_trigger =
      new SparkFlex(FeedConstants.kTriggerCANId, MotorType.kBrushless);

  private final SparkMax m_indexerLeft =
      new SparkMax(/*FeedConstants.kIndexerLeftCANId*/ 40, MotorType.kBrushless);
  private final SparkMax m_indexerRight =
      new SparkMax(/*FeedConstants.kIndexerRightCANId*/ 42, MotorType.kBrushless);
  private final SparkMax m_belt =
      new SparkMax(/*FeedConstants.kBeltCANId*/ 41, MotorType.kBrushless);
  private double m_beltSpeed;
  private double m_indexerSpeed;
  private double m_triggerSpeed;
  private double devSpeed;
  // private double m_feedSpeed;
  // private double m_fingerSpeed;

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Feed");

  public FeedSubsystem() {
    m_belt.configure(
        Configs.Feed.beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_indexerLeft.configure(
        Configs.Feed.indexerLeftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_indexerRight.configure(
        Configs.Feed.indexerLeftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_trigger.configure(
        Configs.Feed.triggerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // m_feedSpeed = 0;
    // m_feedSpeed = 0;

    // updateDashboard();
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

  public void setBeltSpeed(double speed) {
    m_beltSpeed = speed;
  }

  public void setIndexerSpeed(double speed) {
    m_indexerSpeed = speed;
  }

  public void setTriggerSpeed(double speed) {
    m_triggerSpeed = speed;
  }

  public void setFeedSpeed(double FeedSpeed) {
    m_beltSpeed = FeedSpeed;
    m_indexerSpeed = FeedSpeed;
    m_triggerSpeed = FeedSpeed;
  }

  public void getStopFeed() {
    m_beltSpeed = 0;
    m_indexerSpeed = 0;
    m_triggerSpeed = 0;
  }

  public Command getBeltCommand() {
    return runOnce(() -> setBeltSpeed(FeedConstants.kBeltSpeed));
  }

  public Command getIndexerCommand() {
    return runOnce(() -> setBeltSpeed(FeedConstants.kBeltSpeed));
  }

  public Command getTriggerCommand() {
    return runOnce(() -> setBeltSpeed(FeedConstants.kBeltSpeed));
  }

  public Command getFeedCommand() {
    // return new ParallelCommandGroup().addCommands(getBeltCommand());
    return Commands.parallel(getBeltCommand(), getIndexerCommand(), getTriggerCommand())
        .finallyDo(() -> getStopFeed());
  }
  // public void setFeedSpeed(double newSpeed, double newFeederSpeed) {
  //   // m_feedSpeed = newSpeed;
  //   // m_fingerSpeed = newFeederSpeed;
  // }

  // public double getFeedSpeed() {
  //   // return m_feedSpeed;
  // }

  // // Command factory methods
  // private void startFeeding() {
  //   // m_feedSpeed = FeedConstants.kDefaultFeedSpeed;
  //   // m_fingerSpeed = FeedConstants.kDefaultFingerSpeed;
  // }

  // private void stopFeeding() {
  //   // m_feedSpeed = 0;
  //   // m_fingerSpeed = 0;
  // }

  // public Command getFeedCommand() {
  //   // return new StartEndCommand(() -> this.startFeeding(), () -> this.stopFeeding(), this);
  // }

  // public Command getFeedStopCommand() {
  //   // return new InstantCommand(() -> this.stopFeeding());
  // }

  // /**
  //  * A factory function that creates a command to run the feed system in reverse to unclog.

  //  * stop the motors when the command ends.
  //  *
  //  * @return A Command that runs the feed system in reverse.
  //  */
  // public Command getUnclogFeedCommand() {
  //   // return new StartEndCommand(
  //   //     () -> {
  //   //       m_feedSpeed = -FeedConstants.kDefaultFeedSpeed;
  //   //       m_fingerSpeed = -FeedConstants.kDefaultFingerSpeed;
  //   //     },
  //   //     () -> this.stopFeeding(),
  //   //     this);
  // }

  public void periodic() {
    m_belt.set(m_beltSpeed);
    m_indexerRight.set(m_indexerSpeed);
    m_trigger.set(m_triggerSpeed);

    updateDashboard();
  }
}
