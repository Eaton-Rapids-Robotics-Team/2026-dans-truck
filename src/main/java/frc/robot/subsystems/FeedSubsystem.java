package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;
import frc.robot.Constants.FeedConstants;

public class FeedSubsystem extends SubsystemBase {
    private final SparkFlex m_feeder = new SparkFlex(FeedConstants.kFeedCanId, MotorType.kBrushless );

    private final SparkMax m_sorterLeft = new SparkMax(FeedConstants.kSorterLeftCANId, MotorType.kBrushless);
    private final SparkMax m_sorterRight = new SparkMax(FeedConstants.kSorterRightCANId, MotorType.kBrushless);

    private double m_feedSpeed;
    private double m_fingerSpeed;
    
    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Feed");

    public FeedSubsystem() {
        m_sorterLeft.configure(Configs.Feed.fingerLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_sorterRight.configure(Configs.Feed.fingerRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_feeder.configure(Configs.Feed.feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_feedSpeed = 0;
        m_feedSpeed = 0;
        
        updateDashboard();
    }

    private void updateDashboard() {
        m_table.getEntry("Feed Speed").setDouble(m_feedSpeed);
        m_table.getEntry("Finger Speed").setDouble(m_fingerSpeed);
        m_table.getEntry("Feed Running").setBoolean(m_feedSpeed > 0);
        m_table.getEntry("Finger Running").setBoolean(m_fingerSpeed > 0);
    }

    public void setFeedSpeed(double newSpeed, double newFeederSpeed) {
        m_feedSpeed = newSpeed;
        m_fingerSpeed = newFeederSpeed;
    }
    
    public double getFeedSpeed() {
        return m_feedSpeed;
    }

    // Command factory methods
    private void startFeeding() {
        m_feedSpeed = FeedConstants.kDefaultFeedSpeed;
        m_fingerSpeed = FeedConstants.kDefaultFingerSpeed;
    }

    private void stopFeeding() {
        m_feedSpeed = 0;
        m_fingerSpeed = 0;
    }

    public Command getFeedCommand() {
        return new StartEndCommand(
            () -> this.startFeeding(),
            () -> this.stopFeeding(),
            this
        );
    }

    public Command getFeedStopCommand() {
        return new InstantCommand(() -> this.stopFeeding());
    }

    /**
     * A factory function that creates a command to run the feed system in reverse to unclog.
     * Will stop the motors when the command ends.
     * @return A Command that runs the feed system in reverse.
     */
    public Command getUnclogFeedCommand() {
        return new StartEndCommand(
            () -> {
                m_feedSpeed = -FeedConstants.kDefaultFeedSpeed;
                m_fingerSpeed = -FeedConstants.kDefaultFingerSpeed;
            },
            () -> this.stopFeeding(),
            this
        );
    }
    
    public void periodic() {
        m_feeder.set(-m_feedSpeed);
        m_sorterRight.set(m_fingerSpeed);
        updateDashboard();
    }
}
