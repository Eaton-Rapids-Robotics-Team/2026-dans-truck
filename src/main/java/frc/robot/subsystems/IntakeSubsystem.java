package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkFlex m_intake = new SparkFlex(IntakeConstants.kIntakeCANId, MotorType.kBrushless);
    private double m_intakeSpeed;
    
    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Intake");

    public IntakeSubsystem() {
        m_intake.configure(Configs.Intake.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_intakeSpeed = 0;
        updateDashboard();
    }

    private void updateDashboard() {
        m_table.getEntry("Intake Speed").setDouble(m_intakeSpeed);
        m_table.getEntry("Intake Running Forward").setBoolean(m_intakeSpeed > 0);
        m_table.getEntry("Intake Running Backward").setBoolean(m_intakeSpeed < 0);
        m_table.getEntry("Intake Stalled").setBoolean(isIntakeStalled());
        m_table.getEntry("Intake Current").setDouble(m_intake.getOutputCurrent());
        m_table.getEntry("Intake Velocity").setDouble(m_intake.getEncoder().getVelocity());
    }

    private void turnOnIntake() {
        m_intakeSpeed = IntakeConstants.kDefaultIntakeSpeed;
    }
    
    private void turnOnIntakeReversed() {
        m_intakeSpeed = -IntakeConstants.kDefaultIntakeSpeed;
    }

    private void turnOffIntake() {
        m_intakeSpeed = 0;
    }

    public Command getIntakeOffCommand() {
        return new InstantCommand(() -> this.turnOffIntake());
    }

    public Command getIntakeOnCommand() {
        return new InstantCommand(() -> this.turnOnIntake());
    }
    
    public Command getIntakeOnReversedCommand() {
        return new InstantCommand(() -> this.turnOnIntakeReversed());
    }

    public double getIntakeSpeed() {
        return m_intakeSpeed;
    }

    public boolean isIntakeRunning() {
        return m_intakeSpeed > 0;
    }

    public boolean isIntakeRunningReversed() {
        return m_intakeSpeed < 0;
    }

    /**
     * Checks if the intake motor is stalled by monitoring current and velocity.
     * A motor is considered stalled if it's drawing high current but moving slowly.
     * 
     * @return true if the motor is stalled, false otherwise
     */
    public boolean isIntakeStalled() {
        // Get the current draw in amps
        double current = m_intake.getOutputCurrent();
        // Get the velocity in RPM
        double velocity = Math.abs(m_intake.getEncoder().getVelocity());
        
        // Define stall thresholds
        double currentThreshold = 40.0; // Amps - adjust based on your motor
        double velocityThreshold = 100.0; // RPM - adjust based on expected speed
        
        // Motor is stalled if current is high and velocity is low, and we're trying to run it
        return (Math.abs(m_intakeSpeed) > 0.1) && (current > currentThreshold) && (velocity < velocityThreshold);
    }

    public void periodic() {
        m_intake.set(m_intakeSpeed);
        updateDashboard();
    }
}
