// Copyright (c) FIRST and other WPILib contributors.    
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Files we made
import frc.robot.Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SwerveModule {
  private final SparkFlex m_drivingSpark;
  private final SparkFlex m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d()); // TODO: Why is this 0 and is it needed?

  /**
   * Constructs a SwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * Swerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   * TODO : Do we use NEOS? How do we have to update for vortex?
   */
  public SwerveModule(int drivingCANId, int turningCANId) {
    m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkFlex(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.SwerveModule.drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.SwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_drivingEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    optimizedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setSetpoint(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Sets the PID values for the turning motor controller.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   */
  public void setTurningPID(double kP, double kI, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.apply(Configs.SwerveModule.turningConfig);
    config.closedLoop.pid(kP, kI, kD);
    m_turningSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Returns the current velocity of the drive motor.
   *
   * @return The current velocity in meters per second.
   */
  public double getDriveVelocity() {
    return m_drivingEncoder.getVelocity();
  }
}    
