package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.FeedConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public final class Configs {
  public static final class SwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static final double TURNING_FACTOR = 2 * Math.PI; // The diestance around circle
    static final double drivingFactor =
        ModuleConstants.kWheelDiameterMeters
            * Math.PI
            / ModuleConstants.kDrivingMotorReduction; // TODO: Document why this is the case
    static final double nominalVoltage = 12.0; // TODO: This is correct right?
    static final double drivingVelocityFeedForward =
        nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps; // TODO: Why is this the case?

    static {
      drivingConfig
          .inverted(false) // False for protobot, true for comp
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ModuleConstants.kCurrentLimit);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(
              ModuleConstants.kDrivingP,
              ModuleConstants.kDrivingI,
              ModuleConstants.kDrivingD) // TODO Tune these values
          .outputRange(-1, 1)
          .feedForward
          .kV(drivingVelocityFeedForward);
      turningConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(20);
      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(false)
          .positionConversionFactor(TURNING_FACTOR) // radians
          .velocityConversionFactor(TURNING_FACTOR / 60.0) // radians per second
          // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
          .apply(
              AbsoluteEncoderConfig.Presets
                  .REV_ThroughBoreEncoderV2); // TODO: Check if this is correct encoder
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(
              ModuleConstants.kTurningP,
              ModuleConstants.kTurningI,
              ModuleConstants.kTurningD) // TODO Tune these values
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, TURNING_FACTOR);
    }
  }

  public static final class Shooter {
    public static final SparkFlexConfig leftShooterConfig = new SparkFlexConfig();
    public static final SparkFlexConfig rightShooterConfig = new SparkFlexConfig();

    static {
      // Left shooter follows right shooter (inverted)
      leftShooterConfig.follow(ShooterConstants.kRightShooterCanId, true);

      // Right shooter is the leader - configure as needed
      rightShooterConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
    }
  }

  public static final class Intake {
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();

    static {
      intakeConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(IntakeConstants.kCurrent)
          .inverted(true);
    }
  }

  public static final class Feed {
    public static final SparkMaxConfig beltConfig = new SparkMaxConfig();
    public static final SparkMaxConfig indexerLeftConfig = new SparkMaxConfig();
    public static final SparkMaxConfig indexerRightConfig = new SparkMaxConfig();

    public static final SparkFlexConfig triggerConfig = new SparkFlexConfig();

    static {
      indexerLeftConfig.follow(FeedConstants.kIndexerRightCANId, true);
      indexerRightConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(FeedConstants.kCurrent);

      triggerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(FeedConstants.kCurrent);

      beltConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(FeedConstants.kCurrent);
    }
  }
}
