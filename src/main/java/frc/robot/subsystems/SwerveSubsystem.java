// Copyright (c) FIRST and other WPILib contributors.    
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.TurnBotDeltaAngleCommand;

public class SwerveSubsystem extends SubsystemBase {
    // Create four swerve modules
    private final SwerveModule m_frontLeft = new SwerveModule(
        ModuleConstants.kFrontLeftDrivingCanId,
        ModuleConstants.kFrontLeftTurningCanId
    );

    private final SwerveModule m_frontRight = new SwerveModule(
        ModuleConstants.kFrontRightDrivingCanId,
        ModuleConstants.kFrontRightTurningCanId
    );

    private final SwerveModule m_backLeft = new SwerveModule(
        ModuleConstants.kBackLeftDrivingCanId,
        ModuleConstants.kBackLeftTurningCanId
    );

    private final SwerveModule m_backRight = new SwerveModule(
        ModuleConstants.kBackRightDrivingCanId,
        ModuleConstants.kBackRightTurningCanId
    );

    
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI); // navX gyro plugged into rio's MXP port
    
    // We use these to publish SwerveModuleStates for advantage scope to display
    private final SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4]; // TODO is this being updated?
    StructArrayPublisher<SwerveModuleState> actualStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Actual States", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> desiredStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Desired States", SwerveModuleState.struct).publish();
    

    private SwerveModulePosition[] swerveModulePositions = {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    };

    private final SwerveDriveOdometry odometer = 
        new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), swerveModulePositions);
    
    // SwerveDriveKinematics kinematics = new SwerveDriveKinematics(null);
    boolean isFieldOriented = true;

    public double turnModifier = 0;
    public boolean useDifferentTurnSpeed = false;
    
    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Swerve");

    // the constructor creates the subsystem and initializes variables
    // we pass in a supplier that tells us when to toggle field orientation
    public SwerveSubsystem() {
        updateDashboard();

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        
            AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(0.5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0.5, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            System.out.println("Failed to load AutoBuilder configuration from GUI settings. AutoBuilder will not be configured.");
        }
        
    }

    /**
     * Updates NetworkTable with current values and reads settable parameters.
     */
    private void updateDashboard() {
        m_table.getEntry("Field Oriented").setBoolean(isFieldOriented);
        m_table.getEntry("Gyro Yaw").setDouble(m_gyro.getYaw());

        desiredStatesPublisher.set(swerveModuleStates);
        actualStatePublisher.set(new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState(),
        });

        // Drive motor velocities for each module
        m_table.getEntry("Front Left Drive Velocity").setDouble(Math.abs(m_frontLeft.getDriveVelocity()));
        m_table.getEntry("Front Right Drive Velocity").setDouble(Math.abs(m_frontRight.getDriveVelocity()));
        m_table.getEntry("Back Left Drive Velocity").setDouble(Math.abs(m_backLeft.getDriveVelocity()));
        m_table.getEntry("Back Right Drive Velocity").setDouble(Math.abs(m_backRight.getDriveVelocity()));
        m_table.getEntry("Is Field Relative").setBoolean(isFieldOriented);
        // System.out.println(getCurrentSpeeds());

        // TODO add chassis setspeeds and read speeds to the dashboard
        // TODO add rotation to the dashboard
        // TODO look into using advantagekit for better dashboarding
    }

    /**
     * This method is called once per scheduler run.
     * Used to update anything that needs updating regularly.
     */
    @Override
    public void periodic() {

        swerveModulePositions = new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };

        odometer.update(
            getRotation2d(),
            swerveModulePositions
        );

        updateDashboard();
    }

    /**
     * Sets all swerve modules to the specified angle with zero speed.
     * Useful for debugging module alignment.
     *
     * @param angleDegrees The desired angle in degrees.
     */
    public void setModuleAngles(double angleDegrees) {
        var desiredState = new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(angleDegrees));
        m_frontLeft.setDesiredState(desiredState);
        m_frontRight.setDesiredState(desiredState);
        m_backLeft.setDesiredState(desiredState);
        m_backRight.setDesiredState(desiredState);
        
        SwerveModuleState[] states = {desiredState, desiredState, desiredState, desiredState};
        desiredStatesPublisher.set(states);
    }

    /**
     * Resets the gyro yaw to zero.
     * This sets the current direction as the new forward direction for field-oriented drive.
     */
    public void resetGyro() {
        m_gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return m_gyro.getRotation2d();
    }

    public void resetPose(edu.wpi.first.math.geometry.Pose2d pose) {
        odometer.resetPosition(getRotation2d(), swerveModulePositions, pose);
    }

    public void stopModules() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)));
        m_backRight.setDesiredState(new SwerveModuleState(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)));
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    /**
     * Factory function for the default command for the swerve subsystem.
     * This command drives the robot based on joystick inputs.
     *
     * @param xSpdSupplier Supplier for the x-axis speed.
     * @param ySpdSupplier Supplier for the y-axis speed.
     * @param turningSpdSupplier Supplier for the turning speed.
     * @return The default command for the swerve subsystem.
     */   
    public Command getDefaultCommand(DoubleSupplier xSpdSupplier, DoubleSupplier ySpdSupplier, DoubleSupplier turningSpdSupplier, BooleanSupplier setFieldOriented) {
        return new RunCommand(() -> this.drive(xSpdSupplier.getAsDouble(), ySpdSupplier.getAsDouble(), turningSpdSupplier.getAsDouble(), .02, setFieldOriented.getAsBoolean()), this);
    }

    /**
     * Factory function for the reset gyro command.
     * This command resets the gyro yaw to zero, setting the current direction as the new forward direction.
     *
     * @return A command that resets the gyro when executed.
     */
    public Command getResetGyroCommand() {
        return runOnce(() -> resetGyro());
    }

        /**
     * Drives the robot using the specified speeds Used by the default command.
     *
     * @param xSpeed Speed in the x direction (forward), in meters per second.
     * @param ySpeed Speed in the y direction (sideways), in meters per second.
     * @param rot Rotational speed, in radians per second.
     * @param periodSeconds The time period over which to discretize the speeds.
     */
    public void drive(double xSpeed, double ySpeed, double rot, double periodSeconds, Boolean fieldOriented) {
        isFieldOriented = fieldOriented;
        
        if (useDifferentTurnSpeed) {
            rot = turnModifier;
        }

        // boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
        
        var chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot); // default to field oriented, will be overridden if not field oriented
        
        if (isFieldOriented) { // Why is this not false?
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds,
                getRotation2d()
            );
        }

        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, periodSeconds);


        // var chassisSpeeds = ChassisSpeeds.discretize(
        //     !isFieldOriented
        //         ? ChassisSpeeds.fromFieldRelativeSpeeds(
        //             xSpeed, ySpeed, rot, 
        //             m_gyro.getRotation2d())
        //         : new ChassisSpeeds(xSpeed, ySpeed, rot), periodSeconds); 
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState(),
        });
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        double vx = getCurrentSpeeds().vxMetersPerSecond;
        double vy = getCurrentSpeeds().vyMetersPerSecond;
        double omega = getCurrentSpeeds().omegaRadiansPerSecond;
        return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, new Rotation2d(getHeading()));
    }

    public void setTurnSpeed(double turnSpeed, boolean useLimeTurnSpeed) {
        if (turnModifier >= .5) {
            turnModifier = .5;
        } else if (turnModifier <= -.5) {
            turnModifier = -.5;
        }
        
        turnModifier = -turnSpeed;
        useDifferentTurnSpeed = useLimeTurnSpeed;
    }

    public Command getTurnByDeltaAngleCommand(double deltaDegrees) {
        return new TurnBotDeltaAngleCommand(this, deltaDegrees);
    }
}