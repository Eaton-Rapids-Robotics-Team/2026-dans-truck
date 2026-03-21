// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControlConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSpark;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // declare input devices
  private final CommandJoystick m_driverLeft;
  private final CommandJoystick m_driverRight;
  private final CommandJoystick m_buttonBoard;

  // Subsystems
  private final Drive drive;
  private final ShooterSubsystem m_shooterSubsystem;
  // private final FeedSubsystem m_feedSubsystem;
  // private final LimeLightSubsystem m_limeLightSubsystem;
  // private final IntakeSubsystem m_intakeSubsystem;
  // private final PneumaticsSubsystem m_PneumaticsSubsystem;

  // Controller
  // private final CommandXboxController controller = new CommandXboxController(0);

  // Auto chooser
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // NetworkTable for game information
  private final NetworkTable m_gameInfoTable =
      NetworkTableInstance.getDefault().getTable("Game Info");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // initialize all objects in the constructor
    m_driverLeft = new CommandJoystick(ControlConstants.kDriverLeftPort);
    m_driverRight = new CommandJoystick(ControlConstants.kDriverRightPort);
    m_buttonBoard = new CommandJoystick(ControlConstants.kButtonBoardPort);

    // we pass suppliers to the subsystems for any joystick inputs they need
    // this allows them to get the latest values when needed
    m_shooterSubsystem = new ShooterSubsystem();
    // m_swerveSubsystem = new SwerveSubsystem();
    // m_intakeSubsystem = new IntakeSubsystem();
    // m_limeLightSubsystem = new LimeLightSubsystem();
    // m_feedSubsystem = new FeedSubsystem();
    // m_PneumaticsSubsystem = new PneumaticsSubsystem();
    // m_LEDSubsystem = new LEDSubsystem();

    // NamedCommands.registerCommand("set lights red",
    // m_LEDSubsystem.getChangeLightColorCommand(Color.kRed));// TODO make a turn command
    // NamedCommands.registerCommand("turn 45 CW",
    // m_swerveSubsystem.getTurnByDeltaAngleCommand(-50));
    // NamedCommands.registerCommand("Rev Shooter", m_shooterSubsystem.getRevShooterCommand(.5));
    // NamedCommands.registerCommand("Run Feed", m_feedSubsystem.getFeedCommand());
    // NamedCommands.registerCommand("Extend Climb",
    // m_PneumaticsSubsystem.getInstantClimbExtendCommand());
    // NamedCommands.registerCommand("Retract Climb",
    // m_PneumaticsSubsystem.getInstantClimbRetractCommand());
    // NamedCommands.registerCommand("Zero Gyro", new InstantCommand(() ->
    // m_swerveSubsystem.resetGyro()));

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

        // case SIM:
        //   // Sim robot, instantiate physics sim IO implementations
        //   drive =
        //       new Drive(
        //           new GyroIO() {},
        //           new ModuleIOSim(),
        //           new ModuleIOSim(),
        //           new ModuleIOSim(),
        //           new ModuleIOSim());
        //   break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIONavX() {}, // todo fix this
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // We separate the bindings configuration into its own method for clarity
    // This is where controls are connected to commands
    configureBindings();

    // Configure the button bindings
    configureDriveControlsBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriveControlsBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> m_driverLeft.getRawAxis(ControlConstants.kMoveYJoystick),
            () -> m_driverLeft.getRawAxis(ControlConstants.kMoveXJoystick),
            () -> -m_driverRight.getRawAxis(ControlConstants.kRotateJoystick)));

    // Robot-centric drive when button 2 is held (red trigger stage two on left stick)
    m_driverLeft
        .button(ControlConstants.kRobotCentricButton)
        .whileTrue(
            DriveCommands.joystickDriveRobotCentric(
                drive,
                () -> m_driverLeft.getRawAxis(ControlConstants.kMoveYJoystick),
                () -> m_driverLeft.getRawAxis(ControlConstants.kMoveXJoystick),
                () -> -m_driverRight.getRawAxis(ControlConstants.kRotateJoystick)));

    // Lock to 0° when Trigger is held
    m_driverLeft
        .button(1)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> m_driverLeft.getRawAxis(ControlConstants.kMoveYJoystick),
                () -> m_driverLeft.getRawAxis(ControlConstants.kMoveXJoystick),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    m_driverLeft
        .button(3)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Auto-aim at field center when auto-aim button is held
    m_driverLeft
        .button(ControlConstants.kAutoAimButton)
        .and(drive.getManualTrigger().negate())
        .whileTrue(
            DriveCommands.autoAimAtFieldCenter(
                drive,
                () -> m_driverLeft.getRawAxis(ControlConstants.kMoveYJoystick),
                () -> m_driverLeft.getRawAxis(ControlConstants.kMoveXJoystick)))
        .whileFalse(m_shooterSubsystem.getRunPIDcommand(() -> 1000));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureBindings() {
    // LED Triggers
    // m_LEDSubsystem.setDefaultCommand(m_LEDSubsystem.getChangeLightColorCommand(Constants.ColorConstants.OFF));
    // Trigger inTakeOnTrigger = new Trigger(() -> m_intakeSubsystem.isIntakeRunning());
    // inTakeOnTrigger
    //   .whileTrue(m_LEDSubsystem.getChangeLightColorCommand(Constants.ColorConstants.GREEN))
    //   .onFalse(m_LEDSubsystem.getChangeLightColorCommand(Constants.ColorConstants.OFF));

    // Trigger on right stick: Reset field oriented zero (reset gyro)
    // new JoystickButton(m_driverLeft,
    // ControlConstants.kResetFieldButton).onTrue(m_swerveSubsystem.getResetGyroCommand());

    // // Shooter commands - pinky buttons are hardcoded, rev shoot button is variable
    // m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.getDefaultCommand());

    // Max velocity is 4000?
    new JoystickButton(m_driverLeft.getHID(), ControlConstants.kRevShootButton)
        .whileTrue(m_shooterSubsystem.getRunPIDcommand(() -> 1000))
        // .onFalse(m_shooterSubsystem.getResetVariableSpeedCommand()) // Reset to 50% on release
        .onFalse(m_shooterSubsystem.getRunPIDcommand(() -> 0));

    new JoystickButton(m_driverRight.getHID(), ControlConstants.kAutoAimButton)
        .onTrue(drive.getToggleCommand());
    // new JoystickButton(m_driverRight,
    // ControlConstants.kRightPinkyButton).whileTrue(m_shooterSubsystem.getRevShooterCommand(0.65));
    // // Hardcoded 65%
    // new JoystickButton(m_driverLeft,
    // ControlConstants.kLeftPinkyButton).whileTrue(m_shooterSubsystem.getRevShooterCommand(0.75));
    // // Hardcoded 75%

    // // Unclog command - runs feed system and fingers in reverse
    // new JoystickButton(m_driverLeft, ControlConstants.kUnclogButton)
    //   .whileTrue(m_feedSubsystem.getUnclogFeedCommand());

    // // Button board speed control
    // new JoystickButton(m_buttonBoard,
    // ControlConstants.kShooterSpeedUpButton).onTrue(m_shooterSubsystem.getIncrementSpeedCommand());
    // new JoystickButton(m_buttonBoard,
    // ControlConstants.kShooterSpeedDownButton).onTrue(m_shooterSubsystem.getDecrementSpeedCommand());

    // m_PneumaticsSubsystem.setDefaultCommand(m_PneumaticsSubsystem.getDefaultCommand());
    // Trigger intakePistonTrigger = new JoystickButton(m_buttonBoard,
    // ControlConstants.kIntakeExtendButton);
    // intakePistonTrigger.onTrue(m_PneumaticsSubsystem.getIntakeExtendCommand());

    // m_intakeSubsystem.setDefaultCommand(m_intakeSubsystem.getDefaultCommand());
    // new JoystickButton(m_buttonBoard,
    // ControlConstants.kIntakeOnButton).onTrue(m_intakeSubsystem.getIntakeOnCommand());
    // new JoystickButton(m_buttonBoard,
    // ControlConstants.kIntakeOffButton).onTrue(m_intakeSubsystem.getIntakeOffCommand());

    // new JoystickButton(m_buttonBoard, ControlConstants.kIntakeReverseButton)
    //   .whileTrue(m_intakeSubsystem.getIntakeOnReversedCommand())
    //   .onFalse(m_intakeSubsystem.getIntakeOffCommand());

    // Trigger retractIntakeTrigger = new JoystickButton(m_buttonBoard,
    // ControlConstants.kIntakeRetractButton);
    // retractIntakeTrigger.onTrue(new RetractCommand(m_PneumaticsSubsystem, m_intakeSubsystem));

    // Trigger climbRetractTrigger = new JoystickButton(m_buttonBoard,
    // ControlConstants.kClimbRetractButton);
    // climbRetractTrigger.onTrue(m_PneumaticsSubsystem.getClimbRetractCommand());

    // Trigger climbExtendTrigger = new JoystickButton(m_buttonBoard,
    // ControlConstants.kClimbExtendButton);
    // climbExtendTrigger.onTrue(m_PneumaticsSubsystem.getClimbExtendCommand());

    // m_feedSubsystem.setDefaultCommand(m_feedSubsystem.getDefaultCommand());
    // Trigger feedTrigger = new JoystickButton(m_driverRight, ControlConstants.kFeedButton);
    // feedTrigger.whileTrue(m_feedSubsystem.getFeedCommand());

    // m_limeLightSubsystem.setDefaultCommand(m_limeLightSubsystem.getDefaultCommand());
    // Trigger limeTrigger = new JoystickButton(m_driverLeft, ControlConstants.kAutoAimButton);
    // limeTrigger.whileTrue(new LimelightCommand(m_limeLightSubsystem,m_swerveSubsystem));
  }

  /**
   * Updates the dashboard with current robot status information. Call this method periodically from
   * Robot.robotPeriodic().
   */
  public void updateDashboard() {
    m_gameInfoTable.getEntry("Hub Active").setBoolean(isHubActive());
    m_gameInfoTable.getEntry("Red Hub Active").setBoolean(isRedHubActive());
    m_gameInfoTable.getEntry("Blue Hub Active").setBoolean(isBlueHubActive());
  }

  /**
   * Determines if the scoring hub is currently active based on game rules. The hub activity depends
   * on alliance, game mode, match time, and game data.
   *
   * @return true if the hub is active, false otherwise
   */
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }

    // Use the shared logic for our alliance
    return getHubActiveForAlliance(alliance.get());
  }

  /**
   * Determines if the Red alliance hub is currently active.
   *
   * @return true if Red's hub is active, false otherwise
   */
  public boolean isRedHubActive() {
    return getHubActiveForAlliance(Alliance.Red);
  }

  /**
   * Determines if the Blue alliance hub is currently active.
   *
   * @return true if Blue's hub is active, false otherwise
   */
  public boolean isBlueHubActive() {
    return getHubActiveForAlliance(Alliance.Blue);
  }

  /**
   * Helper method to determine if a specific alliance's hub is active.
   *
   * @param targetAlliance The alliance to check (Red or Blue)
   * @return true if the specified alliance's hub is active, false otherwise
   */
  private boolean getHubActiveForAlliance(Alliance targetAlliance) {
    // Hub is always enabled in autonomous for everyone
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }

    // If we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute based on game data
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    // If we have no game data, assume hub is active
    if (gameData.isEmpty()) {
      return true;
    }

    // Parse game data to determine which alliance has their hub ACTIVE in shift 1
    boolean redHubActiveInShift1 = false;
    boolean blueHubActiveInShift1 = false;

    switch (gameData.charAt(0)) {
      case 'R' -> {
        redHubActiveInShift1 = false; // Red inactive in shift 1
        blueHubActiveInShift1 = true; // Blue active in shift 1
      }
      case 'B' -> {
        redHubActiveInShift1 = true; // Red active in shift 1
        blueHubActiveInShift1 = false; // Blue inactive in shift 1
      }
      default -> {
        // Invalid game data, assume hub is active
        return true;
      }
    }

    // Get the hub status for the target alliance in shift 1
    boolean targetHubActiveInShift1 =
        (targetAlliance == Alliance.Red) ? redHubActiveInShift1 : blueHubActiveInShift1;

    // Determine which shift we're in based on match time
    // Match time counts DOWN from 135 (teleop start) to 0 (match end)
    boolean hubActive;
    if (matchTime > 130) {
      // Transition shift at start of teleop, use shift 1 logic
      hubActive = targetHubActiveInShift1;
    } else if (matchTime > 105) {
      // Shift 1
      hubActive = targetHubActiveInShift1;
    } else if (matchTime > 80) {
      // Shift 2 - opposite of shift 1
      hubActive = !targetHubActiveInShift1;
    } else if (matchTime > 55) {
      // Shift 3 - same as shift 1
      hubActive = targetHubActiveInShift1;
    } else if (matchTime > 30) {
      // Shift 4 - opposite of shift 1
      hubActive = !targetHubActiveInShift1;
    } else {
      // End game, hub always active for everyone
      hubActive = true;
    }
    return hubActive;
  }
} // End Class
