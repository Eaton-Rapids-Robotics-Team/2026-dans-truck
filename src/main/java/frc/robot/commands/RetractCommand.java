package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class RetractCommand extends SequentialCommandGroup {
  // TODO: Bug if intake off hit at any point, the intake part of this will stop working.

  /*
   * 1.   Constructor - Might have parameters for this command such as target positions of devices. Should also set the name of the command for debugging purposes.
   *  This will be used if the status is viewed in the dashboard. And the command should require (reserve) any devices is might use.
   */
  public RetractCommand(PneumaticsSubsystem pneumaticsSubsystem, IntakeSubsystem intakeSubsystem) {
    Command turnOnIntake = intakeSubsystem.getIntakeOnCommand();
    Command turnOffIntake = intakeSubsystem.getIntakeOffCommand();
    Command retractIntake = pneumaticsSubsystem.getIntakeRetractCommand().withTimeout(0.25);
    addCommands(
        turnOnIntake, new WaitCommand(.5), retractIntake, new WaitCommand(1), turnOffIntake);
  }
}
