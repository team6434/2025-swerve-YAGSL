package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SlowDriveToggle extends Command
{

  private final SwerveSubsystem swerveSubsystem;

  public SlowDriveToggle(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    swerveSubsystem.slowDriveDB = !swerveSubsystem.slowDriveDB;
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }

  @Override
  public void end(boolean interrupted)
  {
  }
}
