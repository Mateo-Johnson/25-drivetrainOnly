package frc.robot.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToAngle extends Command {
  /** Creates a new TurnAngle. */
  private final int targetAngle;
  private final Drivetrain drivetrain = new Drivetrain();
  double currentAngle;

  double kP = 0.00000001;
  double kI = 0;
  double kD = 0;
  PIDController turnPID = new PIDController(kP, kI, kD);

  public TurnToAngle(int targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = drivetrain.getHeading();
    drivetrain.drive(0, 0, turnPID.calculate(currentAngle, targetAngle), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
