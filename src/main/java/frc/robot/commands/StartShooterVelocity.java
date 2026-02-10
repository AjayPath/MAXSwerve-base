package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooterVelocity extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final double velocityRPS;

  public StartShooterVelocity(ShooterSubsystem shooterSubsystem, double velocityRPS) {
    this.shooterSubsystem = shooterSubsystem;
    this.velocityRPS = velocityRPS;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.setVelocity(velocityRPS);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until button released
  }
}
