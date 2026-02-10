package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorSubsystem;

public class StartFloor extends Command {
  private final FloorSubsystem floorSubsystem;
   
  public StartFloor(FloorSubsystem floorSubsystem) {
    this.floorSubsystem = floorSubsystem;
    addRequirements(floorSubsystem);
    }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    floorSubsystem.start();
  }

  @Override
  public void end(boolean interrupted) {
    floorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}