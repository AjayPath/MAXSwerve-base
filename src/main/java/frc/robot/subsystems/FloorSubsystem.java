package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class FloorSubsystem extends SubsystemBase {
  private final TalonFX floorMotor;
  private final VelocityVoltage velocityRequest;
  
  public FloorSubsystem() {
    floorMotor = new TalonFX(32);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    floorMotor.getConfigurator().apply(Configs.floorMotor.floorConfig);
  }

  public void start() {
    floorMotor.setControl(velocityRequest.withVelocity(22));

  }

  public double getSpeed() {
    return floorMotor.getVelocity().getValueAsDouble();
  }

  public void stop() {
    floorMotor.stopMotor();
  }

  public boolean atTargetSpeed(double targetRPS, double tolerance) {
    return Math.abs(floorMotor.getVelocity().getValueAsDouble() - targetRPS) < tolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Floor", getSpeed());
  }
}