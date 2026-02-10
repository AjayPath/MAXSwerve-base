package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class FeederSubsystem extends SubsystemBase {
  private final TalonFX feederMotor;
  private final VelocityVoltage velocityRequest;
  
  public FeederSubsystem() {
    feederMotor = new TalonFX(31);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    feederMotor.getConfigurator().apply(Configs.feederMotor.feederConfig);
  }

  public void start() {
    feederMotor.setControl(velocityRequest.withVelocity(22));
  }

  public double getSpeed() {
    return feederMotor.getVelocity().getValueAsDouble();
  }

  public void stop() {
    feederMotor.stopMotor();
  }

  public boolean atTargetSpeed(double targetRPS, double tolerance) {
    return Math.abs(feederMotor.getVelocity().getValueAsDouble() - targetRPS) < tolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder", getSpeed());
  }
}