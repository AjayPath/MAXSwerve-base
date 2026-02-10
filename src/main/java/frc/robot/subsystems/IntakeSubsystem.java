package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final VelocityVoltage velocityRequest;
  
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(32);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    intakeMotor.getConfigurator().apply(Configs.intakeMotor.intakeConfig);
  }

  public void start() {
    intakeMotor.setControl(velocityRequest.withVelocity(50));

  }

  public double getSpeed() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  public boolean atTargetSpeed(double targetRPS, double tolerance) {
    return Math.abs(intakeMotor.getVelocity().getValueAsDouble() - targetRPS) < tolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Floor", getSpeed());
  }
}