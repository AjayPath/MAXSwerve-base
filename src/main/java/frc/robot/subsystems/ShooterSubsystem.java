// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  // initalize motor
  private final TalonFX m_shootingMotor;

  // set control type
  private final VelocityVoltage m_shootingVelocityRequest;

  public ShooterSubsystem() {
    // Change CAN ID FOR TESTING
    m_shootingMotor = new TalonFX(52);

    // Configure motor
    m_shootingMotor.getConfigurator().apply(Configs.shootingMotor.shootingConfig);

    // Setup velocity
    m_shootingVelocityRequest = new VelocityVoltage(0).withSlot(0);

  }

  public void setVelocity(double shooterVelocity) {
    m_shootingMotor.setControl(m_shootingVelocityRequest.withVelocity(shooterVelocity)); // make sure this is double
  }

  public void stop() {
    m_shootingMotor.stopMotor();
  }

  public boolean atTargetSpeed(double targetRPS, double tolerance) {
    return Math.abs(m_shootingMotor.getVelocity().getValueAsDouble() - targetRPS) < tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", m_shootingMotor.getVelocity().getValueAsDouble());
  }
}
