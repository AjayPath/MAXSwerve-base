// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-aptag");

  public double getDoubleEntry(String entry) {
    return limelight.getEntry(entry).getDouble(0);
  }

  public double[] getArraryEntry(String entry) {
    return limelight.getEntry(entry).getDoubleArray(new double[6]);
  }

  public boolean hasValidTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1;
  }

  public double getTID() {
    return getDoubleEntry("tid");
  }

  public double getTA() {
    return getDoubleEntry("ta");
  }

  public double getTX() {
    return getDoubleEntry("tx");
  }

  public double getTY() {
    return getDoubleEntry("ty");
  }

  public LimelightSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("HAS TARGET", hasValidTarget());

    if (hasValidTarget()) {
      SmartDashboard.putNumber("tid", getTID());
      SmartDashboard.putNumber("ta", getTA());
      SmartDashboard.putNumber("ty", getTY());
      SmartDashboard.putNumber("tx", getTX());
    }
  }

  
}
