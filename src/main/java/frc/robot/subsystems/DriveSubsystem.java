package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.APOdometry;
import frc.robot.utils.Pose;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGryoID);

  // Custom odometry class for tracking robot pose
  private final APOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    
    // Initialize custom odometry with modules in FL, FR, BL, BR order
    List<MAXSwerveModule> modules = List.of(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight);
    m_odometry = APOdometry.getInstance(modules, m_gyro);
  }

  @Override
  public void periodic() {
    // Update the custom odometry in the periodic block
    m_odometry.update();
    
    // Publish odometry data to SmartDashboard
    //m_odometry.publishToSmartDashboard();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose with normalized angle [0, 360)
   */
  public Pose getPose() {
    return m_odometry.getPose();
  }

  /**
   * Returns the currently-estimated pose of the robot with continuous angle.
   *
   * @return The pose with continuous angle (can exceed 360°)
   */
  public Pose getPoseContinuous() {
    return m_odometry.getPoseContinuous();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose pose) {
    m_odometry.setPose(pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, 
                ySpeedDelivered, 
                rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble() * 
           (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Resets the gyro to zero after waiting for initialization.
   */
  public void resetGyroToZero() {
    // Wait for gyro to initialize (max 2 seconds)
    Timer timer = new Timer();
    timer.start();
    
    while (Double.isNaN(m_gyro.getRotation2d().getDegrees()) && timer.get() < 2.0) {
      Timer.delay(0.01);
    }
    
    // Reset gyro yaw to zero
    m_gyro.setYaw(0);
  }
  
  /**
   * Resets the odometry to origin with current heading.
   */
  public void resetOdometry() {
    m_odometry.reset();
  }

  public Pose getCustomPose() {
    return m_odometry.getPose();
  }

  public void setOdom(double x, double y, double angle) {
      m_odometry.setPose(new Pose(x, y, angle));
  }
}


// package frc.robot.subsystems;

// import java.util.List;

// import com.ctre.phoenix6.hardware.Pigeon2;

// import edu.wpi.first.hal.FRCNetComm.tInstances;
// import edu.wpi.first.hal.FRCNetComm.tResourceType;
// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.DriveConstants; // you still have this in your project
// import frc.robot.utils.APOdometry;
// import frc.robot.utils.Pose;
// import frc.robot.utils.SlewRateLimiter;

// public class DriveSubsystem extends SubsystemBase {

//   // -------------------- TUNABLE VALUES (put them here) --------------------
//   // These are applied to *normalized* inputs (-1..1) because we slew before scaling.
//   // Start here; tune on carpet.
//   private static final double SLEW_RATE_LIMIT_X = 3.0;   // joystick units/sec
//   private static final double SLEW_JERK_LIMIT_X = 12.0;  // joystick units/sec^2
//   private static final double SLEW_RATE_LIMIT_Y = 3.0;
//   private static final double SLEW_JERK_LIMIT_Y = 12.0;
//   private static final double SLEW_RATE_LIMIT_R = 4.0;
//   private static final double SLEW_JERK_LIMIT_R = 18.0;

//   // Hold-heading behavior (matches OPR: ~350ms)
//   private static final double HOLD_HEADING_DELAY_SEC = 0.350;

//   // Simple P controller for heading hold (OPR uses PID; start with P only)
//   // This is output in normalized rot (-1..1).
//   private static final double HOLD_HEADING_kP = 0.015; // tune: increase until it holds without oscillating
//   private static final double HOLD_HEADING_MAX_NORM = 0.6; // clamp hold assist

//   // Deadbands (so "0" really means 0)
//   private static final double DEADBAND_XY = 0.03;
//   private static final double DEADBAND_R  = 0.03;

//   // -------------------- MODULES / SENSORS --------------------
//   private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
//       DriveConstants.kFrontLeftDrivingCanId,
//       DriveConstants.kFrontLeftTurningCanId,
//       DriveConstants.kFrontLeftChassisAngularOffset);

//   private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
//       DriveConstants.kFrontRightDrivingCanId,
//       DriveConstants.kFrontRightTurningCanId,
//       DriveConstants.kFrontRightChassisAngularOffset);

//   private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
//       DriveConstants.kRearLeftDrivingCanId,
//       DriveConstants.kRearLeftTurningCanId,
//       DriveConstants.kBackLeftChassisAngularOffset);

//   private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
//       DriveConstants.kRearRightDrivingCanId,
//       DriveConstants.kRearRightTurningCanId,
//       DriveConstants.kBackRightChassisAngularOffset);

//   private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGryoID);
//   private final APOdometry m_odometry;

//   // -------------------- OPR-LIKE STATE MACHINE --------------------
//   public enum DriveState { IDLE, FIELD_DRIVE, ROBOT_DRIVE }
//   private DriveState m_state = DriveState.FIELD_DRIVE;

//   // Slew limiters (your class)
//   private final SlewRateLimiter xLimiter = new SlewRateLimiter(SLEW_RATE_LIMIT_X, SLEW_JERK_LIMIT_X);
//   private final SlewRateLimiter yLimiter = new SlewRateLimiter(SLEW_RATE_LIMIT_Y, SLEW_JERK_LIMIT_Y);
//   private final SlewRateLimiter rLimiter = new SlewRateLimiter(SLEW_RATE_LIMIT_R, SLEW_JERK_LIMIT_R);

//   // Hold heading bookkeeping
//   private final Timer holdHeadingTimer = new Timer();
//   private double oldHeadingDeg = 0.0;

//   // Latest joystick command (set from your command)
//   private double cmdX = 0.0;
//   private double cmdY = 0.0;
//   private double cmdR = 0.0;

//   public DriveSubsystem() {
//     HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

//     List<MAXSwerveModule> modules = List.of(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight);
//     m_odometry = APOdometry.getInstance(modules, m_gyro);

//     holdHeadingTimer.start();
//     oldHeadingDeg = getHeading();
//   }

//   @Override
//   public void periodic() {
//     m_odometry.update();
//     runStateMachine();
//   }

//   // -------------------- PUBLIC API (like OPR calls) --------------------

//   /** Call this from your default drive command every loop with joystick inputs (-1..1). */
//   public void setTeleopCommand(double xSpeed, double ySpeed, double rot) {
//     cmdX = applyDeadband(xSpeed, DEADBAND_XY);
//     cmdY = applyDeadband(ySpeed, DEADBAND_XY);
//     cmdR = applyDeadband(rot,    DEADBAND_R);
//   }

//   public void setState(DriveState state) {
//     if (m_state != state) {
//       m_state = state;
//       // match OPR: refresh heading bookkeeping when changing modes
//       oldHeadingDeg = getHeading();
//       holdHeadingTimer.reset();
//       // optional: reset slew so mode switch doesn't "snap"
//       xLimiter.ResetSlewRate(cmdX);
//       yLimiter.ResetSlewRate(cmdY);
//       rLimiter.ResetSlewRate(cmdR);
//     }
//   }

//   /** Call this at TeleopInit (or when enabling) to match OPR behavior. */
//   public void teleopInitLikeOPR() {
//     oldHeadingDeg = getHeading();
//     holdHeadingTimer.reset();
//     m_state = DriveState.FIELD_DRIVE;

//     // Start slew from 0 so it doesn’t jump
//     xLimiter.ResetSlewRate(0);
//     yLimiter.ResetSlewRate(0);
//     rLimiter.ResetSlewRate(0);
//   }

//   // -------------------- OPR-LIKE LOGIC --------------------

//   private void runStateMachine() {
//     // OPR does: if no input -> IDLE, else drive mode
//     boolean hasInput = (cmdX != 0.0) || (cmdY != 0.0) || (cmdR != 0.0);

//     switch (m_state) {
//       case IDLE:
//         if (hasInput) {
//           m_state = DriveState.FIELD_DRIVE;
//         }
//         oldHeadingDeg = getHeading();
//         holdHeadingTimer.reset();
//         // optionally: stop modules or keep last angles
//         driveInternal(0, 0, 0, true);
//         break;

//       case FIELD_DRIVE: {
//         double x = cmdX;
//         double y = cmdY;
//         double r = cmdR;

//         // --- Slew limit like OPR (BEFORE field transform) ---
//         x = xLimiter.CalculateSlewRate(x);
//         y = yLimiter.CalculateSlewRate(y);
//         r = rLimiter.CalculateSlewRate(r);

//         // --- Hold heading like OPR when rVel == 0 for 350ms ---
//         if (Math.abs(r) < 1e-9 && holdHeadingTimer.get() > HOLD_HEADING_DELAY_SEC) {
//           // TurnPID to oldHeading; here: simple P on angle error
//           double errDeg = angleErrorDeg(oldHeadingDeg, getHeading());
//           double assist = clamp(errDeg * HOLD_HEADING_kP, -HOLD_HEADING_MAX_NORM, HOLD_HEADING_MAX_NORM);
//           r = assist;
//         } else if (Math.abs(r) > 1e-9) {
//           holdHeadingTimer.reset();
//         } else {
//           oldHeadingDeg = getHeading();
//         }

//         driveInternal(x, y, r, true);
//         break;
//       }

//       case ROBOT_DRIVE: {
//         double x = xLimiter.CalculateSlewRate(cmdX);
//         double y = yLimiter.CalculateSlewRate(cmdY);
//         double r = rLimiter.CalculateSlewRate(cmdR);

//         driveInternal(x, y, r, false);

//         oldHeadingDeg = getHeading();
//         holdHeadingTimer.reset();
//         break;
//       }
//     }
//   }

//   /**
//    * Your existing drive(), but called internally with normalized inputs.
//    */
//   private void driveInternal(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
//     double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
//     double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
//     double rotDelivered    = rot    * DriveConstants.kMaxAngularSpeed;

//     SwerveModuleState[] states =
//         DriveConstants.kDriveKinematics.toSwerveModuleStates(
//             fieldRelative
//                 ? ChassisSpeeds.fromFieldRelativeSpeeds(
//                     xSpeedDelivered, ySpeedDelivered, rotDelivered,
//                     Rotation2d.fromDegrees(getHeading()))
//                 : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

//     SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

//     m_frontLeft.setDesiredState(states[0]);
//     m_frontRight.setDesiredState(states[1]);
//     m_rearLeft.setDesiredState(states[2]);
//     m_rearRight.setDesiredState(states[3]);
//   }

//   // -------------------- UTIL --------------------

//   private static double applyDeadband(double val, double db) {
//     if (Math.abs(val) < db) return 0.0;
//     return val;
//   }

//   private static double clamp(double v, double lo, double hi) {
//     return Math.max(lo, Math.min(hi, v));
//   }

//   /** shortest signed error (deg) from current -> target, like PID continuous input */
//   private static double angleErrorDeg(double targetDeg, double currentDeg) {
//     double err = targetDeg - currentDeg;
//     while (err > 180.0) err -= 360.0;
//     while (err < -180.0) err += 360.0;
//     return err;
//   }

//   // -------------------- YOUR EXISTING METHODS --------------------

//   public double getHeading() {
//     return Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees()).getDegrees();
//   }

//   public Pose getPose() { return m_odometry.getPose(); }
//   public Pose getPoseContinuous() { return m_odometry.getPoseContinuous(); }
//   public void resetOdometry(Pose pose) { m_odometry.setPose(pose); }
//   public void resetOdometry() { m_odometry.reset(); }

//   public void resetGyroToZero() {
//     Timer timer = new Timer();
//     timer.start();
//     while (Double.isNaN(m_gyro.getRotation2d().getDegrees()) && timer.get() < 2.0) {
//       Timer.delay(0.01);
//     }
//     m_gyro.setYaw(0);
//   }
// }
