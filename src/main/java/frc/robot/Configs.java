package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            drivingConfig.CurrentLimits.SupplyCurrentLimit = 50;
            drivingConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            drivingConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            drivingConfig.Feedback.SensorToMechanismRatio = ModuleConstants.kDrivingMotorReduction;

            drivingConfig.Slot0.kP = 0.1;
            drivingConfig.Slot0.kI = 0.0;
            drivingConfig.Slot0.kD = 0.0;
            drivingConfig.Slot0.kV = drivingVelocityFeedForward;

            drivingConfig.Voltage.PeakForwardVoltage = 12.0;
            drivingConfig.Voltage.PeakReverseVoltage = -12.0;

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0) // radians per second
                    // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
}
