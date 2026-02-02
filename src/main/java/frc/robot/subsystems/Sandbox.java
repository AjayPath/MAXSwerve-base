package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;

public class Sandbox extends SubsystemBase {

    public enum Speed {
        FEED(6000);

        private final double rpm;

        Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    private final TalonFX motorA; // normal
    private final TalonFX motorB; // inverted

    private final VelocityVoltage velocityRequest =
        new VelocityVoltage(0).withSlot(0);

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Sandbox() {
        motorA = new TalonFX(50);
        motorB = new TalonFX(51);

        TalonFXConfiguration normalConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(50))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(1)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / (KrakenX60.kFreeSpeed / 60.0))
            );

        TalonFXConfiguration invertedConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(normalConfig.CurrentLimits)
            .withSlot0(normalConfig.Slot0);

        motorA.getConfigurator().apply(invertedConfig);
        motorB.getConfigurator().apply(normalConfig);
    }

    public void set(Speed speed) {
        motorA.setControl(
            velocityRequest.withVelocity(speed.angularVelocity())
        );

        motorB.setControl(
            velocityRequest.withVelocity(speed.angularVelocity())
        );
    }

    public void setPercentOutput(double percentOutput) {
        motorA.setControl(
            voltageRequest.withOutput(Volts.of(percentOutput * 12.0))
        );

        motorB.setControl(
            voltageRequest.withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public Command feedCommand() {
        return startEnd(
            () -> set(Speed.FEED),
            () -> setPercentOutput(0)
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Motor A RPM",
            () -> motorA.getVelocity().getValue().in(RPM),
            null
        );
        builder.addDoubleProperty(
            "Motor B RPM",
            () -> motorB.getVelocity().getValue().in(RPM),
            null
        );
    }
}
