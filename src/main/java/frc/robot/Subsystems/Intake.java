package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkFlex roller;
    private boolean enabled = true;
    private Command defaultCommand = intake();

    @SuppressWarnings("removal")
    public Intake(int rollerId) {
        roller = new SparkFlex(rollerId, MotorType.kBrushless);
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.inverted(true);
        rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(30);

        roller.configure(rollerConfig, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);
        setDefaultCommand(defaultCommand);
    }


    public Command intake() {
        return runEnd(() -> {
            roller.setVoltage(Constants.Intake.rollerVoltage);
        }, () -> {}).withName("Intake");
        // return run(() -> {});
    }

    public Command discharge() {
        return runEnd(() -> {
            roller.setVoltage(-Constants.Intake.rollerVoltage);
        }, () -> {}).withName("Intake");
        // return run(() -> {});
    }

    public Command stop() {
        return runEnd(() -> {
            roller.setVoltage(0);
        }, () -> {}).withName("Stop intake");
        // return run(() -> {});
    }

    public Command doNothing() {
        return Commands.idle().withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public boolean getIsEnabled() {
        return enabled;
    }

    public void enable() {
        setDefaultCommand(defaultCommand);
        enabled = true;
    }

    public void disable() {
        CommandScheduler.getInstance().requiring(this).cancel();
        setDefaultCommand(doNothing());
        enabled = false;
    }

    public void toggle(boolean targetState) {
        if (targetState) {
            enable();
        }
        else {
            disable();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}