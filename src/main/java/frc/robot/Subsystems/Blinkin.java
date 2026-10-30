package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import frc.robot.Commands.LEDStateCommands.ArmIndexerState;
import frc.robot.Commands.LEDStateCommands.ArmState;
import frc.robot.Commands.LEDStateCommands.IndexerState;

import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Indexer;
import frc.robot.Constants;

public class Blinkin extends SubsystemBase {
    private final PWMSparkMax blinkin;
    private final double defaultColorValue;
    private boolean enabled = true;
    private Command defaultCommand;

    public Blinkin(int PWMslot, double defaultColorValue, Arm arm, Indexer indexer) {
        this.defaultColorValue = defaultColorValue;
        blinkin = new PWMSparkMax(PWMslot);

        defaultCommand = new ArmIndexerState(arm, indexer, this);
        setDefaultCommand(defaultCommand);
    }

    public void setDefaultColor() {
        blinkin.set(defaultColorValue);
    }

    public void setColor(double colorValue) {
        blinkin.set(colorValue);
    }

    public void turnOffLEDs() {
        blinkin.set(Constants.Blinkin.SolidColors.BLACK.value);
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
}
