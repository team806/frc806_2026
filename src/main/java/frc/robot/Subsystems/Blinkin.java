package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import frc.robot.Constants;

public class Blinkin extends SubsystemBase {
    PWMSparkMax blinkin;
    private final Constants.Blinkin.SolidColors DefaultColor;

    public Blinkin(int PWMslot, Constants.Blinkin.SolidColors DefaultColor) {
        this.DefaultColor = DefaultColor;
        blinkin = new PWMSparkMax(PWMslot);
    }

    public Command setDefaultColor() {
        return runOnce(() -> blinkin.set(DefaultColor.value));
    }

    public Command setColor(Constants.Blinkin.SolidColors color) {
        return runOnce(() -> blinkin.set(color.value));
    }

    public Command turnOffLEDs() {
        return runOnce(() -> blinkin.set(Constants.Blinkin.SolidColors.BLACK.value));
    }
}
