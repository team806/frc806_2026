package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import frc.robot.Constants;

public class Blinkin extends SubsystemBase {
    private final PWMSparkMax blinkin;
    private final double defaultColorValue;

    public Blinkin(int PWMslot, double defaultColorValue) {
        this.defaultColorValue = defaultColorValue;
        blinkin = new PWMSparkMax(PWMslot);
        setDefaultCommand(setDefaultColor());
    }

    public Command setDefaultColor() {
        return run(() -> blinkin.set(defaultColorValue));
    }

    public Command setColor(double colorValue) {
        return run(() -> blinkin.set(colorValue));
    }

    public Command turnOffLEDs() {
        return run(() -> blinkin.set(Constants.Blinkin.SolidColors.BLACK.value));
    }
}
