package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import frc.robot.Commands.RobotState;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Indexer;
import frc.robot.Constants;

public class Blinkin extends SubsystemBase {
    private final PWMSparkMax blinkin;
    private final double defaultColorValue;

    public Blinkin(int PWMslot, double defaultColorValue, Arm arm, Indexer indexer) {
        this.defaultColorValue = defaultColorValue;
        blinkin = new PWMSparkMax(PWMslot);
        setDefaultCommand(new RobotState(arm, indexer, this));
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
}
