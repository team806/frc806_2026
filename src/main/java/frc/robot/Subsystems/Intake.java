package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final SparkFlex roller;

    private double rollerSpeed;

    @SuppressWarnings("removal")
    public Intake(int armId, int rollerId) {
        roller = new SparkFlex(rollerId, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(30);

        roller.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        rollerSpeed = Preferences.getDouble("rollerSpeed", 0);

        setDefaultCommand(deploy());
        SmartDashboard.putData(this);
        SmartDashboard.putData(saveSettings());
    }

    public Command deploy() {
        // Default command, motion profiled, ideallty feedforward contolled, deploy arm
        // For now we will manually retract
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
        return run(() -> roller.set(rollerSpeed)).withName("Deploy"); 
    }

    public Command bump() {
        // Motion profiled, ideallty feedforward contolled, raise arm a bit, lower arm after raise
        return runOnce(() -> {});
    }

    // We _might_ need to temporarily slow down intake during shooting but that is to be determined later

    public Command saveSettings() {
        return runOnce(() -> {
            Preferences.setDouble("rollerSpeed", rollerSpeed);
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Roller speed", () -> rollerSpeed, (newSpeed) -> rollerSpeed = newSpeed);
    }
}
