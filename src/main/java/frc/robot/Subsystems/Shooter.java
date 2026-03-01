package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Shooter extends SubsystemBase {
    private final SparkFlex shooter;

    private double primeSpeed;
    private double shootSpeed;
    
    @SuppressWarnings("removal")
    public Shooter(int MotorID) {
        shooter = new SparkFlex(MotorID, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(30);

        shooter.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        primeSpeed = Preferences.getDouble("primeSpeed", 0);
        shootSpeed = Preferences.getDouble("shootSpeed", 0);

        setDefaultCommand(prime());
        SmartDashboard.putData(this);
        SmartDashboard.putData(saveSettings());
    }

    // _If_ we need a third state where motor does not turn, turn off the motor
    // when leaving the other two states and have no default command.

    public Command prime() {
        // Default command, rotate slowly to reduce shooting prep time
        return runEnd(() -> shooter.set(primeSpeed), () -> {}).withName("Prime");
    }

    public Command shoot() {
        // Speed up rollers, control velocty
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
        return runEnd(() -> shooter.set(shootSpeed), () -> {}).withName("Shoot");
    }

    public Command saveSettings() {
        return runOnce(() -> {
            Preferences.setDouble("primeSpeed", primeSpeed);
            Preferences.setDouble("shootSpeed", shootSpeed);
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Prime speed", () -> primeSpeed, (newSpeed) -> primeSpeed = newSpeed);
        builder.addDoubleProperty("Shoot speed", () -> shootSpeed, (newSpeed) -> shootSpeed = newSpeed);
    }
}
