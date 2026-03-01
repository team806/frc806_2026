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

public class Indexer extends SubsystemBase {
    private final SparkFlex bottomRoller;
    private final SparkFlex topRoller;

    private double floorIdleSpeed;
    private double floorIndexSpeed;
    private double ceilingIndexSpeed;

    @SuppressWarnings("removal")
    public Indexer(int bottomRollerID, int topRollerID) {
        bottomRoller = new SparkFlex(bottomRollerID, MotorType.kBrushless);
        topRoller = new SparkFlex(topRollerID, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        config.inverted(true);

        bottomRoller.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);
        topRoller.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        floorIdleSpeed = Preferences.getDouble("floorIdleSpeed", 0);
        floorIndexSpeed = Preferences.getDouble("floorIndexSpeed", 0);
        ceilingIndexSpeed = Preferences.getDouble("ceilingIndexSpeed", 0);

        setDefaultCommand(rollFloor());
        SmartDashboard.putData(this);
        SmartDashboard.putData(saveSettings());
    }

    public Command rollFloor() {
        return runEnd(() -> {
            bottomRoller.set(floorIdleSpeed);
        }, () -> {}).withName("Roll floor");
    }

    public Command index() {
        return runEnd(() -> {
            bottomRoller.set(floorIndexSpeed);
            topRoller.set(ceilingIndexSpeed);
        }, () -> {
            topRoller.set(0);
        }).withName("Start indexing");
    }

    public Command saveSettings() {
        return runOnce(() -> {
            Preferences.setDouble("floorIdleSpeed", floorIdleSpeed);
            Preferences.setDouble("floorIndexSpeed", floorIndexSpeed);
            Preferences.setDouble("ceilingIndexSpeed", ceilingIndexSpeed);
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Floor idle speed", () -> floorIdleSpeed, (newSpeed) -> floorIdleSpeed = newSpeed);
        builder.addDoubleProperty("Floor index speed", () -> floorIndexSpeed, (newSpeed) -> floorIndexSpeed = newSpeed);
        builder.addDoubleProperty("Ceiling index speed", () -> ceilingIndexSpeed, (newSpeed) -> ceilingIndexSpeed = newSpeed);
    }
}
