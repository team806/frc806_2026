package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private final SparkFlex topRoller;
    private Constants.RobotState.IndexerStates indexerState;

    @SuppressWarnings("removal")
    public Indexer(int topRollerID) {
        topRoller = new SparkFlex(topRollerID, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        config.inverted(true);

        topRoller.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        indexerState = Constants.RobotState.IndexerStates.Backwards;

        setDefaultCommand(idleIndex());
    }

    public Command idleIndex() {
        return runEnd(() -> {
            topRoller.setVoltage(-Constants.Indexer.ceilingIdleVoltage);
            indexerState = Constants.RobotState.IndexerStates.Backwards;
        }, () -> {}).withName("Idle index");
        // return run(() -> {});
    }

    public Command index() {
        return runEnd(() -> {
            topRoller.setVoltage(Constants.Indexer.ceilingIndexVoltage);
            indexerState = Constants.RobotState.IndexerStates.Forwards;
        }, () -> {}).withName("Index");
        // return run(() -> {});
    }

    public Command stop() {
        return runEnd(() -> {
            topRoller.setVoltage(0);
            indexerState = Constants.RobotState.IndexerStates.Stopped;
        }, () -> {}).withName("Stop indexer");
        // return run(() -> {});
    }

    public Constants.RobotState.IndexerStates getState() {
        return indexerState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Indexer State", indexerState.name());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
