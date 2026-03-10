package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;


public class Shooter extends SubsystemBase {
    private final SparkFlex shooter;
    private final SparkClosedLoopController closedLoopController;

    private double primeRPM = 0;
    private double shootRPM = 0;
    
    @SuppressWarnings("removal")
    public Shooter(int MotorID) {
        shooter = new SparkFlex(MotorID, MotorType.kBrushless);
        closedLoopController = shooter.getClosedLoopController();
        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop.pid(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);
        config.closedLoop.velocityFF(Constants.Shooter.FF);
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(30);
        config.inverted(true);
        shooter.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        setDefaultCommand(prime());
    }

    // _If_ we need a third state where motor does not turn, turn off the motor
    // when leaving the other two states and have no default command.


    public void setSpeed(double targetRPM) {
        closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
    }

    public Command prime() {
        // Default command, rotate slowly to reduce shooting prep time
        return runEnd(() -> setSpeed(primeRPM), () -> {}).withName("Prime");
    }

    public Command shoot() {
        // Speed up rollers, control velocty
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
        return runEnd(() -> setSpeed(shootRPM), () -> {}).withName("Shoot");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
