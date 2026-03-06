package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Intake extends SubsystemBase {
    private final SparkFlex roller;
    TalonFX armMotor;

    private double rollerSpeed = 0.5;

    @SuppressWarnings("removal")
    public Intake(int armId, int rollerId) {
        //roller
        roller = new SparkFlex(rollerId, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        roller.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        setDefaultCommand(deploy());

        //Arm
        TalonFX armMotor = new TalonFX(armId);
        var armMotorConfig = new TalonFXConfiguration();
        var slot0 = armMotorConfig.Slot0;

        //PID
        slot0.kP = 12.0;
        slot0.kI = 0.0;
        slot0.kD = 1.5;
        slot0.kS = 0.25;
        slot0.kG = 0.45;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        //gears 5^2
        armMotorConfig.Feedback.SensorToMechanismRatio = 25.0;

        //stops

        double ForwardDegreeLimit = 90.0;
        double BackwardDegreeLimit = 90.0;
        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ForwardDegreeLimit/360.0;
        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -BackwardDegreeLimit/360.0; 
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        armMotor.getConfigurator().apply(armMotorConfig);
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

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
