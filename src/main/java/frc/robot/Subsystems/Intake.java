package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkFlex roller;
    private final TalonFX arm;
    // private final MotionMagicExpoVoltage armRequest;

    private double rollerSpeed = 12;

    @SuppressWarnings("removal")
    public Intake(int armId, int rollerId) {
        roller = new SparkFlex(rollerId, MotorType.kBrushless);
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.inverted(true);
        rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(30);

        roller.configure(rollerConfig, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);


        arm = new TalonFX(armId);
        TalonFXConfiguration armConfig = new TalonFXConfiguration();

        armConfig.Feedback.FeedbackRemoteSensorID = Constants.Intake.ArmEncoderId;
        armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armConfig.Feedback.SensorToMechanismRatio = 1.0;

        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armConfig.CurrentLimits.SupplyCurrentLimit = 40;

        var s0 = armConfig.Slot0;
        s0.GravityType = GravityTypeValue.Arm_Cosine;
        // s0.kG = Constants.Intake.kG;
        // s0.kS = Constants.Intake.kS;
        // s0.kV = Constants.Intake.kV;
        // s0.kA = Constants.Intake.kA;
        s0.kP = Constants.Intake.kP;
        s0.kI = Constants.Intake.kI;
        s0.kD = Constants.Intake.kD;
        var motionMagicConfigs = armConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Intake.MotionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Intake.MotionMagicAcceleration;
        // motionMagicConfigs.MotionMagicJerk = 300;
        // motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Intake.MotionMagicCruiseVelocity;
        // motionMagicConfigs.MotionMagicExpo_kV = Constants.Intake.kV;
        // motionMagicConfigs.MotionMagicExpo_kA = Constants.Intake.kA;

        // double ForwardDegreeLimit = 30.0;
        // double BackwardDegreeLimit = 30.0;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.08;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.28;
        armConfig.ClosedLoopGeneral.ContinuousWrap = false;
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        arm.getConfigurator().apply(armConfig);
        // arm.getConfigurator().apply(s0);

        // armRequest = new MotionMagicExpoVoltage(0);

        setDefaultCommand(deploy());
    }

    // public void setArmPos(double rotations) {
    //     double addOn = 0;
    //     if (arm.getPosition() < )
    //     final PositionVoltage request = new PositionVoltage(0);
    //     arm.setControl(request.withPosition(rotations).withFeedForward(addOn));
    // }

    // public double smartkS(double target) {
    //     double current = arm.getPosition().getValueAsDouble();
    //     if (target-current > 0) {
    //         return Constants.Intake.kS;
    //     } 
    //     else {
    //         return 0;
    //     } 
    // }

    public Command deploy() {
        // Default command, motion profiled, ideallty feedforward contolled, deploy arm
        // For now we will manually retract
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
        // return run(() -> roller.set(rollerSpeed)).withName("Deploy"); 
        // return runEnd(() -> {
        //     final MotionMagicVoltage request = new MotionMagicVoltage(0);
        //     arm.setControl(request.withPosition(0.08));
        //     roller.setVoltage(rollerSpeed);
        // }, () -> {});
        return run(() -> {});
    }

    public Command bump() {
        // Motion profiled, ideallty feedforward contolled, raise arm a bit, lower arm after raise
        return runEnd(() -> {
            final MotionMagicVoltage request = new MotionMagicVoltage(0);
            arm.setControl(request.withPosition(0));
            roller.setVoltage(rollerSpeed);
        }, () -> {});
        // return run(() -> {});
    }

    public Command top() {
        return runEnd(() -> {
            final MotionMagicVoltage request = new MotionMagicVoltage(0);
            arm.setControl(request.withPosition(-0.25));
            roller.setVoltage(rollerSpeed);
        }, () -> {});
    }

    // We _might_ need to temporarily slow down intake during shooting but that is to be determined later

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}