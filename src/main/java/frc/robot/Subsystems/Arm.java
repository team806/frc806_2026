package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final TalonFX armLeader;
    private final TalonFX armFollower;
    private final CANcoder armEncoder;
    private Constants.Arm.States armTargetState;

    @SuppressWarnings("removal")
    public Arm(int ArmLeaderId, int ArmFollowerId) {
        armLeader = new TalonFX(ArmLeaderId);
        armFollower = new TalonFX(ArmFollowerId);
        armEncoder = new CANcoder(Constants.Arm.ArmEncoderId);

        TalonFXConfiguration armConfig = new TalonFXConfiguration();

        armConfig.Feedback.FeedbackRemoteSensorID = Constants.Arm.ArmEncoderId;
        armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armConfig.Feedback.SensorToMechanismRatio = 1.0;

        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armConfig.CurrentLimits.SupplyCurrentLimit = 40;

        var s0 = armConfig.Slot0;
        s0.GravityType = GravityTypeValue.Arm_Cosine;
        // s0.kG = Constants.Arm.kG;
        // s0.kS = Constants.Arm.kS;
        // s0.kV = Constants.Arm.kV;
        // s0.kA = Constants.Arm.kA;
        s0.kP = Constants.Arm.kP;
        s0.kI = Constants.Arm.kI;
        s0.kD = Constants.Arm.kD;
        var motionMagicConfigs = armConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Arm.MotionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Arm.MotionMagicAcceleration;
        // motionMagicConfigs.MotionMagicJerk = 300;

        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.ArmDeployPos;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.ArmBackPos;
        armConfig.ClosedLoopGeneral.ContinuousWrap = false;
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        armConfig.Feedback.RotorToSensorRatio = Constants.Arm.GearRatio;

        armLeader.getConfigurator().apply(armConfig);

        armFollower.setControl(new Follower(ArmLeaderId, MotorAlignmentValue.Opposed));

        armTargetState = Constants.Arm.States.Deployed;

        SmartDashboard.putData("Arm subsystem", this);

        setDefaultCommand(deploy());
    }


    public Command deploy() {
        return runEnd(() -> {
            final MotionMagicVoltage request = new MotionMagicVoltage(0);
            armLeader.setControl(request.withPosition(Constants.Arm.ArmDeployPos));
            armTargetState = Constants.Arm.States.Deployed;
        }, () -> {}).withName("Deploy");
        // return run(() -> {});
    }

    public Command bump() {
        return runEnd(() -> {
            final MotionMagicVoltage request = new MotionMagicVoltage(0);
            armLeader.setControl(request.withPosition(Constants.Arm.ArmVerticalPos));
            armTargetState = Constants.Arm.States.Vertical;
        }, () -> {}).withName("Bump");
        // return run(() -> {});
    }

    public Command top() {
        return runEnd(() -> {
            final MotionMagicVoltage request = new MotionMagicVoltage(0);
            armLeader.setControl(request.withPosition(Constants.Arm.ArmBackPos));
            armTargetState = Constants.Arm.States.Back;
        }, () -> {}).withName("Top");
        // return run(() -> {});
    }

    public Constants.Arm.States getState() {
        return armTargetState;
    }

    public boolean approximatelyEquals(double a, double b, double tolerance) {
        return Math.abs(a-b) < tolerance;
    }

    public boolean isBetween(double x, double y, double target) {
        return ((x <= target && target <= y) || (y <= target && target <= x));
    }

    public Constants.Arm.States getActualState() {
        double currentArmPos = armEncoder.getAbsolutePosition().getValueAsDouble();
        double tolerance = 0.01;
        if (approximatelyEquals(currentArmPos, Constants.Arm.ArmDeployPos, tolerance)) {
            return Constants.Arm.States.Deployed;
        }
        else if (approximatelyEquals(currentArmPos, Constants.Arm.ArmVerticalPos, tolerance)) {
            return Constants.Arm.States.Vertical;
        }
        else if (approximatelyEquals(currentArmPos, Constants.Arm.ArmBackPos, tolerance)) {
            return Constants.Arm.States.Back;
        }
        else if (isBetween(Constants.Arm.ArmDeployPos, Constants.Arm.ArmVerticalPos, currentArmPos)) {
            return Constants.Arm.States.Deployed_Vertical;
        }
        else if (isBetween(Constants.Arm.ArmVerticalPos, Constants.Arm.ArmBackPos, currentArmPos)) {
            return Constants.Arm.States.Vertical_Back;
        }
        return armTargetState;
    }

    @Override
    public void periodic() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Arm target state", () -> this.getState().name(), null);
        builder.addStringProperty("Arm actual state", () -> this.getActualState().name(), null);
    }
}