package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
    private final TalonFX arm_leader;
    private final TalonFX arm_follower;
    private Constants.RobotState.ArmStates armState;

    @SuppressWarnings("removal")
    public Arm(int ArmLeaderId, int ArmFollowerId) {
        arm_leader = new TalonFX(ArmLeaderId);
        arm_follower = new TalonFX(ArmFollowerId);
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
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.ArmBottomPos;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.ArmBackPos;
        armConfig.ClosedLoopGeneral.ContinuousWrap = false;
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        armConfig.Feedback.RotorToSensorRatio = Constants.Arm.GearRatio;

        arm_leader.getConfigurator().apply(armConfig);

        arm_follower.setControl(new Follower(ArmLeaderId, MotorAlignmentValue.Opposed));

        armState = Constants.RobotState.ArmStates.Stowed;

        setDefaultCommand(deploy());
    }


    public Command deploy() {
        return runEnd(() -> {
            final MotionMagicVoltage request = new MotionMagicVoltage(0);
            arm_leader.setControl(request.withPosition(Constants.Arm.ArmBottomPos));
            armState = Constants.RobotState.ArmStates.Deployed;
        }, () -> {}).withName("Deploy");
        // return run(() -> {});
    }

    public Command bump() {
        return runEnd(() -> {
            final MotionMagicVoltage request = new MotionMagicVoltage(0);
            arm_leader.setControl(request.withPosition(Constants.Arm.ArmHorizontalPos));
            armState = Constants.RobotState.ArmStates.Horizontal;
        }, () -> {}).withName("Bump");
        // return run(() -> {});
    }

    public Command top() {
        return runEnd(() -> {
            final MotionMagicVoltage request = new MotionMagicVoltage(0);
            arm_leader.setControl(request.withPosition(Constants.Arm.ArmVerticalPos));
            armState = Constants.RobotState.ArmStates.Stowed;
        }, () -> {}).withName("Top");
        // return run(() -> {});
    }

    public Constants.RobotState.ArmStates getState() {
        return armState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Arm State", armState.name());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}