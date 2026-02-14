package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{

    //drive 
    TalonFX driveMotor;
    int driveMotorID;
    //steer
    TalonFX steerMotor;
    PIDController steerController;
    //module encoder 
    CANcoder moduleEncoder;
    double encoderOffsetRotations;
    //conversion factors
    final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    final double GEAR_RATIO = 1.0 / 6.75;
    final double DRIVE_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
    final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.0;
    final double STEER_POSITION_CONVERSION = 1;
    final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / 60.0;

    //CONSTRUCTOR//
    public SwerveModule(int driveMotorID, int steerMotorID, int encoderID, Double encoderOffsetRotations){
        this.driveMotorID = driveMotorID;

        //drive motor 
        driveMotor = new TalonFX(driveMotorID);
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        driveMotor.getConfigurator().apply(config);

        //steer motor
        steerMotor = new TalonFX(steerMotorID);
        var steerConfig = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        steerMotor.getConfigurator().apply(config);
        
        // module encoder
        moduleEncoder = new CANcoder(encoderID, new CANBus("*"));
        this.encoderOffsetRotations = encoderOffsetRotations;

        //controllers
        //driveController = driveMotor.getPIDController();
        //driveController.setP(Constants.Modules.SpeedKP);
        //driveController.setI(Constants.Modules.SpeedKI);
        //driveController.setD(Constants.Modules.SpeedKD);

        steerController = new PIDController(Constants.Modules.SteerKP, Constants.Modules.SteerKI, Constants.Modules.SteerKD);
        steerController.enableContinuousInput(0, 1);

    }

    //DRIVE//
    public void setTargetState(SwerveModuleState targetState, boolean isCosineCompensated) {
        //PID experement
        //steerMotor.set(-steerController.calculate(getModuleAngRotations(),targetState.angle.getRotations()));
        //driveController.setReference(targetState.speedMetersPerSecond / DRIVE_VELOCITY_CONVERSION, ControlType.kVelocity);

        // FUNCTIONING
        double currentAngle = getModuleAngRotations();
        steerMotor.set(-steerController.calculate(currentAngle, targetState.angle.getRotations()));
        if (isCosineCompensated) {
            targetState.speedMetersPerSecond *= targetState.angle.minus(new Rotation2d(currentAngle*2*Math.PI)).getCos();
        }
        driveMotor.set(targetState.speedMetersPerSecond/Constants.attainableMaxModuleSpeedMPS); 
    }
    
    //FEEDBACK//
    public void periodic() {
        SmartDashboard.putNumber("S" + driveMotorID, getModuleAngRotations());
    }

    public double getModuleAngRotations(){
        return moduleEncoder.getAbsolutePosition().getValueAsDouble() - encoderOffsetRotations;
    }
    
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveMotor.getPosition(true).getValueAsDouble(),
            Rotation2d.fromRotations(getModuleAngRotations())
        );  
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(
            driveMotor.getVelocity(true).getValueAsDouble(),
            Rotation2d.fromRotations(getModuleAngRotations()));
    }

}
