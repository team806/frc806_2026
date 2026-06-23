package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Commands.DriveFieldRelative;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class Drivetrain extends SubsystemBase {
    Pigeon2 IMU;
    public SwerveModule[] modules;
    SwerveDriveKinematics kinematics;
    double translationMaxAccelerationMetersPerSecondSquared = 25;
    double rotationMaxAccelerationRadiansPerSecondSquared = 50;
    SlewRateLimiter translationXLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
    SlewRateLimiter translationYLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(rotationMaxAccelerationRadiansPerSecondSquared);

    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(Constants.Drivetrain.FieldLayout, Constants.Drivetrain.RobotToCamera);
    private final SwerveDrivePoseEstimator poseEstimator;
    private boolean hasRecievedVisionMeasurement;
    private final Field2d field = new Field2d();
    
    public Drivetrain(SwerveModule[] modules, CommandXboxController controller, String cameraName) {
        IMU = new Pigeon2(Constants.PigeonID, new CANBus("*"));
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(Constants.moduleLocations);
        setDefaultCommand(new DriveFieldRelative(this, controller));

        camera = new PhotonCamera(cameraName);
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroscopeRotation(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(Constants.Drivetrain.Odometry.PositionStdDev, Constants.Drivetrain.Odometry.PositionStdDev, Constants.Drivetrain.Odometry.AngleStdDev),
            // We calculate these upon update provide those real details 
            VecBuilder.fill(999999, 999999, 999999)
        );

        hasRecievedVisionMeasurement = false;

        SmartDashboard.putData("Field", field);
        SmartDashboard.putData(calibrate());
        SmartDashboard.putData(this);

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChasisSpeed,
            (speeds, feedforwards) -> drive(speeds),

            new PPHolonomicDriveController(
                new PIDConstants(Constants.Drivetrain.SpeedKP, Constants.Drivetrain.SpeedKI, Constants.Drivetrain.SpeedKD),
                new PIDConstants(Constants.Drivetrain.SteerKP, Constants.Drivetrain.SteerKI, Constants.Drivetrain.SteerKD)
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (!alliance.isPresent()) {
                return false;
              }
              return alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );
    }

    @Override
    public void periodic() {
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            var visionEstimate = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEstimate.isEmpty()) {
                visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
            }

            visionEstimate.ifPresent(e -> {
                var targets = e.targetsUsed;
                Pose2d estPose = e.estimatedPose.toPose2d();
                
                if ((targets.size() == 1 && targets.get(0).getPoseAmbiguity() > 0.2) ||
                        targets.size() == 0) {
                    return;
                }

                if (Math.abs(e.estimatedPose.getZ()) > 0.5) {
                    return;
                }

                if (estPose.getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation()) > 0.5 && 
                    hasRecievedVisionMeasurement) {
                    return;
                }

                if (!hasRecievedVisionMeasurement) {
                    poseEstimator.resetPose(estPose);
                    hasRecievedVisionMeasurement = true;
                    return;
                }

                Matrix<N3, N1> stdDevs = getStdDevs(e, targets);
                poseEstimator.addVisionMeasurement(estPose, e.timestampSeconds, stdDevs);

                double lag = Timer.getFPGATimestamp() - e.timestampSeconds;
                SmartDashboard.putNumber("Vision/MeasurementLag_s", lag);
            });            
        }

        poseEstimator.update(getGyroscopeRotation(), getModulePositions());

        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    private Matrix<N3, N1> getStdDevs(EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
        var estStdDevs = Constants.Drivetrain.SingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var target : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            avgDist += tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
        }

        if (numTags > 0) {
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) {
                estStdDevs = Constants.Drivetrain.MultiTagStdDevs;
            } 
            // Increase std devs based on (average) distance
            if (numTags == 1 && avgDist > 4) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            }
        }
        
        return estStdDevs;
    }

	public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(IMU.getYaw().getValueAsDouble());
    }

    public void resetGyro() {
        IMU.reset();
        poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), getPose());
    }

    private Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    private void resetPose(Pose2d pose) {
        if (!hasRecievedVisionMeasurement) {
            poseEstimator.resetPose(pose);
        }
    }
    
    public Command calibrate() {
        return parallel(
            runOnce(() -> {System.out.println("Swerve calibration triggered");}),
            modules[0].calibrate(),
            modules[1].calibrate(),
            modules[2].calibrate(),
            modules[3].calibrate()
        ).ignoringDisable(true).withName("Calibrating");
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        setModuleTargetStates(chassisSpeeds, new Translation2d());
    }

    public void drive(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
        setModuleTargetStates(chassisSpeeds, centerOfRotation);
    }

    public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds.vxMetersPerSecond = translationXLimiter.calculate(chassisSpeeds.vxMetersPerSecond);
        chassisSpeeds.vyMetersPerSecond = translationYLimiter.calculate(chassisSpeeds.vyMetersPerSecond);
        chassisSpeeds.omegaRadiansPerSecond = rotationLimiter.calculate(chassisSpeeds.omegaRadiansPerSecond);

        setModuleTargetStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroscopeRotation()));
    }

    public void setModuleTargetStates(ChassisSpeeds chassisSpeeds) {
        setModuleTargetStates(chassisSpeeds, new Translation2d());
    }

    public void setModuleTargetStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.attainableMaxModuleSpeedMPS);
        for (int i = 0; i < modules.length; ++i) {
            modules[i].setTargetState(targetStates[i]);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {modules[0].getModulePosition(),modules[1].getModulePosition(),modules[2].getModulePosition(),modules[3].getModulePosition()};
    }

    public ChassisSpeeds getChasisSpeed() {
        return kinematics.toChassisSpeeds(
            modules[0].getSwerveModuleState(),
            modules[1].getSwerveModuleState(),
            modules[2].getSwerveModuleState(),
            modules[3].getSwerveModuleState()
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        var frontLeftModule = modules[1];
        var frontRightModule = modules[0];
        var backLeftModule = modules[2];
        var backRightModule = modules[3];
    
        builder.addDoubleProperty("Front Left Angle", () -> frontLeftModule.getModulePosition().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeftModule.getSwerveModuleState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> frontRightModule.getModulePosition().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRightModule.getSwerveModuleState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> backLeftModule.getModulePosition().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeftModule.getSwerveModuleState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> backRightModule.getModulePosition().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRightModule.getSwerveModuleState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> getGyroscopeRotation().getRadians(), null);
    }
}
