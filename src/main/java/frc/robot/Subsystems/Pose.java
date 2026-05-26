package frc.robot.Subsystems;

import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pose extends SubsystemBase {
    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<SwerveModulePosition[]> positionSupplier;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();
    private final HashMap<PhotonCamera, PhotonPoseEstimator> cameras= new HashMap<>();

    public Pose(SwerveDriveKinematics kinematics, Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> positionProvider) {
        for (Constants.Pose.Cameras cameraConfig: Constants.Pose.Cameras.values()) {
            PhotonCamera cameraInstance = new PhotonCamera(cameraConfig.name());
            PhotonPoseEstimator estimatorInstance = new PhotonPoseEstimator(Constants.Pose.FieldLayout, cameraConfig.offset);
            cameras.put(cameraInstance, estimatorInstance);
        }

        this.rotationSupplier = rotationSupplier;
        this.positionSupplier = positionProvider;

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            rotationSupplier.get(),
            positionSupplier.get(),
            new Pose2d(),
            VecBuilder.fill(Constants.Pose.Odometry.PositionStdDev, Constants.Pose.Odometry.PositionStdDev, Constants.Pose.Odometry.AngleStdDev),
            // We calculate these upon update and give the real values then
            VecBuilder.fill(999999, 999999, 999999)
        );

        SmartDashboard.putData("Field", field);
    }
 
    @Override
    public void periodic() {
        for (Map.Entry<PhotonCamera, PhotonPoseEstimator> entry : cameras.entrySet()) {
            PhotonCamera camera = entry.getKey();
            PhotonPoseEstimator photonEstimator = entry.getValue();
            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                var visionEstimate = photonEstimator.estimateCoprocMultiTagPose(result);
                if (visionEstimate.isEmpty()) {
                    visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
                }

                if (visionEstimate.isPresent()) {
                    var e = visionEstimate.get();
                    var targets = e.targetsUsed;

                    //ambiguity filter
                    if (targets.size() == 1 && targets.get(0).getPoseAmbiguity() > 0.2) {
                        continue;
                    }

                    //no results filter
                    if (targets.size() == 0) {
                        continue;
                    }

                    //height filter
                    if (Math.abs(e.estimatedPose.getZ()) > 0.5) {
                        continue;
                    }

                    //teleportation filter
                    if (e.estimatedPose.toPose2d()
                            .getTranslation()
                            .getDistance(poseEstimator.getEstimatedPosition().getTranslation()) > 0.5) {
                        continue;
                    }

                    Matrix<N3, N1> stdDevs = getStdDevs(photonEstimator, e, result.getTargets());
                    poseEstimator.addVisionMeasurement(e.estimatedPose.toPose2d(), e.timestampSeconds, stdDevs);

                    double lag = Timer.getFPGATimestamp() - e.timestampSeconds;
                    SmartDashboard.putNumber("Vision/MeasurementLag_s", lag);
                }
            }
        }

        poseEstimator.update(rotationSupplier.get(), positionSupplier.get());

        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    private Matrix<N3, N1> getStdDevs(PhotonPoseEstimator photonEstimator, EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
        var estStdDevs = Constants.Pose.SingleTagStdDevs;
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
                estStdDevs = Constants.Pose.MultiTagStdDevs;
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

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
