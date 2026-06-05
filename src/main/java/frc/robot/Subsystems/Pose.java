package frc.robot.Subsystems;

import java.util.List;
import java.util.ArrayList;
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
    public record CameraSetup(PhotonCamera camera, PhotonPoseEstimator estimator) {}
    private final List<CameraSetup> cameras = new ArrayList<>();
    private boolean hasReceivedFirstCameraUpdate = false;

    public Pose(SwerveDriveKinematics kinematics, Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> positionProvider) {
        for (Constants.Pose.Camera cameraConfig: Constants.Pose.Cameras) {
            PhotonCamera cameraInstance = new PhotonCamera(cameraConfig.name());
            PhotonPoseEstimator estimatorInstance = new PhotonPoseEstimator(Constants.Pose.FieldLayout, cameraConfig.transform());
            cameras.add(new CameraSetup(cameraInstance, estimatorInstance));
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
        updateCameraPoses();

        poseEstimator.update(rotationSupplier.get(), positionSupplier.get());

        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    public void updateCameraPoses() {
        for (CameraSetup setup : cameras) {
            PhotonCamera camera = setup.camera();
            PhotonPoseEstimator photonEstimator = setup.estimator();
            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                var visionEstimate = photonEstimator.estimateCoprocMultiTagPose(result);
                if (visionEstimate.isEmpty()) {
                    visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
                }

                if (visionEstimate.isPresent()) {
                    var e = visionEstimate.get();
                    var targets = e.targetsUsed;

                    //no results filter
                    if (targets.size() == 0) {
                        SmartDashboard.putString(camera.getName() + " rejection", "No tags");
                        continue;
                    }

                    //single target ambiguity filter
                    if (targets.size() == 1 && targets.get(0).getPoseAmbiguity() > 0.2) {
                        SmartDashboard.putString(camera.getName() + " rejection", "Single tag ambiguity");
                        continue;
                    }

                    //height filter
                    if (Math.abs(e.estimatedPose.getZ()) > 0.5) {
                        SmartDashboard.putString(camera.getName() + " rejection", "Height");
                        continue;
                    }

                    //teleportation filter
                    var distanceTraveled = e.estimatedPose.toPose2d().getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation());
                    if (distanceTraveled > 0.5 && hasReceivedFirstCameraUpdate) {
                        SmartDashboard.putString(camera.getName() + " rejection", "Teleportation");
                        continue;
                    }

                    //out of bounds filter
                    var pose = e.estimatedPose.toPose2d();
                    if (pose.getX() < 0 || pose.getX() > Constants.Pose.FieldLayout.getFieldLength() ||
                        pose.getY() < 0 || pose.getY() > Constants.Pose.FieldLayout.getFieldWidth()) {
                        SmartDashboard.putString(camera.getName() + " rejection", "Out of bounds");
                        continue;
                    }

                    Matrix<N3, N1> stdDevs = getStdDevs(photonEstimator, e, result.getTargets());
                    poseEstimator.addVisionMeasurement(e.estimatedPose.toPose2d(), e.timestampSeconds, stdDevs);
                    hasReceivedFirstCameraUpdate = true;
                    SmartDashboard.putString(camera.getName() + " rejection", "No rejection");

                    double lag = Timer.getFPGATimestamp() - e.timestampSeconds;
                    SmartDashboard.putNumber(camera.getName() + " vision lag", lag);
                }
                else {
                    SmartDashboard.putString(camera.getName() + " rejection", "No estimate");
                }
            }
        }
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
