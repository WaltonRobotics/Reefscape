package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldK;
import frc.util.AllianceFlipUtil;
import frc.util.WaltLogger;
import frc.util.WaltLogger.Pose3dLogger;
import frc.util.WaltLogger.Transform3dLogger;

import static frc.robot.Constants.FieldK.kTagLayout;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class Vision {

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.5, 1.5, 6.24);
    public static final Matrix<N3, N1> kMultipleTagStdDevs = VecBuilder.fill(0.5, 0.5, 6.24);

    public static final double kMaxPoseHeight = 0.405;
    public static final double kMaxPoseAngle = 0.3;

    // Total of 16 AprilTags
    // https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf (page 35 and more)
    // Tag Locations (1-16) | Source: 1,2,9,10 | Speaker: 3,4,7,8 | Amp: 5,6 | Stage 11,12,13,14,15,16
    // First half of locations are on red side, second half on blue side
    // (ex. source: 1,2 is red, 9,10)

    // Unsure if getFiducialId() returns the tags from 1-16 or 0-15 (assuming 0-15)
    public static final double[] TAG_WEIGHTS = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

    public static final record PhotonMeasurement (PhotonTrackedTarget target, double latencyMilliseconds) {}
    public static final record VisionMeasurement2d (Integer id, Double yaw, Double pitch, Double area) {}
    public static final record VisionMeasurement3d (EstimatedRobotPose estimate, Matrix<N3, N1> stdDevs) {}
    public static final record VisMeas3dEx (boolean hasTarget, Optional<VisionMeasurement3d> measOpt) {}
    
    // private final Matrix<N3, N1> kDefaultStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    private final PhotonCamera m_flatbotCam = new PhotonCamera("FlatbotCam");
    // private final PhotonCamera m_frontCam = new PhotonCamera("FrontCam");
    private final Transform3d m_roboToCam = new Transform3d(
        Units.inchesToMeters(-13.562), Units.inchesToMeters(11.562), Units.inchesToMeters(8.13), 
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(-45)));
    public final PhotonPoseEstimator m_flatbotCam_poseEstimator = 
        new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_roboToCam);

    private final Pose3dLogger log_flatbotCamOnRobot = WaltLogger.logPose3d("Vision", "frontCamOffset");
    private final Transform3dLogger log_speakerTag = WaltLogger.logTransform3d("Vision", "speakerTag");
    private final Pose3dLogger log_flatbotCamRawEstimate = WaltLogger.logPose3d("Vision", "frontCamRawEstimate");
    private final Pose3dLogger log_flatbotCamFilteredEstimate = WaltLogger.logPose3d("Vision", "frontCamFilteredEstimate");

    public Vision() {
        m_flatbotCam_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        log_flatbotCamOnRobot.accept(new Pose3d().plus(m_roboToCam));
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Optional<Matrix<N3, N1>> getEstimationStdDevs(
        Pose2d estimatedPose, PhotonPipelineResult pipelineResult) {
        var estStdDevs = kSingleTagStdDevs;
        var usedIds = pipelineResult.getMultiTagResult().get().fiducialIDsUsed;
        int numTags = usedIds.size();
        var targets = pipelineResult.getTargets();
        double avgDist = 0;
        double avgWeight = 0;
        for (var tgt : targets) {
            if (!usedIds.contains(tgt.getFiducialId())) { continue; } // skip tags not in layout
            var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            avgDist +=
                tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
            avgWeight += TAG_WEIGHTS[tgt.getFiducialId() - 1];
        }
        if (numTags == 0) return Optional.of(estStdDevs);

        avgDist /= numTags;
        if (avgDist > 5) return Optional.empty();

        avgWeight /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultipleTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        estStdDevs = estStdDevs.times(avgWeight);

        return Optional.of(estStdDevs);
    }

    public static int getMiddleSpeakerId() {
        boolean red = AllianceFlipUtil.shouldFlip();
        return red ? 4 : 7;
    }

    public static Pose3d getMiddleSpeakerTagPose() {
        return kTagLayout.getTagPose(getMiddleSpeakerId()).get();
    }

   public static Pose3d getTagPose(int id) {
        return kTagLayout.getTagPose(id).get();
    } 

    // TODO: someone else please sanity check me; i believe this actually returns robot position
    public VisMeas3dEx getFlatbotCamPoseEst() {
        var result = m_flatbotCam.getLatestResult();
        var estimateOpt = m_flatbotCam_poseEstimator.update(result);
        if (estimateOpt.isEmpty()) return new VisMeas3dEx(result.hasTargets(), Optional.empty());
        log_flatbotCamRawEstimate.accept(estimateOpt.get().estimatedPose);
        if (FieldK.inField(estimateOpt.get().estimatedPose.toPose2d()) && estimateOpt.get().estimatedPose.getZ() >= -0.2) {
            var filtered = estimateOpt.get();
            var stdDevsOpt = getEstimationStdDevs(filtered.estimatedPose.toPose2d(), result);
            if (stdDevsOpt.isEmpty()) {
                return new VisMeas3dEx(true, Optional.empty());
            }
            var stdDevs = stdDevsOpt.get();
            log_flatbotCamFilteredEstimate.accept(filtered.estimatedPose);
            return new VisMeas3dEx(true, Optional.of(new VisionMeasurement3d(filtered, stdDevs)));
        }

        return new VisMeas3dEx(false, Optional.empty());
    }

    public Supplier<Optional<List<VisionMeasurement2d>>> shooterDataSupplier() {
        return () -> {
            var result = m_flatbotCam.getLatestResult();
            if (result.hasTargets()) {
                var targets = result.getTargets();
                var measurements = new ArrayList<VisionMeasurement2d>();
                
                for (PhotonTrackedTarget t : targets) {
                    measurements.add(new VisionMeasurement2d(t.getFiducialId(), t.getYaw(), t.getPitch(), t.getArea()));
                }

                return Optional.of(measurements);
            }
            return Optional.empty();
        };
    }

    public Supplier<Optional<PhotonMeasurement>> speakerTargetSupplier() {
        return () -> {
            var result = m_flatbotCam.getLatestResult();
            if (result.hasTargets()) {
                for (var target : result.targets) {
                    if(target.getFiducialId() == getMiddleSpeakerId()) {
                        log_speakerTag.accept(target.getBestCameraToTarget());
                        var msmt = new PhotonMeasurement(target, result.metadata.getLatencyMillis());
                        return Optional.of(msmt);
                    }
                }
            }
            return Optional.empty();
        };
    }

    /**
     * @param tagNum
     * @return whether or not the target with fiducial id tagNum is visible
     */
    public boolean isTagVisible(int tagNum) {
        var results = m_flatbotCam.getLatestResult();

        if (results.hasTargets()) {
            for (var target : results.getTargets()) {
                if (target.getFiducialId() == tagNum) {
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * @param tag tag number to getYaw for
     * @return If target is visible: returns the yaw
     *          Not visible: returns empty Optional 
     */
    public Optional<Double> getTagYaw(int tagNum) {
        Optional<Double> tagYaw = Optional.empty();
        var results = m_flatbotCam.getLatestResult();

        if (results.hasTargets()) {
            for (var target : results.getTargets()) {
                if (target.getFiducialId() == tagNum) {
                    // Found Tag, record its information
                    tagYaw = Optional.of(target.getYaw());
                }
            }
        }
        return tagYaw;
    }

    /**
     * @param tagNum tag number to get XDiff for
     * @return If target is visible: returns the x difference
     *          Not visible: returns empty Optional
     */
    public Optional<Double> getTagXDiff(int tagNum) {
        Optional<Double> tagXDiff = Optional.empty();
        var results = m_flatbotCam.getLatestResult();

        if (results.hasTargets()) {
            for (var target : results.getTargets()) {
                if (target.getFiducialId() == tagNum) {
                    var bestTransform = target.getBestCameraToTarget();
                    tagXDiff = Optional.of(bestTransform.getX());
                }
            }
        }

        return tagXDiff;
    }

    /**
     * @param tagNum tag number to get YDiff for
     * @return If target is visible: returns the y difference
     *          Not visible: returns empty Optional
     */
    public Optional<Double> getTagYDiff(int tagNum) {
        Optional<Double> tagYDiff = Optional.empty();
        var results = m_flatbotCam.getLatestResult();

        if (results.hasTargets()) {
            for (var target : results.getTargets()) {
                if (target.getFiducialId() == tagNum) {
                    var bestTransform = target.getBestCameraToTarget();
                    tagYDiff = Optional.of(bestTransform.getY());
                }
            }
        }

        return tagYDiff;
    }
}