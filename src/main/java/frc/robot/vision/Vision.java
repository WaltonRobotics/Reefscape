package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.Constants.FieldK;
import frc.robot.Constants.FieldK.Reef;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.util.AllianceFlipUtil;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.FieldK.kTagLayout;

public class Vision {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private Optional<PhotonPipelineResult> m_latestPhotonPipelineResultOptional = Optional.empty();

    // Simulation
    private PhotonCameraSim m_cameraSim;
    private final VisionSim m_visionSim;
    private final String m_simVisualName;

    private String m_cameraName;
    private final Transform3d m_roboToCam;

    //Constants
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.5, 1.5, 6.24);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 6.24);

    private final StructPublisher<Pose2d> log_camPose;
    private final DoubleArrayPublisher log_stdDevs;

    public Vision(String cameraName, String simVisualName, Transform3d roboToCam, VisionSim visionSim, SimCameraProperties simCameraProperties) {
        m_cameraName = cameraName;
        m_camera = new PhotonCamera(m_cameraName);
        m_roboToCam = roboToCam;
        m_simVisualName = simVisualName;
        m_visionSim = visionSim;

        photonEstimator =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, roboToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        log_camPose = NetworkTableInstance.getDefault()
            .getStructTopic("Vision/" + cameraName + "/estRobotPose", Pose2d.struct).publish();
        log_stdDevs = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("Vision/" + cameraName + "/stdDevs").publish(); 

        // Simulation
        if (Robot.isSimulation()) {
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible targets.
            m_cameraSim = new PhotonCameraSim(m_camera, simCameraProperties);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(m_cameraSim, m_roboToCam);

            m_cameraSim.enableDrawWireframe(true);
        }
    }

    /**
     * @param rightReef Negative left reef, positive right reef
     * @return Returns empty optional if an ideal robot pose could not be found, returns a Pose2d of where to go if it can be found
     */
    public Optional<Pose2d> getReefScorePose(boolean rightReef) {
        Optional<PhotonTrackedTarget> bestReefTagOptional = getBestReefTag();
        // if there is no good reef april tag available, we can't get a pose
        if (bestReefTagOptional.isEmpty()) {
            System.out.println("no reef tag available");
            return Optional.empty();
        }
        PhotonTrackedTarget bestReefTag = bestReefTagOptional.get();
        int tagId = bestReefTag.getFiducialId();
        ReefLocs correctReefLocation;
        // map nearest tag and left vs right to correct reef branch
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            // blue
            switch (tagId) {
                case 18:
                    correctReefLocation = rightReef ? ReefLocs.REEF_B : ReefLocs.REEF_A;
                    break;
                case 17:
                    correctReefLocation = rightReef ? ReefLocs.REEF_D : ReefLocs.REEF_C;
                    break;
                case 22:
                    correctReefLocation = rightReef ? ReefLocs.REEF_F : ReefLocs.REEF_E;
                    break;
                case 21:
                    correctReefLocation = rightReef ? ReefLocs.REEF_H : ReefLocs.REEF_G;
                    break;
                case 20:
                    correctReefLocation = rightReef ? ReefLocs.REEF_J : ReefLocs.REEF_I;
                    break;
                case 19:
                    correctReefLocation = rightReef ? ReefLocs.REEF_L : ReefLocs.REEF_K;
                    break;
                default:
                    System.out.println("VISION[109] WARN: getBestReefTag returned not reef tag for some reason");
                    return Optional.empty();
            }
        } else {
            // red
            switch (tagId) {
                case 7:
                    correctReefLocation = rightReef ? ReefLocs.REEF_B : ReefLocs.REEF_A;
                    break;
                case 8:
                    correctReefLocation = rightReef ? ReefLocs.REEF_D : ReefLocs.REEF_C;
                    break;
                case 9:
                    correctReefLocation = rightReef ? ReefLocs.REEF_F : ReefLocs.REEF_E;
                    break;
                case 10:
                    correctReefLocation = rightReef ? ReefLocs.REEF_H : ReefLocs.REEF_G;
                    break;
                case 11:
                    correctReefLocation = rightReef ? ReefLocs.REEF_J : ReefLocs.REEF_I;
                    break;
                case 6:
                    correctReefLocation = rightReef ? ReefLocs.REEF_L : ReefLocs.REEF_K;
                    break;
                default:
                    System.out.println("VISION[134] WARN: getBestReefTag returned not reef tag for some reason");
                    return Optional.empty();
            }
        }
        return AllianceFlipUtil.shouldFlip() ? Optional.of(AllianceFlipUtil.flip(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(correctReefLocation)))
            : Optional.of(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(correctReefLocation));
    }

    /**
     * This code selects the best reef tag.
     * Only selects from tags on the correct reef for the correct alliance as selected in FMS/DS.
     * @return Returns the a PhotonTrackedTarget for the best reef tag if possible. <p>
     *  Otherwise return empty
     */
    public Optional<PhotonTrackedTarget> getBestReefTag() {
        // if no latest result is available, then we can't find the best reef tag
        if (m_latestPhotonPipelineResultOptional.isEmpty()) {
            System.out.println("no latest photon pipeline result available");
            return Optional.empty();
        }
        PhotonPipelineResult latestPhotonPipelineResult = m_latestPhotonPipelineResultOptional.get();
        // if there are no april tags available, then we can't find the best reef tag
        if (!latestPhotonPipelineResult.hasTargets()) {
            System.out.println("no april tags present");
            return Optional.empty();
        }

        // if the best target is on the reef, just return ts (that shit)
        if (isTagIdOnAllianceReef(latestPhotonPipelineResult.getBestTarget().getFiducialId())) {
            return Optional.of(latestPhotonPipelineResult.getBestTarget());
        }
        
        // PhotonPipelineResult::getTargets() should return the targets in sort order as set in photonvision UI
        List<PhotonTrackedTarget> trackedTargets = latestPhotonPipelineResult.getTargets();
        int[] trackedTargetIds = new int[trackedTargets.size()];
        for (int i = 0; i < trackedTargetIds.length; i++) {
            trackedTargetIds[i] = trackedTargets.get(i).fiducialId;
        }
        for (int i = 0; i < trackedTargets.size(); i++) {
            if (isTagIdOnAllianceReef(trackedTargets.get(i).getFiducialId())) {
                return Optional.of(trackedTargets.get(i));
            }
        }
        
        // if we get to the end, there must not be any april tags on the reef
        return Optional.empty();
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        List<PhotonPipelineResult> unreadCameraResults = m_camera.getAllUnreadResults();

        // update m_latestPhotonPipelineResult
        if (unreadCameraResults.size() > 0) {
            m_latestPhotonPipelineResultOptional = Optional.of(unreadCameraResults.get(unreadCameraResults.size()-1));
        } else {
            // i could make a threshold of time difference between result time and current time
            // to make it stale after a time, but that adds potential for silliness and i would
            // rather just not use something older when something new should come in regularly
            m_latestPhotonPipelineResultOptional = Optional.empty();
        }

        for (var change : unreadCameraResults) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
            log_stdDevs.accept(curStdDevs.getData());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                m_visionSim.getSimDebugField()
                                    .getObject(m_simVisualName)
                                    .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            m_visionSim.getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }
        }

        if (visionEst.isPresent()) {
            log_camPose.accept(visionEst.get().estimatedPose.toPose2d());
        }

        return visionEst;
    }

    private boolean isTagIdOnAllianceReef(int givenId) {
        if (DriverStation.getAlliance().isEmpty() || (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Blue))) {
            if (DriverStation.getAlliance().isEmpty()) {
                System.out.println("VISION[165] WARN: DriverStation not providing alliance color, default to blue");
            }
            
            // this sets our function to use correct bounds to determine if a given tag is on the correct reef
            return givenId >= 17 && givenId <= 22;
        } else {
            return givenId >= 6 && givenId <= 11;
        }
    } 

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
}