package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.Constants.AutoAlignmentK;
import frc.robot.Constants.FieldK;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.util.AllianceFlipUtil;

import java.lang.StackWalker.Option;
import java.rmi.StubNotFoundException;
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

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.Constants.FieldK.kTagLayout;

public class Vision {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    // private Optional<PhotonPipelineResult> m_latestPhotonPipelineResultOptional = Optional.empty();

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
     * <p>Calculates slightly future pose using constants and combines it with current pose to account somewhat for velocity.
     * <p>See {@link #getMostRealisticScorePose(SwerveDriveState, boolean)}
     * <p>Takes in a current swerve drive state and returns weighted pose from current velociites and position.
     * @param swerveDriveState Current state of the drivetrain
     * @return Velocity weighted pose.
     */
    public static Pose2d getVelocityWeightedPose(SwerveDriveState swerveDriveState) {
        Pose2d curPose = swerveDriveState.Pose;
        ChassisSpeeds curChassisSpeeds = swerveDriveState.Speeds;
        double heading = curPose.getRotation().getRadians();

        // yay the math should finally be worked out (but TODO check math. implemented teleop drive logging for it)
        double fieldXVel = curChassisSpeeds.vxMetersPerSecond * Math.cos(heading) + curChassisSpeeds.vyMetersPerSecond * Math.sin(Math.PI / 2 + heading);
        double fieldYVel = curChassisSpeeds.vyMetersPerSecond * Math.sin(heading) + curChassisSpeeds.vyMetersPerSecond * Math.cos(Math.PI / 2 + heading);
        Transform2d transformToFuture = new Transform2d(fieldXVel * AutoAlignmentK.kFutureDelta, fieldYVel * AutoAlignmentK.kFutureDelta,
            Rotation2d.fromRadians(curChassisSpeeds.omegaRadiansPerSecond * AutoAlignmentK.kFutureDelta));
        Pose2d futurePoseFieldCalculated = curPose.transformBy(transformToFuture);

        // this might cook? math would be more verbose and i trust the wpilib people to optimize their code better than mine
        return curPose.interpolate(futurePoseFieldCalculated, AutoAlignmentK.kFutureWeight);
    }

    /**
     * <p>See {@link #getMostRealisticScorePose(Pose2d, boolean)}
     * <p>Takes in a current pose and returns the id of the closest reef april tag
     * @param currentPose a Pose2d for the current position
     * @return returns empty if it fails for some reason to avoid crashing the robot in the event it fails
     */
    public static Optional<Integer> getClosestReefTagId(Pose2d currentPose) {
        // cache current alliance
        Optional<Alliance> curAlliance = DriverStation.getAlliance();

        // this WILL get updated. it loops through all april tags later
        Optional<AprilTag> closestReefAprilTag = Optional.empty();
        double minimumDistance = Double.MAX_VALUE; // meters (nothing will actually be this far away, right?)
        for (AprilTag aprilTag : FieldK.kTagLayout.getTags()) {
            // makes sure it is on the correct reef before doing anything
            if (!isTagIdOnAllianceReef(aprilTag.ID, curAlliance)) {
                continue;
            }
            Pose2d aprilTagPose = aprilTag.pose.toPose2d();
            Transform2d diff = currentPose.minus(aprilTagPose).plus(new Transform2d(new Translation2d(), Rotation2d.k180deg));
            // find the distance between x and y coordintaes and throw rotation in radians in there as a 3rd dimension
            // should work to throw out poses that have more disimilar rotations
            double distance = Math.sqrt(Math.pow(diff.getX(), 2) + Math.pow(diff.getY(), 2)
                + AutoAlignmentK.kRotationWeight * Math.pow(diff.getRotation().getRadians(), 2));
            // actually update values if the distance is the smallest
            if (distance <= minimumDistance) {
                closestReefAprilTag = Optional.of(aprilTag);
                minimumDistance = distance;
            }
        }
        if (closestReefAprilTag.isEmpty()) {
            System.out.println("AUTO ALIGN EMPTY closestReefAprilTag");
            return Optional.empty();
        }

        return Optional.of(closestReefAprilTag.get().ID);
    }

    /**
     * <p>See {@link #getMostRealisticScorePose(Pose2d, boolean)} or {@link #getMostRealisticScorePose(SwerveDriveState, boolean)}
     * <p>This takes in a reef aprilTag id and whether you want left or right reef and returns the correct pose.
     * @param tagId
     * @param rightReef
     * @return
     */
    public static Optional<Pose2d> getScorePose(int tagId, boolean rightReef) {
        ReefLocs correctReefLocation = null;
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
                System.out.println("AUTO ALIGN [VISION] FAIL: Vision::getReefScorePose switch case defaulted");
                return Optional.empty();
        }

        if (correctReefLocation == null) {
            System.out.println("AUTO ALIGN [VISION] FAIL: Vision::getReefScorePose correctReefLocation null uncaught");
            return Optional.empty();
        }

        // AllianceFlipUtil::flip handles checking whether flipping should occur
        return Optional.of(AllianceFlipUtil.apply(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(correctReefLocation)));
    }

    /**
     * Returns the most realistic scoring position.
     * @param curPose Current position
     * @param rightReef false for left reef, true for right reef
     * @return <p>Returns the scoring position in the form of a Pose2d if no error is encountered.
     * If it encounters an error, it returns empty to avoid crashing the robot in the event of a failure.
     */
    public static Optional<Pose2d> getMostRealisticScorePose(Pose2d curPose, boolean rightReef) {
        Optional<Integer> closestReefFaceTagId = getClosestReefTagId(curPose);
        if (closestReefFaceTagId.isEmpty()) {
            return Optional.empty();
        }
        return getScorePose(closestReefFaceTagId.get(), rightReef);
    }

    /**
     * Returns the most realistic scoring position taking into some account velocity
     * @param swerveDriveState Current drivetrain state
     * @param rightReef false for left reef, true for right reef
     * @return <p>Returns the scoring position in the form of a Pose2d if no error is encountered.
     * If it encounters an error, it returns empty to avoid crashing the robot in the event of a failure.
     */
    public static Optional<Pose2d> getMostRealisticScorePose(SwerveDriveState swerveDriveState, boolean rightReef) {
        Optional<Integer> closestReefFaceTagId = getClosestReefTagId(getVelocityWeightedPose(swerveDriveState));
        if (closestReefFaceTagId.isEmpty()) {
            return Optional.empty();
        }
        return getScorePose(closestReefFaceTagId.get(), rightReef);
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
        // if (unreadCameraResults.size() > 0) {
        //     m_latestPhotonPipelineResultOptional = Optional.of(unreadCameraResults.get(unreadCameraResults.size()-1));
        // } else {
        //     // i could make a threshold of time difference between result time and current time
        //     // to make it stale after a time, but that adds potential for silliness and i would
        //     // rather just not use something older when something new should come in regularly
        //     m_latestPhotonPipelineResultOptional = Optional.empty();
        // }

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

    /**
     * <p>Returns whether a tag id corresponds to an apriltag on the current alliances reef
     * @param givenId integer april tag id
     * @param curAlliance current alliance, if the optional is empty assume blue
     * @return whether a tag id corresponds to an apriltag on the current alliance reef
     */
    private static boolean isTagIdOnAllianceReef(int givenId, Optional<Alliance> curAlliance) {
        if (curAlliance.isEmpty() || curAlliance.get().equals(Alliance.Blue)) {
            if (curAlliance.isEmpty()) {
                System.out.println("VISION WARN: Vision::isTagIdOnAllianceReef default to blue alliance");
            }
            
            // this sets our function to use correct bounds to determine if a given tag is on the correct reef
            return givenId >= 17 && givenId <= 22;
        } else {
            return givenId >= 6 && givenId <= 11;
        }
    }

    /**
     * <p>Returns whether a tag id corresponds to an apriltag on the current alliances reef
     * <p>If no alliance is available from the driver station, assumes blue
     * <p>Consider caching the alliance and using {@link #isTagIdOnAllianceReef(int, Optional)}
     *  to improve performance
     * @param givenId integer april tag id
     * @return whether a tag id corresponds to an apriltag on the current alliance reef
     */
    private boolean isTagIdOnAllianceReef(int givenId) {
        return isTagIdOnAllianceReef(givenId, DriverStation.getAlliance());
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