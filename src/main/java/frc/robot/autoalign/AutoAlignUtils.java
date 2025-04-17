package frc.robot.autoalign;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.MovingAutoAlignK;
import frc.robot.Constants.SharedAutoAlignK;
import frc.robot.Constants.FieldK;
import frc.robot.FieldConstants;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.subsystems.Swerve;
import frc.util.AllianceFlipUtil;

/**
 * Both AutoAlign classes extend this because it has helper methods they both need to use
 */
public class AutoAlignUtils {
    private static final String kTopicPrefix = "Robot/AutoAlignUtils/";
    private static final StructPublisher<Pose2d> log_predictedFuturePose = NetworkTableInstance.getDefault()
        .getStructTopic(kTopicPrefix + "predictedFuturePose", Pose2d.struct).publish();

        /**
         * <p> Intakes current state of the robot to try and predict where it will be in the future and 
         *  guess where the driver will want to score
         * @param curState Current state of the swerve drive state when it is commanded to auto align
         * @return Returns an int corresponding to the tag id for most likely reef face
         */
    public static int getMostLikelyReefTagId(SwerveDriveState curState) {
        // cache current alliance
        Optional<Alliance> curAlliance = DriverStation.getAlliance();
        // this all handles looking slightly ahead to the future and predicting future position
        Pose2d curPose = curState.Pose;
        ChassisSpeeds fieldRelativeChassisSpeeds = Swerve.getFieldRelativeChassisSpeeds(curState);
        Transform2d transformToFuture = new Transform2d(
            fieldRelativeChassisSpeeds.vxMetersPerSecond * MovingAutoAlignK.kFutureDelta, 
            fieldRelativeChassisSpeeds.vyMetersPerSecond * MovingAutoAlignK.kFutureDelta, 
            Rotation2d.fromRadians(fieldRelativeChassisSpeeds.omegaRadiansPerSecond * MovingAutoAlignK.kFutureDelta));
        Pose2d futurePose = curPose.transformBy(transformToFuture);
        log_predictedFuturePose.accept(futurePose);

        // this WILL get updated. it loops through all april tags later
        Optional<AprilTag> closestReefAprilTag = Optional.empty();
        double minimumDistance = Double.MAX_VALUE; // meters (nothing will actually be this far away, right?)
        for (AprilTag aprilTag : FieldK.kTagLayout.getTags()) {
            // makes sure it is on the correct reef before doing anything
            if (!isTagIdOnAllianceReef(aprilTag.ID, curAlliance)) {
                continue;
            }
            Pose2d aprilTagPose = aprilTag.pose.toPose2d();
            Transform2d diff = futurePose.minus(aprilTagPose).plus(new Transform2d(new Translation2d(), Rotation2d.k180deg));
            // find the distance between x and y coordintaes and throw rotation in radians in there as a 3rd dimension
            // should work to throw out poses that have more disimilar rotations
            // weight rotation less so that it acts more as a tie breaker than a real part of this
            double distance = Math.sqrt(Math.pow(diff.getX(), 2) + Math.pow(diff.getY(), 2)
                + MovingAutoAlignK.kRotationWeight * Math.pow(diff.getRotation().getRadians(), 2));
            // actually update values if the distance is the smallest
            if (distance <= minimumDistance) {
                closestReefAprilTag = Optional.of(aprilTag);
                minimumDistance = distance;
            }
        }
        // this is really a relic from when optionals were more part of this
        // the code should however always output a reef tag and prevent the Map from getting angry with us
        if (closestReefAprilTag.isEmpty()) {
            System.out.println("Vision::getClosestReefTagId empty closestReefAprilTag");
        }

        return closestReefAprilTag.get().ID;
    }

    /**
     * <p> This intakes a tagId and boolean to determine which reef branch it should align to
     * <p> Then use pose map in FIeldConstants to determine corresponding Pose2d
     * @param tagId tagId of reef face you've already identified you'd like to score on.
     *  <b>WARNING</b>: if you input an int that isn't a reef tag id you are going to get Pose2d.kZero
     * @param rightReef true to score right branch, false to score left branch
     * @return returns a pose to move to be aligned ideally
     */
    public static Pose2d getScorePose(int tagId, boolean rightReef) {
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
                return AllianceFlipUtil.apply(Pose2d.kZero);
        }

        if (correctReefLocation == null) {
            System.out.println("AUTO ALIGN [VISION] FAIL: Vision::getReefScorePose correctReefLocation null uncaught");
            return AllianceFlipUtil.apply(Pose2d.kZero);
        }

        // handles checking whether flipping should occur
        Pose2d pose = FieldConstants.kReefRobotLocationPoseMap.get(correctReefLocation);
        return AllianceFlipUtil.apply(pose);
    }

    /**
     * <p> This is simply a helper method to combine {@link #getScorePose(int, boolean)} and {@link #getMostLikelyReefTagId(SwerveDriveState)}
     *  because it makes the API easier to work with.
     * @param drivetrainState State of the drivetrain at the moment you would like to find most likely score pose
     * @param rightReef True right reef, false left reef
     * @return Returns a Pose2d that the robot should be in for ideal scoring on the most likely branch the robot would score on.
     */
    public static Pose2d getMostLikelyScorePose(SwerveDriveState drivetrainState, boolean rightReef) {
        return getScorePose(getMostLikelyReefTagId(drivetrainState), rightReef);
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
     * <p> Returns whether a tag id corresponds to an apriltag on the current alliances reef
     * <p> If no alliance is available from the driver station, assumes blue
     * <p> Consider caching the alliance when repeated calls are neccessary 
     *  and using {@link #isTagIdOnAllianceReef(int, Optional)}
     *  to improve performance
     * @param givenId integer april tag id
     * @return whether a tag id corresponds to an apriltag on the current alliance reef
     */
    private boolean isTagIdOnAllianceReef(int givenId) {
        return isTagIdOnAllianceReef(givenId, DriverStation.getAlliance());
    }

    /**
     * <p> 8033 code. Returns true when a pose is close to another with tolerances defined in SharedAutoAlignK
     * @param pose
     * @param pose2
     * @return true when within tolerances, false otherwise
     */
    public static boolean isInTolerance(Pose2d pose, Pose2d pose2) {
        final Transform2d diff = pose.minus(pose2);
        return MathUtil.isNear(
                0.0, Math.hypot(diff.getX(), diff.getY()), SharedAutoAlignK.kFieldTranslationTolerance.in(Meters))
            && MathUtil.isNear(
                0.0, diff.getRotation().getRadians(), SharedAutoAlignK.kFieldRotationTolerance.in(Radians));
    }

    /**
     * <p> Modified isInTolerance. Requires the provided speeds to be within the tolerance defined in SharedAutoAlignK
     *  along with the normal requirements to be in tolerances.
     * @param pose
     * @param pose2
     * @param speeds
     * @return true when within tolerances, false otherwise
     */
    public static boolean isInTolerance(Pose2d pose, Pose2d pose2, ChassisSpeeds speeds) {
        return isInTolerance(pose, pose2, speeds, SharedAutoAlignK.kFieldTranslationTolerance.in(Meters), 
            SharedAutoAlignK.kFieldRotationTolerance.in(Radians),
            SharedAutoAlignK.kFinishedVelTolerance);
    }

    public static boolean isInTolerance(Pose2d pose, 
            Pose2d pose2, 
            ChassisSpeeds speeds, 
            double linearTolerance, 
            double rotationalTolerance,
            double velocityTolerance) {
        return isInTolerance(pose, pose2, linearTolerance, rotationalTolerance)
            && MathUtil.isNear(
                0.0,
                Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                velocityTolerance
            );
    }

    public static boolean isInTolerance(Pose2d pose,
            Pose2d pose2,
            double linearTolerance,
            double rotationalTolerance) {
        final Transform2d diff = pose.minus(pose2);
        return MathUtil.isNear(
                0.0, Math.hypot(diff.getX(), diff.getY()), linearTolerance)
            && MathUtil.isNear(
                0.0, diff.getRotation().getRadians(), rotationalTolerance);
    }

    // public static BooleanSupplier isInToleranceSupplier(
    //         Supplier<Pose2d> pose,
    //         Supplier<Pose2d> pose2,
    //         Supplier<ChassisSpeeds> speeds,
    //         DoubleSupplier linearTolerance,
    //         DoubleSupplier rotationTolerance,
    //         DoubleSupplier velocityTolerance) {
    //     return () -> {
    //         isInTolerance(pose.get(), pose2.get(), speeds.get(), linearTolerance.getAsDouble(), rotationTolerance.getAsDouble()
    //             velocityTolerance.getAsDouble());
    //     };
    // }
}
