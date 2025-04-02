package frc.robot;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.autons.TrajsAndLocs.ReefLocs;

final class FieldConstants {
	static Pose2d getReefPoseFromTag(Pose3d tag, boolean side, double sideOffset) {
	  // Pose3d mappedPose = mapPose(tag);
	  Rotation3d rotation = tag.getRotation();
	  double baseAngle = rotation.getAngle();
	  double cos = Math.cos(baseAngle);
	  double sin = Math.sin(baseAngle);
	  double xOffset = sideOffset * sin;
	  double yOffset = sideOffset * cos;
	  if (side) {
			return new Pose2d(
				tag.getX() + xOffset,
				tag.getY() - yOffset,
				rotation.toRotation2d().plus(Rotation2d.kPi)
			);
	  } else {
			return new Pose2d(
				tag.getX() - xOffset,
				tag.getY() + yOffset,
				rotation.toRotation2d().plus(Rotation2d.kPi)
			);
	  }
	}

	public static final AprilTagFieldLayout kWeldedTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); 
	public static final double kReefOffset = 0.328676 / 2;

	public static final Pose2d kReefAPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(18).get(), true, kReefOffset);
	public static final Pose2d kReefBPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(18).get(), false, kReefOffset);
	public static final Pose2d kReefCPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(17).get(), true, kReefOffset);
	public static final Pose2d kReefDPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(17).get(), false, kReefOffset);
	public static final Pose2d kReefEPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(22).get(), true, kReefOffset);
	public static final Pose2d kReefFPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(22).get(), false, kReefOffset);
	public static final Pose2d kReefGPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(21).get(), true, kReefOffset);
	public static final Pose2d kReefHPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(21).get(), false, kReefOffset);
	public static final Pose2d kReefIPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(20).get(), true, kReefOffset);
	public static final Pose2d kReefJPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(20).get(), false, kReefOffset);
	public static final Pose2d kReefKPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(19).get(), true, kReefOffset);
	public static final Pose2d kReefLPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(19).get(), false, kReefOffset);

	public static final Map<ReefLocs, Pose2d> kReefFacePoseMap = Map.ofEntries(
		Map.entry(ReefLocs.REEF_A, kReefAPose),
		Map.entry(ReefLocs.REEF_B, kReefBPose),
		Map.entry(ReefLocs.REEF_C, kReefCPose),
		Map.entry(ReefLocs.REEF_D, kReefDPose),
		Map.entry(ReefLocs.REEF_E, kReefEPose),
		Map.entry(ReefLocs.REEF_F, kReefFPose),
		Map.entry(ReefLocs.REEF_G, kReefGPose),
		Map.entry(ReefLocs.REEF_H, kReefHPose),
		Map.entry(ReefLocs.REEF_I, kReefIPose),
		Map.entry(ReefLocs.REEF_J, kReefJPose),
		Map.entry(ReefLocs.REEF_K, kReefKPose),
		Map.entry(ReefLocs.REEF_L, kReefLPose)
	);

	static {
		// todo: build field2d
	}
}