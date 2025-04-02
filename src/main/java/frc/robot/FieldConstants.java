package frc.robot;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.util.AllianceFlipUtil;

public final class FieldConstants {
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

	private static final Pose2d kReefAPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(18).get(), true, kReefOffset);
	private static final Pose2d kReefBPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(18).get(), false, kReefOffset);
	private static final Pose2d kReefCPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(17).get(), true, kReefOffset);
	private static final Pose2d kReefDPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(17).get(), false, kReefOffset);
	private static final Pose2d kReefEPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(22).get(), true, kReefOffset);
	private static final Pose2d kReefFPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(22).get(), false, kReefOffset);
	private static final Pose2d kReefGPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(21).get(), true, kReefOffset);
	private static final Pose2d kReefHPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(21).get(), false, kReefOffset);
	private static final Pose2d kReefIPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(20).get(), true, kReefOffset);
	private static final Pose2d kReefJPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(20).get(), false, kReefOffset);
	private static final Pose2d kReefKPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(19).get(), true, kReefOffset);
	private static final Pose2d kReefLPose = getReefPoseFromTag(kWeldedTagLayout.getTagPose(19).get(), false, kReefOffset);

	public static final Map<ReefLocs, Pose2d> kReefRobotLocationPoseMap = Map.ofEntries(
	Map.entry(ReefLocs.REEF_A, kReefAPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition)),
	Map.entry(ReefLocs.REEF_B, kReefBPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition)),
	Map.entry(ReefLocs.REEF_C, kReefCPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition)),
	Map.entry(ReefLocs.REEF_D, kReefDPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition)),
	Map.entry(ReefLocs.REEF_E, kReefEPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition)),
	Map.entry(ReefLocs.REEF_F, kReefFPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition)),
	Map.entry(ReefLocs.REEF_G, kReefGPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition)),
	Map.entry(ReefLocs.REEF_H, kReefHPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition)),
	Map.entry(ReefLocs.REEF_I, kReefIPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition)),
	Map.entry(ReefLocs.REEF_J, kReefJPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition)),
	Map.entry(ReefLocs.REEF_K, kReefKPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition)),
	Map.entry(ReefLocs.REEF_L, kReefLPose.transformBy(Constants.RobotK.kTransformReefPoseToRobotPosition))
);

	public static final Field2d kReefPosesField2d = new Field2d();

	static {
		// todo: build field2d
		kReefRobotLocationPoseMap.entrySet().stream().forEach(entry -> {
			ReefLocs key = entry.getKey();
			Pose2d value = entry.getValue();

			kReefPosesField2d.getObject("BLUE_"+key.toString()).setPose(value);
			kReefPosesField2d.getObject("RED_"+key.toString()).setPose(AllianceFlipUtil.flip(value));
		});
	}
}