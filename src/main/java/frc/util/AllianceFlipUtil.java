// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.util;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldK.kFieldLengthMeters;
import static frc.robot.Constants.FieldK.kFieldWidthMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
  public static double flipXCoordinate(double xCoordinateMeters) {
    return kFieldLengthMeters - xCoordinateMeters;
  }

  public static double flipYCoordinate(double yCoordinateMeters) {
    return kFieldWidthMeters - yCoordinateMeters;
  }

  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  public static double applyX(double xCoordinate) {
    if (shouldFlip()) {
      return flipXCoordinate(xCoordinate);
    } else {
      return xCoordinate;
    }
  }

  public static double applyY(double yCoordinate) {
    if (shouldFlip()) {
      return flipYCoordinate(yCoordinate);
    } else {
      return yCoordinate;
    }
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return flip(translation);
    } else {
      return translation;
    }
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.k180deg);
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return flip(rotation);
    } else {
      return rotation;
    }
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return flip(pose);
    } else {
      return pose;
    }
  }

  public static Translation3d flip(Translation3d translation3d) {
    return new Translation3d(
      flipXCoordinate(translation3d.getX()), flipYCoordinate(translation3d.getY()), translation3d.getZ());
  }

  public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlip()) {
      return flip(translation3d);
    } else {
      return translation3d;
    }
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
    // return true;
  }
}