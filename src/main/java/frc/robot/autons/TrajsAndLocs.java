package frc.robot.autons;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class TrajsAndLocs {
    /* 
     * possible starting locs
     * liable to add more
     * 
     * POV: driver station
     */
    public static enum StartingLoc {
        // TODO: fill out the Real Values
        SUPER_LEFT("Start_Left"),
        LEFT("Start_Left"),
        MID_G("Start_Mid"),
        MID_H("Start_Mid"),
        SUPER_RIGHT("Start_Mid"),
        RIGHT("Start_Right");

        public final String str;
        private StartingLoc(String _str ) {
            str = _str;
        }
    }

    public static enum ReefLoc {
        REEF_A("A"),
        REEF_B("B"),
        REEF_C("C"),
        REEF_D("D"),
        REEF_E("E"),
        REEF_F("F"),
        REEF_G("G"),
        REEF_H("H"),
        REEF_I("I"),
        REEF_J("J"),
        REEF_K("K"),
        REEF_L("L");

        public final String str;
        private ReefLoc(String _str) {
            str = _str;
        }
        
    }

    public static enum HPStation {
        HP_LEFT(new Pose2d(Meters.of(1.45), Meters.of(6.89), Rotation2d.fromRadians(-0.90)), "Left"),
        HP_RIGHT(new Pose2d(Meters.of(1.53), Meters.of(1.16), Rotation2d.fromRadians(0.92)), "Right");
        
        public final Pose2d pose;
        public final String str;
        private HPStation(Pose2d _pose, String _str) {
            pose = _pose;
            str = _str;
        }
    }

    // trajectory naming
    public static String getStartingTrajName(StartingLoc startingLoc, ReefLoc scoringLoc, boolean isShort) {
        return isShort ? startingLoc.str + "_" + scoringLoc.str + "_short" : startingLoc.str + "_" + scoringLoc.str;
    }

    public static String getToReefTrajName(HPStation hpStation, ReefLoc scoringLoc, boolean isShort) {
        return isShort ? hpStation.str + "_" + scoringLoc.str + "_short" : hpStation.str + "_" + scoringLoc.str;
    }

    public static String getToHPTrajName(ReefLoc scoringLoc, HPStation hpStation, boolean isShort) {
        return isShort ? scoringLoc.str + "_" + hpStation.str + "_short" : scoringLoc.str + "_" + hpStation.str;
    }

}