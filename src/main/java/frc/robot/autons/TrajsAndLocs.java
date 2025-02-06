package frc.robot.autons;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class TrajsAndLocs {
    /* 
     * possible starting locs
     * liable to add more
     * 
     * POV: driver station
     */
    private static enum StartingLocs {
        // TODO: fill out the Real Values
        LEFT(new Pose2d(1, 2, Rotation2d.fromDegrees(0))),
        MID(new Pose2d(1, 2, Rotation2d.fromDegrees(0))),
        RIGHT(new Pose2d(1, 2, Rotation2d.fromDegrees(0)));

        public final Pose2d pose;
        private StartingLocs(Pose2d _pose) {
            pose = _pose;
        }
    }

    public static enum ReefLocs {
        REEF_H,
        REEF_I,
        REEF_J,
        REEF_K,
        REEF_L,
        REEF_A,
        REEF_B,
        REEF_C,
        REEF_D,
        REEF_E,
        REEF_F,
        REEF_G,
        INVALID_LOC;

        /* 
         * enums used in autonchooser
         * list of best possible next locs given the start cycle u were just at
         */
        public static ArrayList<ReefLocs> OptimalLeftStartCycles = new ArrayList<>();
        static {
            OptimalLeftStartCycles.add(REEF_H);
            OptimalLeftStartCycles.add(REEF_I);         
            OptimalLeftStartCycles.add(REEF_J);
            OptimalLeftStartCycles.add(REEF_K);
        }
        public static ArrayList<ReefLocs> OptimalMidStartCycles = new ArrayList<>();
        static {
            OptimalLeftStartCycles.add(REEF_E);
            OptimalLeftStartCycles.add(REEF_F);         
            OptimalLeftStartCycles.add(REEF_G);
            OptimalLeftStartCycles.add(REEF_H);
            OptimalLeftStartCycles.add(REEF_I);
            OptimalLeftStartCycles.add(REEF_J);
        }
        public static ArrayList<ReefLocs> OptimalRightStartCycles = new ArrayList<>();
        static {
            OptimalLeftStartCycles.add(REEF_D);
            OptimalLeftStartCycles.add(REEF_E);         
            OptimalLeftStartCycles.add(REEF_F);
            OptimalLeftStartCycles.add(REEF_G);
        }
    }

    // will get put on autonchooser
    public static enum HPStation {
        HP_LEFT,
        HP_RIGHT;
    }

    public static class ReefHPPair extends Pair<ReefLocs, HPStation> {
        public ReefHPPair(ReefLocs reef, HPStation hp) {
            super(reef, hp);
        }
    }

    public static class HPReefPair extends Pair<HPStation, ReefLocs> {
        public HPReefPair(HPStation hp, ReefLocs reef) {
            super(hp, reef);
        }
    }

    
}