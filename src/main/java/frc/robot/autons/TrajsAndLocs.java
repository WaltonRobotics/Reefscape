package frc.robot.autons;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class TrajsAndLocs {
    /* 
     * check docu for naming conventions:
    */

    private static enum StartingLocs {
        RIGHT(new Pose2d(1, 2, Rotation2d.fromDegrees(0))),
        MID_1(new Pose2d(1, 2, Rotation2d.fromDegrees(0))),
        MID_12(new Pose2d(1, 2, Rotation2d.fromDegrees(0))),
        LEFT(new Pose2d(1, 2, Rotation2d.fromDegrees(0)));

        public final Pose2d pose;
        private StartingLocs(Pose2d _pose) {
            pose = _pose;
        }
    }
    
    /*
     * the first place you'll score in a match.
     * Kept separate so that ppl dont accidentally choose a first reefLoc that doesn't have a traj going form [startingLoc] -> [reefLoc]
     */
    public static enum FirstScoringLocs {
        REEF_H(StartingLocs.MID_1, ""),
        REEF_I(StartingLocs.LEFT, ""),
        REEF_J(StartingLocs.LEFT, ""),

        REEF_E(StartingLocs.RIGHT, ""),
        REEF_F(StartingLocs.RIGHT, ""),
        REEF_G(StartingLocs.MID_12, "");

        public Pair<StartingLocs, String> m_startAndTraj;

        /*
         * for possible starting scoring positions
         */
        private FirstScoringLocs(StartingLocs startLoc, String trajName) {
            m_startAndTraj = new Pair<TrajsAndLocs.StartingLocs,String>(startLoc, trajName);
        }
    }

    public static enum ReefLocation {
        REEF_H(FirstScoringLocs.REEF_H),
        REEF_I(FirstScoringLocs.REEF_I),
        REEF_J(FirstScoringLocs.REEF_J),
        REEF_K,
        REEF_L,
        REEF_A,
        REEF_B,
        REEF_C,
        REEF_D,
        REEF_E(FirstScoringLocs.REEF_E),
        REEF_F(FirstScoringLocs.REEF_F),
        REEF_G(FirstScoringLocs.REEF_G),
        INVALID_LOC;

        public FirstScoringLocs m_pairedLoc;

        public static ArrayList<ReefLocation> OptimalLeftStartCycles = new ArrayList<>();
        static {
            OptimalLeftStartCycles.add(REEF_H);
            OptimalLeftStartCycles.add(REEF_I);
            OptimalLeftStartCycles.add(REEF_J);
            OptimalLeftStartCycles.add(REEF_K);
        }

        private ReefLocation() {
            m_pairedLoc = null;
        }

        /*
         * for the positions that *could* be starting locations
         */
        private ReefLocation(FirstScoringLocs sameLoc) {
            m_pairedLoc = sameLoc;
        }

        public static ReefLocation getSameLoc(FirstScoringLocs startLoc) {
            switch (startLoc) {
                case REEF_H:
                    return REEF_H;
                case REEF_I:
                    return REEF_I;
                case REEF_J:
                    return REEF_J;
                case REEF_E:
                    return REEF_E;
                case REEF_F:
                    return REEF_F;
                case REEF_G:
                    return REEF_G;
                default:
                    return INVALID_LOC; //TODO: find better fail case T^T
            }
        }
    }

    public static enum HpStation {
        /* pov from driver station */
        CS_RIGHT,
        CS_LEFT;
    }

    public static class ReefHpPair extends Pair<ReefLocation, HpStation> {
        public ReefHpPair(ReefLocation reef, HpStation hp) {
            super(reef, hp);
        }
    }

    public static class HpReefPair extends Pair<HpStation, ReefLocation> {
        public HpReefPair(HpStation hp, ReefLocation reef) {
            super(hp, reef);
        }
    }

    //TODO: fill out values frfr
    public static class Trajectories {

        public static HashMap<Pair<ReefLocation, HpStation>, String> ReefToHpMap = new HashMap<>();
        public static HashMap<Pair<HpStation, ReefLocation>, String> HpToReefMap = new HashMap<>();

        static {
            // fill in maps here
            ReefToHpMap.put(new Pair<ReefLocation, HpStation>(ReefLocation.REEF_A, HpStation.CS_LEFT), "cs_6_left");
        }
    }
}
