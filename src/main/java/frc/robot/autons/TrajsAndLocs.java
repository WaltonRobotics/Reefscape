package frc.robot.autons;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.Pair;

public abstract class TrajsAndLocs {
    /* 
     * check docu for naming conventions:
    */

    private static enum StartingLocs {
        RIGHT,
        MID_1,
        MID_12,
        LEFT;
    }
    
    /*
     * the first place you'll score in a match.
     * Kept separate so that ppl dont accidentally choose a first reefLoc that doesn't have a traj going form [startingLoc] -> [reefLoc]
     */
    public static enum FirstScoringLocs {
        REEF_1(StartingLocs.MID_1, ""),
        REEF_2(StartingLocs.LEFT, ""),
        REEF_3(StartingLocs.LEFT, ""),

        REEF_10(StartingLocs.RIGHT, ""),
        REEF_11(StartingLocs.RIGHT, ""),
        REEF_12(StartingLocs.MID_12, "");

        public Pair<StartingLocs, String> m_startAndTraj;

        /*
         * for possible starting scoring positions
         */
        private FirstScoringLocs(StartingLocs startLoc, String trajName) {
            m_startAndTraj = new Pair<TrajsAndLocs.StartingLocs,String>(startLoc, trajName);
        }
    }

    public static enum ScoringLocs {
        REEF_1(FirstScoringLocs.REEF_1),
        REEF_2(FirstScoringLocs.REEF_2),
        REEF_3(FirstScoringLocs.REEF_3),
        REEF_4,
        REEF_5,
        REEF_6,
        REEF_7,
        REEF_8,
        REEF_9,
        REEF_10(FirstScoringLocs.REEF_10),
        REEF_11(FirstScoringLocs.REEF_11),
        REEF_12(FirstScoringLocs.REEF_12),
        INVALID_LOC;

        public FirstScoringLocs m_pairedLoc;

        private ScoringLocs() {
            m_pairedLoc = null;
        }

        /*
         * for the positions that *could* be starting locations
         */
        private ScoringLocs(FirstScoringLocs sameLoc) {
            m_pairedLoc = sameLoc;
        }

        public static ScoringLocs getSameLoc(FirstScoringLocs startLoc) {
            switch (startLoc) {
                case REEF_1:
                    return REEF_1;
                case REEF_2:
                    return REEF_2;
                case REEF_3:
                    return REEF_3;
                case REEF_10:
                    return REEF_10;
                case REEF_11:
                    return REEF_11;
                case REEF_12:
                    return REEF_12;
                default:
                    return INVALID_LOC; //TODO: find better fail case T^T
            }
        }
    }

    public static enum CS {
        /* pov from driver station */
        CS_RIGHT,
        CS_LEFT;
    }

    //TODO: fill out values frfr
    public static class Trajectories {
        public HashMap<Pair<ScoringLocs, CS>, String> m_toCSTrajMap;
        public HashMap<Pair<CS, ScoringLocs>, String> m_toRTrajMap;

        public Trajectories() {
            m_toCSTrajMap = new HashMap<Pair<ScoringLocs, CS>, String>();
            m_toRTrajMap = new HashMap<Pair<CS, ScoringLocs>, String>();
        }

        public void configureTrajectories() {
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_1, CS.CS_LEFT), "");
        }
    }
}
