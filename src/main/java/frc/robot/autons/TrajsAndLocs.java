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

    public static enum ScoringLocs {
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
            //TODO: like actually do. later tho :3
        }
    }
}
