package frc.robot.autons;

import java.util.HashMap;

import edu.wpi.first.math.Pair;

public abstract class TrajsAndLocs {
    /* 
     * check docu for naming conventions:
    */

    public static enum StartingLocs {
        RIGHT,
        MID_G,
        MID_H,
        LEFT;
    }
    
    /*
     * the first place you'll score in a match.
     * Kept separate so that ppl dont accidentally choose a first reefLoc that doesn't have a traj going form [startingLoc] -> [reefLoc]
     */
    public static enum FirstScoringLocs {
        REEF_H(StartingLocs.MID_H, "mid_H"),
        REEF_I(StartingLocs.LEFT, "left_I"),
        REEF_J(StartingLocs.LEFT, "left_J"),

        REEF_E(StartingLocs.RIGHT, "right_E"),
        REEF_F(StartingLocs.RIGHT, "right_F"),
        REEF_G(StartingLocs.MID_G, "mid_G");

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
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_G, CS.CS_LEFT), "HP_G_left");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_H, CS.CS_LEFT), "HP_H_left");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_I, CS.CS_LEFT), "HP_I_left");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_J, CS.CS_LEFT), "HP_J_left");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_K, CS.CS_LEFT), "HP_K_left");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_L, CS.CS_LEFT), "HP_L_left");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_A, CS.CS_LEFT), "HP_A_left");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_B, CS.CS_LEFT), "HP_B_left");

            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_A, CS.CS_RIGHT), "HP_A_right");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_B, CS.CS_RIGHT), "HP_B_right");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_C, CS.CS_RIGHT), "HP_C_right");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_D, CS.CS_RIGHT), "HP_D_right");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_E, CS.CS_RIGHT), "HP_E_right");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_F, CS.CS_RIGHT), "HP_F_right");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_G, CS.CS_RIGHT), "HP_G_right");
            m_toCSTrajMap.put(new Pair<ScoringLocs, CS>(ScoringLocs.REEF_H, CS.CS_RIGHT), "HP_H_right");

            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_LEFT, ScoringLocs.REEF_G), "R_left_G");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_LEFT, ScoringLocs.REEF_H), "R_left_H");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_LEFT, ScoringLocs.REEF_I), "R_left_I");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_LEFT, ScoringLocs.REEF_J), "R_left_J");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_LEFT, ScoringLocs.REEF_K), "R_left_K");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_LEFT, ScoringLocs.REEF_L), "R_left_L");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_LEFT, ScoringLocs.REEF_A), "R_left_A");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_LEFT, ScoringLocs.REEF_B), "R_left_B");

            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_RIGHT, ScoringLocs.REEF_A), "R_right_A");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_RIGHT, ScoringLocs.REEF_B), "R_right_B");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_RIGHT, ScoringLocs.REEF_C), "R_right_C");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_RIGHT, ScoringLocs.REEF_D), "R_right_D");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_RIGHT, ScoringLocs.REEF_E), "R_right_E");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_RIGHT, ScoringLocs.REEF_F), "R_right_F");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_RIGHT, ScoringLocs.REEF_G), "R_right_G");
            m_toRTrajMap.put(new Pair<CS, ScoringLocs>(CS.CS_RIGHT, ScoringLocs.REEF_H), "R_right_H");
        }
    }
}
