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
    public static enum StartingLocs {
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
        REEF_G;

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
            OptimalMidStartCycles.add(REEF_E);
            OptimalMidStartCycles.add(REEF_F);         
            OptimalMidStartCycles.add(REEF_G);
            OptimalMidStartCycles.add(REEF_H);
            OptimalMidStartCycles.add(REEF_I);
            OptimalMidStartCycles.add(REEF_J);
        }
        public static ArrayList<ReefLocs> OptimalRightStartCycles = new ArrayList<>();
        static {
            OptimalRightStartCycles.add(REEF_D);
            OptimalRightStartCycles.add(REEF_E);         
            OptimalRightStartCycles.add(REEF_F);
            OptimalRightStartCycles.add(REEF_G);
        }

        /*
         * just to make looping throuugh and displaying all the different ReefLocs easier? will check if good ltr!
         */
        public static ArrayList<ReefLocs> OptimalLeftHPCycles = new ArrayList<>();
        static{
            OptimalLeftHPCycles.add(REEF_A);
            OptimalLeftHPCycles.add(REEF_B);
            OptimalLeftHPCycles.add(REEF_G);
            OptimalLeftHPCycles.add(REEF_H);
            OptimalLeftHPCycles.add(REEF_I);
            OptimalLeftHPCycles.add(REEF_J);
            OptimalLeftHPCycles.add(REEF_K);
            OptimalLeftHPCycles.add(REEF_L);
        }
        public static ArrayList<ReefLocs> OptimalRightHPCycles = new ArrayList<>();
        static{
            OptimalRightHPCycles.add(REEF_A);
            OptimalRightHPCycles.add(REEF_B);
            OptimalRightHPCycles.add(REEF_C);
            OptimalRightHPCycles.add(REEF_D);
            OptimalRightHPCycles.add(REEF_E);
            OptimalRightHPCycles.add(REEF_F);
            OptimalRightHPCycles.add(REEF_G);
            OptimalRightHPCycles.add(REEF_H);
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

    public static class Trajectories {
        public static HashMap<Pair<StartingLocs, ReefLocs>, String> StartToReefTrajs = new HashMap<>();
        public static HashMap<Pair<ReefLocs, HPStation>, String> ReefToHPTrajs = new HashMap<>();
        public static HashMap<Pair<HPStation, ReefLocs>, String> HPToReefTrajs = new HashMap<>();

        static {
            // fill in maps here
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.LEFT, ReefLocs.REEF_H) , "Start_left_H");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.LEFT, ReefLocs.REEF_I) , "Start_left_I");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.LEFT, ReefLocs.REEF_J) , "Start_left_J");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.LEFT, ReefLocs.REEF_K) , "Start_left_K");

            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID, ReefLocs.REEF_E) , "Start_mid_E");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID, ReefLocs.REEF_F) , "Start_mid_F");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID, ReefLocs.REEF_G) , "Start_mid_G");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID, ReefLocs.REEF_H) , "Start_mid_H");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID, ReefLocs.REEF_I) , "Start_mid_I");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID, ReefLocs.REEF_J) , "Start_mid_J");

            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_D) , "Start_right_D");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_E) , "Start_right_E");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_F) , "Start_right_F");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_G) , "Start_right_G");

            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_A, HPStation.HP_LEFT) , "HP_A_left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_B, HPStation.HP_LEFT) , "HP_B_left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_G, HPStation.HP_LEFT) , "HP_G_left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_H, HPStation.HP_LEFT) , "HP_H_left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_I, HPStation.HP_LEFT) , "HP_I_left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_J, HPStation.HP_LEFT) , "HP_J_left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_K, HPStation.HP_LEFT) , "HP_K_left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_L, HPStation.HP_LEFT) , "HP_L_left");

            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_A, HPStation.HP_RIGHT), "HP_A_right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_B, HPStation.HP_RIGHT), "HP_B_right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_C, HPStation.HP_RIGHT), "HP_C_right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_D, HPStation.HP_RIGHT), "HP_D_right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_E, HPStation.HP_RIGHT), "HP_E_right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_F, HPStation.HP_RIGHT), "HP_F_right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_G, HPStation.HP_RIGHT), "HP_G_right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_H, HPStation.HP_RIGHT), "HP_H_right");

            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_A) , "R_left_A");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_B) , "R_left_B");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_G) , "R_left_G");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_H) , "R_left_H");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_I) , "R_left_I");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_J) , "R_left_J");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_K) , "R_left_K");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_L) , "R_left_L");

            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_A) , "R_right_A");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_B) , "R_right_B");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_C) , "R_right_C");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_D) , "R_right_D");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_E) , "R_right_E");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_F) , "R_right_F");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_G) , "R_right_G");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_H) , "R_right_H");
        }
    }
}