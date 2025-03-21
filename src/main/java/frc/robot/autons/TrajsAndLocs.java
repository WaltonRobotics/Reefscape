package frc.robot.autons;

import static edu.wpi.first.units.Units.Meters;

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
        SUPER_LEFT("Start_Left"),
        LEFT("Start_Left"),
        MID_G("Start_Mid"),
        MID_H("Start_Mid"),
        SUPER_RIGHT("Start_Mid"),
        RIGHT("Start_Right");

        public final String str;
        private StartingLocs(String _str ) {
            str = _str;
        }
    }

    public static enum ReefLocs {
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
        private ReefLocs(String _str) {
            str = _str;
        }
        
        /* 
         * enums used in autonchooser
         * list of best possible next locs given the start cycle u were just at
         */
        public static ArrayList<ReefLocs> OptimalLeftStartCycles = new ArrayList<>();
        static {
            OptimalLeftStartCycles.add(REEF_H);
            OptimalLeftStartCycles.add(REEF_I);         
            OptimalLeftStartCycles.add(REEF_J);
        }
        public static ArrayList<ReefLocs> OptimalSuperLeftStartCycles = new ArrayList<>();
        static {
            OptimalLeftStartCycles.add(REEF_K);
        }
        public static ArrayList<ReefLocs> OptimalMidGStartCycles = new ArrayList<>();
        static {
            OptimalMidGStartCycles.add(REEF_E);
            OptimalMidGStartCycles.add(REEF_F);         
            OptimalMidGStartCycles.add(REEF_G);
        }
        public static ArrayList<ReefLocs> OptimalMidHStartCycles = new ArrayList<>();
        static {
            OptimalMidGStartCycles.add(REEF_H);
            OptimalMidGStartCycles.add(REEF_I);
            OptimalMidGStartCycles.add(REEF_J);
        }
        public static ArrayList<ReefLocs> OptimalRightStartCycles = new ArrayList<>();
        static {
            OptimalRightStartCycles.add(REEF_E);         
            OptimalRightStartCycles.add(REEF_F);
            OptimalRightStartCycles.add(REEF_G);
        }
        public static ArrayList<ReefLocs> OptimalSuperRightStartCycles = new ArrayList<>();
        static {
            OptimalRightStartCycles.add(REEF_D);
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
        HP_LEFT(new Pose2d(Meters.of(1.45), Meters.of(6.89), Rotation2d.fromRadians(-0.90)), "left"),
        HP_RIGHT(new Pose2d(Meters.of(1.53), Meters.of(1.16), Rotation2d.fromRadians(0.92)), "right");
        
        public final Pose2d pose;
        public final String str;
        private HPStation(Pose2d _pose, String _str) {
            pose = _pose;
            str = _str;
        }
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
        public static HashMap<StartingLocs, String> PushingTrajs = new HashMap<>();

        static {
            // fill in maps here
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.LEFT, ReefLocs.REEF_H), "Start_Left_H");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.LEFT, ReefLocs.REEF_I), "Start_Left_I");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.LEFT, ReefLocs.REEF_J), "Start_Left_J");

            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.SUPER_LEFT, ReefLocs.REEF_K), "Start_Left_K");

            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID_G, ReefLocs.REEF_E), "Start_Mid_E");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID_G, ReefLocs.REEF_F), "Start_Mid_F");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID_G, ReefLocs.REEF_G), "Start_Mid_G");

            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID_H, ReefLocs.REEF_H), "Start_Mid_H");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID_H, ReefLocs.REEF_I), "Start_Mid_I");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID_H, ReefLocs.REEF_J), "Start_Mid_J");

            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.SUPER_RIGHT, ReefLocs.REEF_D), "Start_Right_D");

            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_E), "Start_Right_E");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_F), "Start_Right_F");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_G), "Start_Right_G");

            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_A, HPStation.HP_LEFT), "A_Left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_B, HPStation.HP_LEFT), "B_Left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_G, HPStation.HP_LEFT), "G_Left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_H, HPStation.HP_LEFT), "H_Left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_I, HPStation.HP_LEFT), "I_Left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_J, HPStation.HP_LEFT), "J_Left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_K, HPStation.HP_LEFT), "K_Left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_L, HPStation.HP_LEFT), "L_Left");

            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_A, HPStation.HP_RIGHT), "A_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_B, HPStation.HP_RIGHT), "B_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_C, HPStation.HP_RIGHT), "C_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_D, HPStation.HP_RIGHT), "D_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_E, HPStation.HP_RIGHT), "E_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_F, HPStation.HP_RIGHT), "F_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_G, HPStation.HP_RIGHT), "G_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_H, HPStation.HP_RIGHT), "H_Right");

            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_A), "Left_A");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_B), "Left_B");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_G), "Left_G");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_H), "Left_H");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_I), "Left_I");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_J), "Left_J");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_K), "Left_K");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_L), "Left_L");

            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_A), "Right_A");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_B), "Right_B");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_C), "Right_C");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_D), "Right_D");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_E), "Right_E");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_F), "Right_F");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_G), "Right_G");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_H), "Right_H");

            PushingTrajs.put(StartingLocs.LEFT, "Pushing_Left");
            PushingTrajs.put(StartingLocs.RIGHT, "Pushing_Right");
            PushingTrajs.put(StartingLocs.MID_G, "Pushing_Mid_G");
            PushingTrajs.put(StartingLocs.MID_H, "Pushing_Mid_H");
            PushingTrajs.put(StartingLocs.SUPER_LEFT, "Pushing_Super_Left");
            PushingTrajs.put(StartingLocs.SUPER_RIGHT, "Pushing_Super_Right");
        }
    }
}