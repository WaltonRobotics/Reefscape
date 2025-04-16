package frc.robot.autons;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;

public abstract class TrajsAndLocs {
    /* 
     * possible starting locs
     * liable to add more
     * 
     * POV: driver station
     */
    public static enum StartingLocs {
        // TODO: fill out the Real Values
        SUPER_LEFT("Left"),
        LEFT("Left"),
        MID_G("Mid"),
        MID_H("Mid"),
        SUPER_RIGHT("Mid"),
        RIGHT("Right");

        public final String pathName;
        public final String name;
        private StartingLocs(String _str ) {
            name = _str;
            pathName = "Start_" + _str;
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

        public Pose2d getIdealScoringPose() {
            var potentialPose = FieldConstants.kReefRobotLocationPoseMap.get(this);
            if (potentialPose != null) {
                return potentialPose;
            }
            return new Pose2d();
        }

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
        public static HashMap<Pair<StartingLocs, ReefLocs>, String> StartToReefShortTrajs = new HashMap<>();
        public static HashMap<Pair<ReefLocs, HPStation>, String> ReefToHPTrajs = new HashMap<>();
        public static HashMap<Pair<HPStation, ReefLocs>, String> HPToReefTrajs = new HashMap<>();
        public static HashMap<Pair<HPStation, ReefLocs>, String> HPToReefShortTrajs = new HashMap<>();
        public static HashMap<StartingLocs, String> PushingTrajs = new HashMap<>();

        static {
            // fill in maps here
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.LEFT, ReefLocs.REEF_I), "Start_Left_I");
            StartToReefShortTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.LEFT, ReefLocs.REEF_I), "Start_Left_I_short");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.LEFT, ReefLocs.REEF_J), "Start_Left_J");
            StartToReefShortTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.LEFT, ReefLocs.REEF_J), "Start_Left_J_short");

            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.SUPER_LEFT, ReefLocs.REEF_K), "Start_Left_K");
            StartToReefShortTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.SUPER_LEFT, ReefLocs.REEF_K), "Start_Left_K_short");

            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID_G, ReefLocs.REEF_G), "Start_Mid_G");
            StartToReefShortTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID_G, ReefLocs.REEF_G), "Start_Mid_G_short");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID_H, ReefLocs.REEF_H), "Start_Mid_H");
            StartToReefShortTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.MID_H, ReefLocs.REEF_H), "Start_Mid_H_short");

            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.SUPER_RIGHT, ReefLocs.REEF_D), "Start_Right_D");
            StartToReefShortTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.SUPER_RIGHT, ReefLocs.REEF_D), "Start_Right_D_short");

            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_E), "Start_Right_E");
            StartToReefShortTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_E), "Start_Right_E_short");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_F), "Start_Right_F");
            StartToReefShortTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_F), "Start_Right_F_short");
            StartToReefTrajs.put(new Pair<StartingLocs, ReefLocs>(StartingLocs.RIGHT, ReefLocs.REEF_G), "Start_Right_G");

            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_A, HPStation.HP_LEFT), "A_Left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_B, HPStation.HP_LEFT), "B_Left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_J, HPStation.HP_LEFT), "J_Left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_K, HPStation.HP_LEFT), "K_Left");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_L, HPStation.HP_LEFT), "L_Left");

            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_A, HPStation.HP_RIGHT), "A_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_B, HPStation.HP_RIGHT), "B_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_C, HPStation.HP_RIGHT), "C_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_D, HPStation.HP_RIGHT), "D_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_E, HPStation.HP_RIGHT), "E_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_F, HPStation.HP_RIGHT), "F_Right");
            ReefToHPTrajs.put(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_G, HPStation.HP_RIGHT), "Madtown_G_Right");

            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_A), "Left_A");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_B), "Left_B");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_J), "Left_J");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_K), "Left_K");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_L), "Left_L");
            HPToReefShortTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_A), "Left_A_short");
            HPToReefShortTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_B), "Left_B_short");
            HPToReefShortTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_J), "Left_J_short");
            HPToReefShortTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_K), "Left_K_short");
            HPToReefShortTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_LEFT, ReefLocs.REEF_L), "Left_L_short");

            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_A), "Right_A");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_B), "Right_B");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_C), "Right_C");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_D), "Right_D");
            HPToReefTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_E), "Right_E");
            HPToReefShortTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_A), "Right_A_short");
            HPToReefShortTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_B), "Right_B_short");
            HPToReefShortTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_C), "Right_C_short");
            HPToReefShortTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_D), "Right_D_short");
            HPToReefShortTrajs.put(new Pair<HPStation, ReefLocs>(HPStation.HP_RIGHT, ReefLocs.REEF_E), "Right_E_short");
        }
    }
}