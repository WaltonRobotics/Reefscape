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
        LEFT(new Pose2d(Meters.of(7.25), Meters.of(5.73), Rotation2d.fromDegrees(0)), "Start_left"),
        MID(new Pose2d(Meters.of(7.25), Meters.of(4.025), Rotation2d.fromDegrees(0)), "Start_mid"),
        RIGHT(new Pose2d(Meters.of(7.25), Meters.of(2.42), Rotation2d.fromDegrees(0)), "Start_right");

        public final Pose2d pose;
        public final String str;
        private StartingLocs(Pose2d _pose,String _str ) {
            pose = _pose;
            str = _str;

        }
    }

    public static enum ReefLocs {
        REEF_A(new Pose2d(Meters.of(2.91), Meters.of(4.22), Rotation2d.fromRadians(0)), "A"),
        REEF_B(new Pose2d(Meters.of(2.88), Meters.of(3.89), Rotation2d.fromRadians(0)), "B"),
        REEF_C(new Pose2d(Meters.of(3.55), Meters.of(2.73), Rotation2d.fromRadians(1.01)), "C"),
        REEF_D(new Pose2d(Meters.of(3.82), Meters.of(2.57), Rotation2d.fromRadians(1.01)), "D"),
        REEF_E(new Pose2d(Meters.of(5.16), Meters.of(2.52), Rotation2d.fromRadians(2.11)), "E"),
        REEF_F(new Pose2d(Meters.of(5.41), Meters.of(2.72), Rotation2d.fromRadians(2.11)), "F"),
        REEF_G(new Pose2d(Meters.of(6.10), Meters.of(3.86), Rotation2d.fromRadians(3.14)), "G"),
        REEF_H(new Pose2d(Meters.of(6.10), Meters.of(4.17), Rotation2d.fromRadians(3.14)), "H"),
        REEF_I(new Pose2d(Meters.of(5.43), Meters.of(5.28), Rotation2d.fromRadians(-2.12)), "I"),
        REEF_J(new Pose2d(Meters.of(5.10), Meters.of(5.42), Rotation2d.fromRadians(-2.12)), "J"),
        REEF_K(new Pose2d(Meters.of(3.87), Meters.of(5.42), Rotation2d.fromRadians(-1.03)), "K"),
        REEF_L(new Pose2d(Meters.of(3.54), Meters.of(5.30), Rotation2d.fromRadians(-1.03)), "L");


        public final Pose2d pose;
        public final String str;
        private ReefLocs(Pose2d _pose, String _str) {
            pose = _pose;
            str = _str;
        }
        
        /* 
         * enums used in autonchooser
         * list of best possible next locs given the start cycle u were just at
         */
        public static HashMap<StartingLocs, ArrayList<ReefLocs>> optimalPathsByMatchStartLocation = new HashMap<StartingLocs, ArrayList<ReefLocs>>();
        static {
            ArrayList<ReefLocs> leftStartReefLocs = new ArrayList<ReefLocs>();
            leftStartReefLocs.add(REEF_H);
            leftStartReefLocs.add(REEF_I);
            leftStartReefLocs.add(REEF_J);
            leftStartReefLocs.add(REEF_K);
            optimalPathsByMatchStartLocation.put(StartingLocs.LEFT, leftStartReefLocs);

            ArrayList<ReefLocs> midStartReefLocs = new ArrayList<ReefLocs>();
            midStartReefLocs.add(REEF_E);
            midStartReefLocs.add(REEF_F);
            midStartReefLocs.add(REEF_G);
            midStartReefLocs.add(REEF_H);
            midStartReefLocs.add(REEF_I);
            midStartReefLocs.add(REEF_J);
            optimalPathsByMatchStartLocation.put(StartingLocs.MID, midStartReefLocs);

            ArrayList<ReefLocs> rightStartReefLocs = new ArrayList<ReefLocs>();
            rightStartReefLocs.add(REEF_D);
            rightStartReefLocs.add(REEF_E);
            rightStartReefLocs.add(REEF_F);
            rightStartReefLocs.add(REEF_G);
            optimalPathsByMatchStartLocation.put(StartingLocs.RIGHT, rightStartReefLocs);
        }
        public static HashMap<HPStation, ArrayList<ReefLocs>> optimalPathsByHPStation = new HashMap<HPStation, ArrayList<ReefLocs>>();
        static {
            ArrayList<ReefLocs> leftHPStationReefLocs = new ArrayList<ReefLocs>();
            leftHPStationReefLocs.add(REEF_A);
            leftHPStationReefLocs.add(REEF_B);
            leftHPStationReefLocs.add(REEF_G);
            leftHPStationReefLocs.add(REEF_H);
            leftHPStationReefLocs.add(REEF_I);
            leftHPStationReefLocs.add(REEF_J);
            leftHPStationReefLocs.add(REEF_K);
            leftHPStationReefLocs.add(REEF_L);
            optimalPathsByHPStation.put(HPStation.HP_LEFT, leftHPStationReefLocs);

            ArrayList<ReefLocs> rightHPStationReefLocs = new ArrayList<ReefLocs>();
            rightHPStationReefLocs.add(REEF_A);
            rightHPStationReefLocs.add(REEF_B);
            rightHPStationReefLocs.add(REEF_C);
            rightHPStationReefLocs.add(REEF_D);
            rightHPStationReefLocs.add(REEF_E);
            rightHPStationReefLocs.add(REEF_F);
            rightHPStationReefLocs.add(REEF_G);
            rightHPStationReefLocs.add(REEF_H);
            optimalPathsByHPStation.put(HPStation.HP_RIGHT, rightHPStationReefLocs);
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