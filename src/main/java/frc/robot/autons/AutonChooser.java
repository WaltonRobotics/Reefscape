package frc.robot.autons;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autons.TrajsAndLocs.*;
import frc.robot.autons.WaltAutonFactory.AutonCycle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.subsystems.Swerve; 

public class AutonChooser {
    private static final Swerve drivetrain = TunerConstants.createDrivetrain();
    private static final AutoFactory autoFactory = drivetrain.createAutoFactory();
    private static WaltAutonFactory autonFactory = new WaltAutonFactory(autoFactory);
    
        public static enum NumCycles {
            CYCLE_1(1),
            CYCLE_2(2),
            CYCLE_3(3),
            CYCLE_4(4);
    
            public int m_cycles;
    
            private NumCycles(int cycles){
                m_cycles = cycles;
            }
            @Override
            public String toString() {
                return String.valueOf(m_cycles);
            }
        }
        //TODO: change suppliers to consumers/listeners
        private static EnumMap<StartingLocs, String> startingLocMap = new EnumMap<>(TrajsAndLocs.StartingLocs.class);
        public static SendableChooser<StartingLocs> startingPositionChooser = new SendableChooser<StartingLocs>();
        //private static Supplier<StartingLocs> startLocChosen = () -> startingPositionChooser.getSelected();
        public static StartingLocs startLocChosen = startingPositionChooser.getSelected();
    
        private static EnumMap<ReefLocs, String> firstScoringMap = new EnumMap<>(TrajsAndLocs.ReefLocs.class);
        public static SendableChooser<ReefLocs> firstScoringChooser = new SendableChooser<ReefLocs>();
        //private static Supplier<ReefLocs> firstScoringChosen = () -> firstScoringChooser.getSelected();
        public static ReefLocs firstScoringChosen = firstScoringChooser.getSelected();
       
        private static EnumMap<HPStation, String> hpStationMap = new EnumMap<>(TrajsAndLocs.HPStation.class);
        public static SendableChooser<HPStation> hpStationChooser = new SendableChooser<HPStation>();
        //private static Supplier<HPStation> hpStationChosen = () -> hpStationChooser.getSelected();
        public static HPStation hpStationChosen = hpStationChooser.getSelected();
    
        private static EnumMap<ReefLocs, String> hpToReefMap = new EnumMap<>(TrajsAndLocs.ReefLocs.class); 
        public static SendableChooser<ReefLocs> hpToReefChooser = new SendableChooser<ReefLocs>();
        //private static Supplier<ReefLocs> hpToReefChosen = () -> hpToReefChooser.getSelected();
        public static ReefLocs hpToReefChosen = hpToReefChooser.getSelected();
    
        private static EnumMap<HPStation, String> reefToHPMap = new EnumMap<>(TrajsAndLocs.HPStation.class); 
        public static SendableChooser<HPStation> reefToHPChooser = new SendableChooser<HPStation>();
        //private static Supplier<HPStation> reefToHPChosen = () -> reefToHPChooser.getSelected();
        public static HPStation reefToHPChosen = reefToHPChooser.getSelected();
    
        private static EnumMap<NumCycles, String> cyclesMap = new EnumMap<>(NumCycles.class);
        public static SendableChooser<NumCycles> cyclesChooser = new SendableChooser<NumCycles>();
        //private static Supplier<NumCycles> cyclesChosen = () -> cyclesChooser.getSelected();
        public static NumCycles cyclesChosen = cyclesChooser.getSelected();

        private static EnumMap<EleHeight, String> startingHeightMap = new EnumMap<>(Elevator.EleHeight.class);
        public static SendableChooser<EleHeight> startingHeightChooser = new SendableChooser<EleHeight>();
        //private static Supplier<EleHeight> eleHeightChosen = () -> eleHeightChooser.getSelected();
        public static EleHeight startingHeightChosen = startingHeightChooser.getSelected();
    
        private static EnumMap<EleHeight, String> eleHeightMap = new EnumMap<>(Elevator.EleHeight.class);
        public static SendableChooser<EleHeight> eleHeightChooser = new SendableChooser<EleHeight>();
        //private static Supplier<EleHeight> eleHeightChosen = () -> eleHeightChooser.getSelected();
        public static EleHeight eleHeightChosen = eleHeightChooser.getSelected();
    
        static{
            SmartDashboard.putData("starting position chooser", startingPositionChooser);   
            SmartDashboard.putData("human player station chooser", hpStationChooser);
            SmartDashboard.putData("number of cycles chooser", cyclesChooser);
            SmartDashboard.putData("starting height chooser", startingHeightChooser);
        }
    
    
    
        public static void assignNumCycles(NumCycles numCycles, String description){
            cyclesMap.put(numCycles, description);
            cyclesChooser.addOption(description, numCycles);
        }
    
        public static void assignStartingPosition(StartingLocs startingLoc, String description){
            startingLocMap.put(startingLoc, description);
            startingPositionChooser.addOption(description, startingLoc);
        }
    
        public static void assignFirstScoring(ReefLocs scoringLoc, String description){
            firstScoringMap.put(scoringLoc, description);
            firstScoringChooser.addOption(description, scoringLoc);
        }
    
        public static void assignHPStation(HPStation hpstation, String description){
            hpStationMap.put(hpstation, description);
            hpStationChooser.addOption(description, hpstation);
        }
    
        public static void assignReefScoring(ReefLocs reefLocs, String description){
            hpToReefMap.put(reefLocs, description);
            hpToReefChooser.addOption(description, reefLocs);
        }
    
        public static void assignReeftoHPScoring(HPStation hpStation, String description){
            reefToHPMap.put(hpStation, description);
            reefToHPChooser.addOption(description, hpStation);
        }

        public static void assignStartingHeight(EleHeight startingHeight, String description){
            startingHeightMap.put(startingHeight, description);
            startingHeightChooser.addOption(description, startingHeight);
        }
    
        public static void assignEleHeight(EleHeight eleHeight, String description){
            eleHeightMap.put(eleHeight, description);
            eleHeightChooser.addOption(description, eleHeight);
        }
    
    
    
        public static void setDefaultAuton(StartingLocs scoringLoc){
            startingPositionChooser.setDefaultOption("default (mid)", scoringLoc);
        }
    
        public static void setDefaultHPStation(HPStation hpStation){
            hpStationChooser.setDefaultOption("default(left)", hpStation);
        }
    
    
    
        /**
         * depending on certain starting location, displays the optimal path for said starting + scoring path
         */
        public static void chooseFirstScoring(){
            firstScoringChooser = new SendableChooser<ReefLocs>();
    
            if(startLocChosen.equals(TrajsAndLocs.StartingLocs.MID)){
    
                for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalMidStartCycles.size(); i++) {
                    assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalMidStartCycles.get(i), 
                        TrajsAndLocs.ReefLocs.OptimalMidStartCycles.get(i).toString());
                }
    
            }else if (startLocChosen.equals(TrajsAndLocs.StartingLocs.LEFT)){
    
                for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.size(); i++) {
                    assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i), 
                        TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i).toString());
                }
    
            } else{
                
                for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalRightStartCycles.size(); i++) {
                    assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i), 
                        TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i).toString());
                }
    
            }   
            SmartDashboard.putData("first scoring chooser", firstScoringChooser);
            SmartDashboard.updateValues(); //FINALLY
    
        }
    
        /**
         * given that an HP Station is selected, creates NT that shows all possible(optimal?) routes to certain reefs
         */
        public static void chooseHPtoReef(String description, HPStation hpChosen){
            hpToReefChooser = new SendableChooser<ReefLocs>();
            
            if(hpChosen.equals(TrajsAndLocs.HPStation.HP_RIGHT)){ 
                for(int i = 0; i < TrajsAndLocs.Trajectories.HPToReefTrajs.size() / 2; i++){
                    assignReefScoring(TrajsAndLocs.ReefLocs.OptimalRightHPCycles.get(i), 
                    TrajsAndLocs.ReefLocs.OptimalRightHPCycles.get(i).toString());
                }
            } else {
                for(int i = 0; i < TrajsAndLocs.Trajectories.HPToReefTrajs.size() / 2; i++){
                    assignReefScoring(TrajsAndLocs.ReefLocs.OptimalLeftHPCycles.get(i), 
                    TrajsAndLocs.ReefLocs.OptimalLeftHPCycles.get(i).toString());
                }
            }
            SmartDashboard.putData(description, hpToReefChooser);
            SmartDashboard.updateValues();
        }
        
        /**
        * given that a reef was selected (after going to HP), creates the possible HP options for that selected reef
        */
        public static void chooseReefToHP(String description){
            reefToHPChooser = new SendableChooser<HPStation>();
            
            if(hpToReefChosen != null){
                if(hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_A)){
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");
    
                } else if(hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_B)){
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");
    
                } else if(hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_C)){
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");
    
                } else if(hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_D)){
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right"); 
    
                } else if(hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_E)){
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");
    
                } else if(hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_F)){
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");
    
                } else if(hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_G)){
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");
    
                } else if(hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_H)){
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");
    
                } else if(hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_I)){
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
    
                } else if(hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_J)){
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
    
                } else if(hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_K)){
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
    
                } else{
                    assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
    
                }
            }
    
            SmartDashboard.putData(description, reefToHPChooser);
            SmartDashboard.updateValues();
        }
    
        /**
         * choose ele height
         */
        public static void chooseEleHeight(String description){
            eleHeightChooser = new SendableChooser<EleHeight>();

            AutonChooser.assignEleHeight(Elevator.EleHeight.L1, "L1");
            AutonChooser.assignEleHeight(Elevator.EleHeight.L2, "L2");
            AutonChooser.assignEleHeight(Elevator.EleHeight.L3, "L3");
            AutonChooser.assignEleHeight(Elevator.EleHeight.L4, "L4");
    
            SmartDashboard.putData(description, eleHeightChooser);
        }
    
        /**
         * depending on how many cycles you choose, will display each cycle's choosers: hp to reef, reef to hp, and ele height
         */
        public static void cycleIterations(){
            NumCycles selectedCycles = cyclesChosen;
    
            if(cyclesChosen != null){
    
                int numIterations = selectedCycles.m_cycles;
    
                for(int i = 1; i <= numIterations; i++){ 
    
                    chooseEleHeight("ele height chooser " + i);
    
                    if(reefToHPChosen != null){
                        
                        chooseHPtoReef("HP to Reef Chooser " + i, reefToHPChosen);
    
                        chooseReefToHP("Reef to HP Chooser " + i);
    
                        // if(hpToReefChosen.get() != null && !(scoringLocationArray.contains(hpToReefChosen.get()))){
                        //     scoringLocationArray.add(hpToReefChosen.get());
                        // }
    
                    } else{
                        chooseHPtoReef("HP to Reef Chooser " + i, hpStationChosen);
                        chooseReefToHP("Reef to HP Chooser " + i);
    
                        // if(hpToReefChosen.get() != null && !(scoringLocationArray.contains(hpToReefChosen.get()))){
                        //     scoringLocationArray.add(hpToReefChosen.get());
                        // }
                    }
                     
                }  
    
                SmartDashboard.updateValues();
            }
        }
    
        // public static void getSelectedValues(){
    
        //     StartingLocs startingPosition = startingPositionChooser.getSelected();
        //     ReefLocs firstScoringLoc = firstScoringChooser.getSelected();
        //     HPStation hpStation = hpStationChooser.getSelected();
    
        //         // for(int i = 0; i < scoringLocationArray.size(); i++){
        //         //     System.out.println(scoringLocationArray.get(i));
        //         // }
    
        //         // if(!(hpStationArray.contains(reefToHPChosen.get()))){
        //         //     hpStationArray.add(reefToHPChooser.getSelected());
        //         // }
    
        //         // for(int i = 0; i < hpStationArray.size(); i++){
        //         //     System.out.println(hpStationArray.get(i));
        //         // }
        // }

        
    
        public static StartingLocs getChosenStart(){
            return startLocChosen;
        }
    
        public static ReefLocs getChosenFirstReef(){
            return firstScoringChosen;
        }
    
        public static HPStation getChosenFirstHP(){
            return hpStationChosen;
        }

        public static EleHeight getStartingHeight(){
            return startingHeightChosen;
        }
    
        public static ArrayList<AutonCycle> getCycles(){
    
            ArrayList<AutonCycle> arrayCycles = new ArrayList<AutonCycle>();
            AutonCycle cycle;
    
            for(int i = 1; i <= cyclesChosen.m_cycles; i++){
                cycle = autonFactory.new AutonCycle(
                    hpToReefChosen, eleHeightChosen, reefToHPChosen);

                arrayCycles.add(cycle);
            }

            return arrayCycles;
        }

}