/* 
 * alexandra and sohan are dummies and this formatting is ABSOLUTELY COOKED WHAT THE hallways 
 * I DIDNT RAISE MY CHILDREN LIKE THIS
 * *crash out*
 */

 package frc.robot.autons;

 import java.lang.reflect.Array;
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
 
     public static SendableChooser<StartingLocs> startingPositionChooser = new SendableChooser<StartingLocs>();
     public static StartingLocs startLocChosen = TrajsAndLocs.StartingLocs.MID;
     
     public static SendableChooser<ReefLocs> firstScoringChooser = new SendableChooser<ReefLocs>();
     public static ReefLocs firstScoringChosen = firstScoringChooser.getSelected();
        
     public static SendableChooser<HPStation> hpStationChooser = new SendableChooser<HPStation>();
     public static HPStation hpStationChosen = hpStationChooser.getSelected();
     
     public static ArrayList<SendableChooser<ReefLocs>> hpToReefChoosers = new ArrayList<SendableChooser<ReefLocs>>();
     public static ReefLocs hpToReefChosen = TrajsAndLocs.ReefLocs.REEF_I;
     
     public static SendableChooser<HPStation> reefToHPChooser = new SendableChooser<HPStation>();
     public static HPStation reefToHPChosen = reefToHPChooser.getSelected();
     
     public static SendableChooser<NumCycles> cyclesChooser = new SendableChooser<NumCycles>();
     public static NumCycles cyclesChosen = cyclesChooser.getSelected();
 
     public static SendableChooser<EleHeight> startingHeightChooser = new SendableChooser<EleHeight>();
     public static EleHeight startingHeightChosen = startingHeightChooser.getSelected();
     
     public static SendableChooser<EleHeight> eleHeightChooser = new SendableChooser<EleHeight>();
     public static EleHeight eleHeightChosen = eleHeightChooser.getSelected();
     
     static {
         SmartDashboard.putData("starting position chooser", startingPositionChooser);   
         SmartDashboard.putData("human player station chooser", hpStationChooser);
         SmartDashboard.putData("number of cycles chooser", cyclesChooser);
         SmartDashboard.putData("starting height chooser", startingHeightChooser);
     }
     
     public static void addChoosers() {
         // want to display all choosers
         for (int i = 1; i <= 4; i++) {
            hpToReefChoosers.add(new SendableChooser<ReefLocs>());
         }
     }
     
     public static void addNumCycles(NumCycles numCycles, String description){
         cyclesChooser.addOption(description, numCycles);
     }
     
     public static void addStartingPosition(StartingLocs startingLoc, String description){
         startingPositionChooser.addOption(description, startingLoc);
     }
     
     public static void addFirstScoring(ReefLocs scoringLoc, String description){
         firstScoringChooser.addOption(description, scoringLoc);
     }
     
     public static void addHPStation(HPStation hpstation, String description){
         hpStationChooser.addOption(description, hpstation);
     }
     
     public static void addReefScoring(ReefLocs reefLocs, String description, SendableChooser<ReefLocs> chooser){
         chooser.addOption(description, reefLocs);
     }
     
     public static void addReeftoHPScoring(HPStation hpStation, String description, SendableChooser<HPStation> chooser){
         chooser.addOption(description, hpStation);
     }
 
     public static void addStartingHeight(EleHeight startingHeight, String description){
         startingHeightChooser.addOption(description, startingHeight);
     }
     
     public static void addEleHeight(EleHeight eleHeight, String description){
         eleHeightChooser.addOption(description, eleHeight);
     }
     
     public static void setDefaultAuton(StartingLocs scoringLoc){
         startingPositionChooser.setDefaultOption("mid", scoringLoc);
     }
     
     public static void setDefaultHPStation(HPStation hpStation){
         hpStationChooser.setDefaultOption("human player left", hpStation);
     }
     
     /**
      * depending on certain starting location, displays the optimal path for said starting + scoring path
      */
     public static void chooseFirstScoring(){
         firstScoringChooser = new SendableChooser<ReefLocs>();
         SmartDashboard.updateValues();
     
         if(startLocChosen.equals(TrajsAndLocs.StartingLocs.MID)){
     
             for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalMidStartCycles.size(); i++) {
                 addFirstScoring(TrajsAndLocs.ReefLocs.OptimalMidStartCycles.get(i), 
                     TrajsAndLocs.ReefLocs.OptimalMidStartCycles.get(i).toString());
             }
     
         }else if (startLocChosen.equals(TrajsAndLocs.StartingLocs.LEFT)){
     
             for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.size(); i++) {
                 addFirstScoring(TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i), 
                     TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i).toString());
             }
         } else {
             for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalRightStartCycles.size(); i++) {
                 addFirstScoring(TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i), 
                     TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i).toString());
             }
     
         }   
         SmartDashboard.putData("first scoring chooser", firstScoringChooser);
     }
     
     /**
      * given that an HP Station is selected, creates NT that shows all possible(optimal?) routes to certain reefs
      */
     public static void chooseHPtoReef(String description, HPStation hpChosen, int index){
         addChoosers();
         SendableChooser<ReefLocs> hpToReefChooser = hpToReefChoosers.get(index);
         hpToReefChooser = new SendableChooser<ReefLocs>();
         SmartDashboard.updateValues();
             
         if(hpChosen.equals(TrajsAndLocs.HPStation.HP_RIGHT)){ 
             for(int i = 0; i < TrajsAndLocs.Trajectories.HPToReefTrajs.size() / 2; i++){
                 addReefScoring(TrajsAndLocs.ReefLocs.OptimalRightHPCycles.get(i), 
                 TrajsAndLocs.ReefLocs.OptimalRightHPCycles.get(i).toString(), 
                 hpToReefChooser);
             }
         } else {
             for(int i = 0; i < TrajsAndLocs.Trajectories.HPToReefTrajs.size() / 2; i++){
                 addReefScoring(TrajsAndLocs.ReefLocs.OptimalLeftHPCycles.get(i), 
                 TrajsAndLocs.ReefLocs.OptimalLeftHPCycles.get(i).toString(),
                 hpToReefChooser);
             }
         }
         SmartDashboard.putData(description, hpToReefChooser);
     }
         
     /**
     * given that a reef was selected (after going to HP), creates the possible HP options for that selected reef
     */
     public static void chooseReefToHP(int index){
         hpToReefChosen = hpToReefChoosers.get(index).getSelected();
         // addChoosers();
         // SendableChooser<HPStation> reefToHPChooser = reefToHPChoosers.get(index);
         reefToHPChooser = new SendableChooser<HPStation>();
         SmartDashboard.updateValues();
             
         if (hpToReefChosen != null) {
             if (hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_A)) {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left", reefToHPChooser);
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right", reefToHPChooser);
     
             } else if (hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_B)) {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left", reefToHPChooser);
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right", reefToHPChooser);
     
             } else if (hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_C)) {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right", reefToHPChooser);
     
             } else if (hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_D)) {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right", reefToHPChooser); 
     
             } else if (hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_E)) {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right", reefToHPChooser);
     
             } else if (hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_F)) {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right",reefToHPChooser);
     
             } else if (hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_G)) {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left", reefToHPChooser);
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right", reefToHPChooser);
     
             } else if (hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_H)) {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left", reefToHPChooser);
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right", reefToHPChooser);
     
             } else if (hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_I)) {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left", reefToHPChooser);
     
             } else if (hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_J)) {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left", reefToHPChooser);
     
             } else if (hpToReefChosen.equals(TrajsAndLocs.ReefLocs.REEF_K)) {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left", reefToHPChooser);
     
             } else {
                 addReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left", reefToHPChooser);
     
             }
         }
     
         SmartDashboard.putData("Reef to HP Chooser " + index, reefToHPChooser);
     }
     
     /**
      * choose ele height
      */
     public static void chooseEleHeight(String description) {
         eleHeightChooser = new SendableChooser<EleHeight>();
 
         addEleHeight(Elevator.EleHeight.L1, "L1");
         addEleHeight(Elevator.EleHeight.L2, "L2");
         addEleHeight(Elevator.EleHeight.L3, "L3");
         addEleHeight(Elevator.EleHeight.L4, "L4");
 
         SmartDashboard.putData(description, eleHeightChooser);
     }
     
     /**
      * depending on how many cycles you choose, will display each cycle's choosers: hp to reef, reef to hp, and ele height
      */
     public static void cycleIterations() {
 
         System.out.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
         System.out.println("Cycle Iteration");
         System.out.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
 
         NumCycles selectedCycles = cyclesChosen;
     
         if (cyclesChosen != null) {
             int numIterations = selectedCycles.m_cycles;
     
             for(int i = 1; i <= numIterations; i++) { 
                 chooseEleHeight("ele height chooser " + i);
     
                 if(reefToHPChosen != null){    
                     chooseHPtoReef("HP to Reef Chooser " + i, reefToHPChosen, i);
                     chooseReefToHP(i);
     
                 } else if (hpStationChosen != null) {
                     chooseHPtoReef("HP to Reef Chooser " + i, hpStationChosen, i);
                     chooseReefToHP(i);
     
                 } else {
                     // TODO: send error!!!
                 }    
             }
         }
     }
     
         // gives the selected values for array lists
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
 }