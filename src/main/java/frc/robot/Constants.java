package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import frc.robot.autons.TrajsAndLocs.ReefLocs;

import org.photonvision.simulation.SimCameraProperties;


public class Constants {
     /* general */
     public static final  boolean kDebugLoggingEnabled = true;
 
     public static final double kRumbleIntensity = 1.0;
     public static final double kRumbleTimeoutSecs = 0.5;
     public static class AlgaeK {
         public static final String kLogTab = "AlgaeSubsys";
         
         public static final int kWristCANID = 12;
         public static final int kIntakeCANID = 13;
 
         // motor configuration section
         // wrist motor
         public static final int kWristGearRatio = 40;   //TODO: check if still accurate
         public static final int kWristSensorToMechanismRatio = kWristGearRatio;
         public static final int kAngleTolerance = 3; //DUMMY VALUE
 
         public static final double kWristMMVelo = 1;
         public static final double kWristMMAccel = 25;
         public static final double kWristMMJerk = 300;
         public static final double kWristMMVeloSlow = 0.65;
         public static final double kWristMMAccelSlow = 10;
 
         private static final CurrentLimitsConfigs kWristCurrentLimitConfigs = new CurrentLimitsConfigs()
             .withStatorCurrentLimit(20)
             .withSupplyCurrentLimit(20)
             .withStatorCurrentLimitEnable(true);
         private static final FeedbackConfigs kWristFeedbackConfigs = new FeedbackConfigs()
             .withSensorToMechanismRatio(kWristSensorToMechanismRatio);
         private static final MotionMagicConfigs kWristMagicConfigs = new MotionMagicConfigs()
             .withMotionMagicCruiseVelocity(RotationsPerSecond.of(kWristMMVelo))
             .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(kWristMMAccel))
             .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(kWristMMJerk));
         private static final Slot0Configs kWristSlot0Configs = new Slot0Configs()
             .withKS(0.25)
             .withKV(4)
             .withKA(0)
             .withKP(10) // gg ez
             .withKI(0)
             .withKD(0);
         public static final MotorOutputConfigs kWristMotorOutputConfigs = new MotorOutputConfigs()
             .withNeutralMode(NeutralModeValue.Brake);
         public static final TalonFXConfiguration kWristConfiguration = new TalonFXConfiguration()
             .withCurrentLimits(kWristCurrentLimitConfigs)
             .withFeedback(kWristFeedbackConfigs)
             .withMotionMagic(kWristMagicConfigs)
             .withSlot0(kWristSlot0Configs)
             .withMotorOutput(kWristMotorOutputConfigs);
         
         // intake motor
         public static final int kIntakeGearRatio = 2;
         public static final int kIntakeSensorToMechanismRatio = kIntakeGearRatio;
         public static final double kHasAlgaeCurrent = 10; //DUMMY VALUE
         private static final CurrentLimitsConfigs kIntakeCurrentLimitConfigs = new CurrentLimitsConfigs()
             .withStatorCurrentLimit(100)
             .withSupplyCurrentLimit(50)
             .withStatorCurrentLimitEnable(true);
         private static final FeedbackConfigs kIntakeFeedbackConfigs = new FeedbackConfigs()
             .withSensorToMechanismRatio(kIntakeSensorToMechanismRatio);
         private static final MotionMagicConfigs kIntakeMagicConfigs = new MotionMagicConfigs()
             .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
             .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
             .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
         private static final Slot0Configs kIntakeSlot0Configs = new Slot0Configs()
             .withKS(0.25)
             .withKV(0.12)
             .withKA(0.01)
             .withKP(60)
             .withKI(0)
             .withKD(0.5);
         public static final MotorOutputConfigs kIntakeMotorOutputConfigs = new MotorOutputConfigs()
             .withInverted(InvertedValue.Clockwise_Positive);
         public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration()
             .withCurrentLimits(kIntakeCurrentLimitConfigs)
             .withFeedback(kIntakeFeedbackConfigs)
             .withMotionMagic(kIntakeMagicConfigs)
             .withSlot0(kIntakeSlot0Configs)
             .withMotorOutput(kIntakeMotorOutputConfigs);
 
         // state machine work -- make real values!
         public static final Current kIntakeCurrentSpikeThreshold = Amps.of(20);
         public static final Current kShootCurrentDropThreshold = Amps.of(5);
         public static final double kMaxAngleDeg = 120; //TODO: check if this is still accurate - from zero position to algae pickup
     }
 
     public class Coralk {
         // coral things
         public static final String kLogTab = "CoralSubsys";
         public static final int kCoralMotorCANID = 30; 
         public static final int kTopBeamBreakChannel = 0;
         public static final int kBotBeamBreakChannel = 1;
 
         // TODO: ask alexandra and hrehaan what these are for (???)
         // public static final double kGearRatio = 1; //for arm spinup and coral intake
         // public static final double kArmGearRatio = 2; //for arm pivot
 
         public static final double kCoralSpeed = 1;
 
         public static final TalonFXConfiguration kCoralMotorTalonFXConfiguration = new TalonFXConfiguration()
             .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));
     }
 
     public final class FingerK {
         // finger things
         public static final String kLogTab = "FingerSubsys";
 
         public static final int kFingerMotorCANID = 31;
 
         public static final double kMaxAngleRotations = 0;
         public static final double kMinAngleRotations = -1;
         public static final double kParallelToGroundRotations = -0.7;
         public static final double kClimbRotations = -0.88;
         public static final double kDefaultPos = -0.08;   
 
         private static final MotorOutputConfigs kMotorOutputConfig = new MotorOutputConfigs()
             .withInverted(InvertedValue.Clockwise_Positive)
             .withNeutralMode(NeutralModeValue.Brake);
 
         private static final CurrentLimitsConfigs kCurrentLimitConfig = new CurrentLimitsConfigs()
             .withStatorCurrentLimit(5)
             .withStatorCurrentLimitEnable(true)
             .withSupplyCurrentLimit(3)
             .withSupplyCurrentLimitEnable(true);        
         private static final ClosedLoopGeneralConfigs kClosedLoopGeneralConfig = new ClosedLoopGeneralConfigs()
             .withContinuousWrap(false);
 
         private static final Slot0Configs kSlot0Config = new Slot0Configs()
             .withKP(100)
             .withKS(1)
             .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
         private static final ExternalFeedbackConfigs kExternalFeedbackConfig = new ExternalFeedbackConfigs()
             .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.Quadrature)
             .withQuadratureEdgesPerRotation(4096)
             .withRotorToSensorRatio(1)
             .withSensorPhase(SensorPhaseValue.Opposed)
             .withSensorToMechanismRatio(2);        
         public static final SoftwareLimitSwitchConfigs kSoftLimitEnabledConfig = new SoftwareLimitSwitchConfigs()
             .withForwardSoftLimitEnable(true)
             .withForwardSoftLimitThreshold(kMaxAngleRotations)
             .withReverseSoftLimitEnable(true)
             .withReverseSoftLimitThreshold(kMinAngleRotations);
 
         public static final SoftwareLimitSwitchConfigs kSoftLimitSwitchDisabledConfig = new SoftwareLimitSwitchConfigs()
             .withForwardSoftLimitEnable(false)
             .withReverseSoftLimitEnable(false);
 
         private static final CommutationConfigs kCommutationConfig = new CommutationConfigs()
             .withMotorArrangement(MotorArrangementValue.Brushed_DC)
             .withBrushedMotorWiring(BrushedMotorWiringValue.Leads_A_and_B);
         
 
         public static final TalonFXSConfiguration kTalonFXSConfig = new TalonFXSConfiguration()
             .withMotorOutput(kMotorOutputConfig)
             .withCurrentLimits(kCurrentLimitConfig)
             .withClosedLoopGeneral(kClosedLoopGeneralConfig)
             .withSlot0(kSlot0Config)
             .withExternalFeedback(kExternalFeedbackConfig)
             .withSoftwareLimitSwitch(kSoftLimitEnabledConfig)
             .withCommutation(kCommutationConfig);
     }
 
     public final class ElevatorK {
         public static final String kLogTab = "EleSubsys";
 
         public static final int kFrontCANID = 10;
         public static final int kBackCANID = 11;
 
         public static final double kGearRatio = 50/12;
         public static final Distance kSpoolRadius = Inches.of(0.9175);  // TODO: ask banks if the thing we considered a spool is a spool?
 
         public static final double kP = 5;
         public static final double kI = 0;
         public static final double kS = 0.05;
         public static final double kV = 0.58403;
         public static final double kA = 0;
         public static final double kG = 0.57989; 
 
         public static final Mass kCarriageMassKg = Pounds.of(5);
         public static final Distance kMinimumHeight = Feet.of(0);
         public static final Distance kMaximumHeight = Meters.of(15);
         public static final Distance kStartingHeightMeters = Feet.of(0);
         public static final double kTolerancePulleyRotations = 0.1;
         //SensorToMechanismRatio = kGearRatio
 
         public static LinearVelocity rotationsToMetersVel(AngularVelocity rotations){
             return kSpoolRadius.per(Second).times(rotations.in(RadiansPerSecond));
         }
 
         public static Angle metersToRotation(Distance meters){
         return Radians.of(meters.in(Meters) / (2 * Math.PI * kSpoolRadius.in(Meters)));
         }
 
         public static Distance rotationsToMeters(Angle rotations) {
             return Meters.of(rotations.in(Radians) * 2 * Math.PI * kSpoolRadius.in(Meters));
         }
 
         public static AngularVelocity metersToRotationVel(LinearVelocity meters){
             return RadiansPerSecond.of(meters.in(MetersPerSecond)/kSpoolRadius.in(Meters));
         }
 
         public static AngularVelocity metersToRotationVel(double metersPerSecond){
             return metersToRotationVel(LinearVelocity.ofBaseUnits(metersPerSecond, MetersPerSecond));
         }
 
         private static final CurrentLimitsConfigs kCurrentLimitConfigs = new CurrentLimitsConfigs()
             .withStatorCurrentLimit(100)
             .withStatorCurrentLimitEnable(true)
             .withSupplyCurrentLimit(75)
             .withSupplyCurrentLimitEnable(true);
         private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
             .withSensorToMechanismRatio(kGearRatio);
         private static final Slot0Configs kSlot0Configs = new Slot0Configs()
             .withKS(kS) 
             .withKV(kV) 
             .withGravityType(GravityTypeValue.Elevator_Static)
             .withKP(kP)
             .withKI(kI) 
             .withKG(kG);
         private static final MotorOutputConfigs kMotorOutputConfigs = new MotorOutputConfigs()
             .withNeutralMode(NeutralModeValue.Brake)
             .withInverted(InvertedValue.CounterClockwise_Positive);
         public static final SoftwareLimitSwitchConfigs kSoftwareLimitConfigs = new SoftwareLimitSwitchConfigs()
             .withForwardSoftLimitEnable(true)
             .withForwardSoftLimitThreshold(12.8975)  // true hard 12.9849854
             .withReverseSoftLimitEnable(true)
             .withReverseSoftLimitThreshold(0);
         public static final SoftwareLimitSwitchConfigs kSoftLimitSwitchDisabledConfig = new SoftwareLimitSwitchConfigs();
         private static final MotionMagicConfigs kMotionMagicConfigs = new MotionMagicConfigs()
             .withMotionMagicCruiseVelocity(20)
             .withMotionMagicAcceleration(100)
             .withMotionMagicJerk(0);
 
 
         public static final TalonFXConfiguration kFrontTalonFXConfig = new TalonFXConfiguration()
             .withCurrentLimits(kCurrentLimitConfigs)
             .withFeedback(kFeedbackConfigs)
             .withMotionMagic(kMotionMagicConfigs)
             .withSlot0(kSlot0Configs)
             .withSoftwareLimitSwitch(kSoftwareLimitConfigs)
             .withMotorOutput(kMotorOutputConfigs);
 
         public static final TalonFXConfiguration kRearTalonFXConfig = new TalonFXConfiguration()
             .withCurrentLimits(kCurrentLimitConfigs)
             .withMotorOutput(kMotorOutputConfigs);
 
     }
 
     public class RobotK {
         public static final String kLogTab = "SuperStructure";
     }

     public static class VisionK {
        public static final SimCameraProperties kEleForwardCamSimProps = new SimCameraProperties();
        static {
            // See3CAM_24CUG
            kEleForwardCamSimProps.setCalibration(1920, 1080, Rotation2d.fromDegrees(128.2));
            kEleForwardCamSimProps.setCalibError(0.35, 0.10);
            kEleForwardCamSimProps.setFPS(35);
            kEleForwardCamSimProps.setAvgLatencyMs(30);
            kEleForwardCamSimProps.setLatencyStdDevMs(15);
        }

        public static final SimCameraProperties kEleRearCamSimProps = new SimCameraProperties();
        static {
            // See3CAM_24CUG
            kEleRearCamSimProps.setCalibration(1920, 1080, Rotation2d.fromDegrees(128.2));
            kEleRearCamSimProps.setCalibError(0.35, 0.10);
            kEleRearCamSimProps.setFPS(35);
            kEleRearCamSimProps.setAvgLatencyMs(30);
            kEleRearCamSimProps.setLatencyStdDevMs(15);
        }

        public static final SimCameraProperties kLowerRightCamSimProps = new SimCameraProperties();
        static {
            // Arducam OV9281
            kLowerRightCamSimProps.setCalibration(1280, 720, Rotation2d.fromDegrees(100));
            kLowerRightCamSimProps.setCalibError(0.35, 0.10);
            kLowerRightCamSimProps.setFPS(45);
            kLowerRightCamSimProps.setAvgLatencyMs(25);
            kLowerRightCamSimProps.setLatencyStdDevMs(15);
        }
        

        public static final String kElevatorForwardsCamName = "EleFrontCam";
        public static final Transform3d kElevatorForwardsCamRoboToCam = new Transform3d(
            Units.inchesToMeters(8.238), Units.inchesToMeters(4.81), Units.inchesToMeters(32), 
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(40), Units.degreesToRadians(-10))
        );
        public static final String kElevatorForwardsCamSimVisualName = "EleForwardsVisionEstimation";

        public static final String kLowerRightCamName = "LowerRightCam";
        public static final Transform3d kLowerRightCamRoboToCam = new Transform3d(
            Units.inchesToMeters(10.943), Units.inchesToMeters(-9.769), Units.inchesToMeters(8.161),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(40))
        );
        public static final String kLowerRightCamSimVisualName = "LowerRightVisionEstimation";
    }

    public static class FieldK {
        public static final double kFieldLengthMeters = Units.inchesToMeters(687.876);
        public static final double kFieldWidthMeters = Units.inchesToMeters(317);
        public static final double kStartLineXMeters = Units.inchesToMeters(299.438); // measured from the inside of starting line

        public static boolean inField(Pose2d pose) {
            return (
                pose.getX() > 0 && pose.getX() < kFieldLengthMeters
                && pose.getY() > 0 && pose.getY() < kFieldWidthMeters
            );
        }

        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        private static final List<AprilTag> kRedReefTags = List.of(
            kTagLayout.getTags().get(5),
            kTagLayout.getTags().get(6), 
            kTagLayout.getTags().get(7), 
            kTagLayout.getTags().get(8), 
            kTagLayout.getTags().get(9), 
            kTagLayout.getTags().get(10)
        );
        private static final List<AprilTag> kBlueReefTags = List.of(
            kTagLayout.getTags().get(16),
            kTagLayout.getTags().get(17), 
            kTagLayout.getTags().get(18), 
            kTagLayout.getTags().get(19), 
            kTagLayout.getTags().get(20),
            kTagLayout.getTags().get(21)
        );
        public static final AprilTagFieldLayout kRedSpeakerTagLayout = new AprilTagFieldLayout(kRedReefTags, kTagLayout.getFieldLength(), kTagLayout.getFieldWidth());
        public static final AprilTagFieldLayout kBlueSpeakerTagLayout = new AprilTagFieldLayout(kBlueReefTags, kTagLayout.getFieldLength(), kTagLayout.getFieldWidth());

        public static class Reef {
            public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
            public static final double faceToZoneLineMeters = Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line
            public static final Pose2d[] centerFaces = new Pose2d[6]; // starting face is the one parallel-ly facing the starting line and then moves counterclockwise
            public static final Map<ReefLocs, Map<ReefHeight, Pose3d>> branchPose3ds = new HashMap<>(); // Map reef locations to Pose3ds. uses branchPositions behind the scenes
            public static final List<Map<ReefHeight, Pose3d>> branchPositions = new ArrayList<>();
            public static final Map<ReefLocs, Pose2d> reefLocationToIdealRobotPoseMap = new HashMap<ReefLocs, Pose2d>();
            public static final List<ReefLocs> leftReefs = new ArrayList<>();
            public static final List<ReefLocs> rightReefs = new ArrayList<>();
            
            static {
                // Initialize faces
                centerFaces[0] =
                    new Pose2d(
                        Units.inchesToMeters(209.489),
                        Units.inchesToMeters(158.502),
                        Rotation2d.fromDegrees(0));
                centerFaces[1] = 
                    new Pose2d(
                        Units.inchesToMeters(193.116),
                        Units.inchesToMeters(186.858),
                        Rotation2d.fromDegrees(60));
                centerFaces[2] =
                    new Pose2d(
                        Units.inchesToMeters(160.373),
                        Units.inchesToMeters(186.857),
                        Rotation2d.fromDegrees(120));
                centerFaces[3] =
                    new Pose2d(
                        Units.inchesToMeters(144.003),
                        Units.inchesToMeters(158.500),
                        Rotation2d.fromDegrees(180));
                centerFaces[4] =
                    new Pose2d(
                        Units.inchesToMeters(160.375),
                        Units.inchesToMeters(130.144),
                        Rotation2d.fromDegrees(-120));
                centerFaces[5] =
                    new Pose2d(
                        Units.inchesToMeters(193.118),
                        Units.inchesToMeters(130.145),
                        Rotation2d.fromDegrees(-60));
                
                // Initialize branch positions
                for (int face = 0; face < 6; face++) {
                    Map<ReefHeight, Pose3d> rightBranches = new HashMap<>();
                    Map<ReefHeight, Pose3d> leftBranches = new HashMap<>();
                    for (var reefLvl : ReefHeight.values()) {
                        Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                        double adjustXMeters = Units.inchesToMeters(30.738);
                        double adjustYMeters = Units.inchesToMeters(6.469);

                        rightBranches.put(
                            reefLvl,
                            new Pose3d(
                                new Translation3d(
                                    poseDirection.transformBy(new Transform2d(adjustXMeters, adjustYMeters, new Rotation2d())).getX(),
                                    poseDirection.transformBy(new Transform2d(adjustXMeters, adjustYMeters, new Rotation2d())).getY(),
                            reefLvl.m_heightMeters),
                            new Rotation3d(0, Units.degreesToRadians(reefLvl.m_pitchDegs),
                            poseDirection.getRotation().getRadians())));
                        leftBranches.put(
                            reefLvl,
                            new Pose3d(
                                new Translation3d(
                                    poseDirection.transformBy(new Transform2d(adjustXMeters, -adjustYMeters, new Rotation2d())).getX(),
                                    poseDirection.transformBy(new Transform2d(adjustXMeters, -adjustYMeters, new Rotation2d())).getY(),
                            reefLvl.m_heightMeters),
                            new Rotation3d(0, Units.degreesToRadians(reefLvl.m_pitchDegs), poseDirection.getRotation().getRadians())));
                    }
                    branchPositions.add(rightBranches);
                    branchPositions.add(leftBranches);
                }

                branchPose3ds.put(ReefLocs.REEF_A, branchPositions.get(1));
                branchPose3ds.put(ReefLocs.REEF_B, branchPositions.get(0));
                branchPose3ds.put(ReefLocs.REEF_C, branchPositions.get(11));
                branchPose3ds.put(ReefLocs.REEF_D, branchPositions.get(10));
                branchPose3ds.put(ReefLocs.REEF_E, branchPositions.get(9));
                branchPose3ds.put(ReefLocs.REEF_F, branchPositions.get(8));
                branchPose3ds.put(ReefLocs.REEF_G, branchPositions.get(7));
                branchPose3ds.put(ReefLocs.REEF_H, branchPositions.get(6));
                branchPose3ds.put(ReefLocs.REEF_I, branchPositions.get(5));
                branchPose3ds.put(ReefLocs.REEF_J, branchPositions.get(4));
                branchPose3ds.put(ReefLocs.REEF_K, branchPositions.get(3));
                branchPose3ds.put(ReefLocs.REEF_L, branchPositions.get(2));

                // TODO: get a real distance from the reef for this
                double distanceFromReef = Units.inchesToMeters(17);
                double leftRightOffset = -Units.inchesToMeters(1.25); // positive left robot
                Transform2d transformToRobotPosition = new Transform2d(distanceFromReef, leftRightOffset, Rotation2d.fromDegrees(180));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_A, branchPose3ds.get(ReefLocs.REEF_A).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_B, branchPose3ds.get(ReefLocs.REEF_B).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_C, branchPose3ds.get(ReefLocs.REEF_C).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_D, branchPose3ds.get(ReefLocs.REEF_D).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_E, branchPose3ds.get(ReefLocs.REEF_E).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_F, branchPose3ds.get(ReefLocs.REEF_F).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_G, branchPose3ds.get(ReefLocs.REEF_G).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_H, branchPose3ds.get(ReefLocs.REEF_H).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_I, branchPose3ds.get(ReefLocs.REEF_I).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_J, branchPose3ds.get(ReefLocs.REEF_J).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_K, branchPose3ds.get(ReefLocs.REEF_K).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
                reefLocationToIdealRobotPoseMap.put(ReefLocs.REEF_L, branchPose3ds.get(ReefLocs.REEF_L).get(ReefHeight.L1).toPose2d()
                    .transformBy(transformToRobotPosition));
            }

            public enum ReefHeight {
                L4(Units.inchesToMeters(72), -90),
                L3(Units.inchesToMeters(47.625), -35),
                L2(Units.inchesToMeters(31.875), -35),
                L1(Units.inchesToMeters(18), 0);

                public double m_heightMeters;
                public double m_pitchDegs;
            
                ReefHeight(double height, double pitch) {
                  m_heightMeters = height;
                  m_pitchDegs = pitch; // in degrees
                }
            }
        }
    
        public static class CS {
            public static final Pose2d leftCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units. inchesToMeters(291.176),
                        Rotation2d.fromDegrees(90 - 144.011));
            public static final Pose2d rightCenterFace =
                new Pose2d(
                    Units.inchesToMeters(33.526),
                    Units.inchesToMeters(25.824),
                    Rotation2d.fromDegrees(144.011 - 90));
        }
        public static class Processor {
            public static final Pose2d centerFace = new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
        }
        public static class Barge {
            public static final Translation2d farCage = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
            public static final Translation2d middleCage = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
            public static final Translation2d closeCage = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));
            // Measured from floor to bottom of cage
            public static final Distance deepHeight = Meters.of(Units.inchesToMeters(3.125));
            public static final Distance shallowHeight = Meters.of(Units.inchesToMeters(30.125));
        }
    }

    public static class AutoAlignmentK {
        public static final double kSideToSideTolerance = 0.0025; // meters
        public static final double kFieldRotationTolerance = 1; // degrees

        // TODO: these will really need tuning
        public static final double kRXMaxVelocity = 3; // m/s
        public static final double kRXMaxAccel = 3; // m/s^2

        public static final double kRYMaxVelocity = 3; // m/s
        public static final double kRYMaxAccel = 3; // m/s^2

        public static final double kThetaMaxVelocity = 45; // deg/s
        public static final double kThetaMaxAccel = 45; // deg/s^2
        
    }
}
