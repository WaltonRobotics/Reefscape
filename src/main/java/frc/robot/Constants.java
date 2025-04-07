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
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.util.AllianceFlipUtil;

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
             .withStatorCurrentLimit(110)
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
         public static final int kTopBeamBreakChannel = 1;
         public static final int kBotBeamBreakChannel = 0;
 
         // TODO: ask alexandra and hrehaan what these are for (???)
         // public static final double kGearRatio = 1; //for arm spinup and coral intake
         // public static final double kArmGearRatio = 2; //for arm pivot
 
         public static final double kCoralSpeed = 1;
 
         public static final TalonFXConfiguration kCoralMotorTalonFXConfiguration = new TalonFXConfiguration()
             .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));
     }

     public class IntakeK {
         //motorized intake schtuff
         public static final String kLogTab = "IntakeSubsys";
         /* HIGHKEY TEMPORARY          | TODO: change when we assign the canid to actual canid :D 
          * ALSO APPLIES TO BEAM BREAK | TODO: change when we figure out beambreak channel :D
         */
         public static final int kIntakeMotorCANID = 32; 
         public static final int kBeamBreakChannel = 2;
        
         // dont see why the configs would need to be different than what the coral intake has
         public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration()
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
         public static final double kParallelToGroundRotations = -0.64;
         public static final double kClimbRotations = -0.9;
         public static final double kDefaultPos = -0.04; 
 
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
        public static final int kServoChannel = 0;

        public static final double kLatchUnlockedPos = 0.85;
        public static final double kLatchLockedPos = 0;

        public static final double kGearRatio = 50/12;
        public static final Distance kSpoolRadius = Inches.of(0.9175);  // TODO: ask banks if the thing we considered a spool is a spool?

        public static final double kP = 5;
        public static final double kI = 0;
        public static final double kS = 0.05;
        public static final double kV = 0.58403;
        public static final double kA = 0;
        public static final double kG = 0.57989; 

        private static final double kPClimb = 100;
        private static final double kSClimb = 0.5;
        private static final double kVClimb = 0.9;


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
        public static final CurrentLimitsConfigs kClimbCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(175)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(55)
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
        private static final Slot1Configs kSlot1Configs = new Slot1Configs()
            .withKS(kSClimb) 
            .withKV(kVClimb) 
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKP(kPClimb);
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
            .withSlot1(kSlot1Configs)
            .withSoftwareLimitSwitch(kSoftwareLimitConfigs)
            .withMotorOutput(kMotorOutputConfigs);

        public static final TalonFXConfiguration kRearTalonFXConfig = new TalonFXConfiguration()
            .withCurrentLimits(kCurrentLimitConfigs)
            .withMotorOutput(kMotorOutputConfigs);
     }

     public class RobotK {
        public static final String kLogTab = "SuperStructure";
        // TODO: get a real distance from the reef for this
        public static final double kRobotCenterDistanceFromReef = Units.inchesToMeters(-17);
        public static final double kRobotScoringOffset = Units.inchesToMeters(2.75); // positive left robot, measured 4/1/2025
        public static final Transform2d kTransformReefPoseToRobotPosition = new Transform2d(kRobotCenterDistanceFromReef, kRobotScoringOffset, Rotation2d.fromDegrees(0));
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
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(30))
        );
        public static final String kLowerRightCamSimVisualName = "LowerRightVisionEstimation";
    }

    public static class FieldK {
        public static final double kFieldLengthMeters = 17.548;
        public static final double kFieldWidthMeters = 8.052;
        public static final double kStartLineXMeters = Units.inchesToMeters(299.438); // measured from the inside of starting line

        public static final Field2d kDestinationPosesField2d = new Field2d();

        public static boolean inField(Pose2d pose) {
            return (
                pose.getX() > 0 && pose.getX() < kFieldLengthMeters
                && pose.getY() > 0 && pose.getY() < kFieldWidthMeters
            );
        }

        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
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

        public static double kXKP = 7;
        public static double kYKP = 7;
        public static double kThetaKP = 10;
        
        private static final TrapezoidProfile.Constraints kAutoAlignConstraints = new TrapezoidProfile.Constraints(3, 12);
        public static final PIDController kAutoAlignXController = new PIDController(kXKP, 0, 0);
        public static final PIDController kAutoAlignYController = new PIDController(kYKP, 0, 0);
        public static final PIDController kAutoAlignThetaController = new PIDController(kThetaKP, 0, 0);
        // we need to do this so the PIDController works properly
        static {
            kAutoAlignThetaController.enableContinuousInput(-Math.PI, Math.PI);
        }

        public static final double kTranslationTolerance = 0.005; // meters
        public static final double kFieldRotationTolerance = 1; // degrees

        public static final double kIntermediatePoseDistance = -Units.inchesToMeters(4);

        // TODO: these will really need tuning
        public static final double kXMaxVelocity = 3; // m/s
        public static final double kXMaxAccel = 3; // m/s^2

        public static final double kYMaxVelocity = 3; // m/s
        public static final double kYMaxAccel = 3; // m/s^2

        public static final double kThetaMaxVelocity = 45; // deg/s
        public static final double kThetaMaxAccel = 45; // deg/s^2
        
        /** <p>Arbitrary number to control how much a difference in rotation should affect tag selection. Higher means more weight
         * <p> 0 means rotation difference has no weight, negative will literally bias it against tags that have more similar rotations */
        public static final double kRotationWeight = 0.2;
        
        /**<p>[0, 1]. Controls weight of predicted future pose in velocity weighted tag selection.
         * <p> 0 is no weight, 1 is 100% weight (no input from current state).
         * <p> Impacts {@link #kCurrentWeight} */
        public static final double kFutureWeight = 0.2;
        /**<p>Equal to 1 - {@link #kFutureWeight}. Controls weight of the current pose in velocity weighted tag selection */
        public static final double kCurrentWeight = 1 - kFutureWeight;
        public static final double kFutureDelta = 0.1; // s

        public static final double kMaxXYSpeedAutoalign = 1.5;
    }
}
