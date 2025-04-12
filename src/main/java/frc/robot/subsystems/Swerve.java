package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.Choreo;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleArrayLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.robot.Robot;
import frc.robot.Constants.AutoAlignmentK;
import frc.robot.generated.TunerConstants;
/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    public static GenericEntry nte_autoAlignmentKPX;
    public static GenericEntry nte_autoAlignmentKPY;
    public static GenericEntry nte_autoAlignmentKPTheta;
    
    // static {
    //     nte_autoAlignmentKPX = Shuffleboard.getTab("Teleoperated")
    //         .add("KPX", kXKP)
    //         .withWidget(BuiltInWidgets.kTextView)
    //         .getEntry();

    //     nte_autoAlignmentKPY = Shuffleboard.getTab("Teleoperated")
    //         .add("KPY", kYKP)
    //         .withWidget(BuiltInWidgets.kTextView)
    //         .getEntry();

    //     nte_autoAlignmentKPTheta = Shuffleboard.getTab("Teleoperated")
    //         .add("KPTheta", kThetaKP)
    //         .withWidget(BuiltInWidgets.kTextView)
    //         .getEntry();
    // }

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* TODO: gotta tune */
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.Position);
    private final PIDController m_pathXController = new PIDController(3, 0, 0);
    private final PIDController m_pathYController = new PIDController(3, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(7, 0, 0);

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.FieldCentric swreq_drive = new SwerveRequest.FieldCentric()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    private final SwerveRequest.SwerveDriveBrake swreq_brake = new SwerveRequest.SwerveDriveBrake();

    /* wheel radius characterization schtuffs */
    // public final DoubleSupplier m_gyroYawRadsSupplier = () -> 360 - Units.degreesToRadians(getPigeon2().getYaw().getValueAsDouble());
    // private final SlewRateLimiter m_omegaLimiter = new SlewRateLimiter(0.5);
    // private final SwerveRequest.RobotCentric m_characterizationReq = new SwerveRequest.RobotCentric()
	// 	.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // private final double m_characterizationSpeed = 1.5;

    /* loggin' */
    private final DoubleLogger log_lastGyro = WaltLogger.logDouble("Swerve", "lastGyro");
    private final DoubleLogger log_avgWheelPos = WaltLogger.logDouble("Swerve", "avgWheelPos");
    private final DoubleLogger log_accumGyro = WaltLogger.logDouble("Swerve", "accumGyro");
    private final DoubleLogger log_currentEffectiveWheelRad = WaltLogger.logDouble("Swerve", "currentEffectiveWheelRad");
    private final DoubleArrayLogger log_wheelRotations = WaltLogger.logDoubleArray("Swerve", "wheelRotations");
    private final DoubleArrayLogger log_wheelDistance = WaltLogger.logDoubleArray("Swerve", "wheelDistance (in)");

    private final String kTopicPrefix = "Robot/Swerve/";

    StructPublisher<Pose2d> log_choreoActualRobotPose = NetworkTableInstance.getDefault()
        .getStructTopic(kTopicPrefix + "actualRobotPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> log_choreoDesiredRobotPose = NetworkTableInstance.getDefault()
        .getStructTopic(kTopicPrefix + "desiredRobotPose", Pose2d.struct).publish();
    private static String staticKTopicPrefix = "Robot/Swerve/";
    private static StructPublisher<Pose2d> log_autoAlignDestinationPose = NetworkTableInstance.getDefault()
        .getStructTopic(staticKTopicPrefix + "autoAlignDestinationPose", Pose2d.struct).publish();

    StructPublisher<Pose2d> log_trajStartPose = NetworkTableInstance.getDefault()
        .getStructTopic(kTopicPrefix + "activeTrajStart", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> log_trajectory = NetworkTableInstance.getDefault()
        .getStructArrayTopic(kTopicPrefix + "activeTrajectory", Pose2d.struct).publish();
    StructPublisher<Pose2d> log_trajEndPose = NetworkTableInstance.getDefault()
        .getStructTopic(kTopicPrefix + "activeTrajEnd", Pose2d.struct).publish();

    private final DoubleLogger log_errorX = WaltLogger.logDouble("Swerve", "x error");
    private final DoubleLogger log_errorY = WaltLogger.logDouble("Swerve", "y error");
    private final DoubleLogger log_chassisSpeedVXError = WaltLogger.logDouble("Swerve", "vx speed error");
    private final DoubleLogger log_chassisSpeedVYError = WaltLogger.logDouble("Swerve", "vy speed error");

    private double lastGyroYawRads = 0;
    private double accumGyroYawRads = 0;
    private double averageWheelPosition = 0;

    private double[] startWheelPositions = new double[4];
    private double currentEffectiveWheelRadius = 0;
   
    public final AutoFactory autoFactory = createAutoFactory();
    private final AutoRoutine m_teleRoutine = autoFactory.newRoutine("tele"); 

    private Command followTrajectory(Trajectory<SwerveSample> traj) {
        final Timer timer = new Timer();

        Runnable onInit = () -> {
            timer.restart();
            trajLogger(traj, false);
        };

        Runnable onExecute = () -> {
            var sampleOpt = traj.sampleAt(timer.get(), false);
            if(sampleOpt.isEmpty()) {
                return;
            }
            var sample = sampleOpt.get();
            followPath(sample);
        };

        Consumer<Boolean> onEnd = (Boolean intr) -> {
            timer.stop();
            onExecute.run();
        };

        BooleanSupplier isFinished = () -> {
            return timer.get() > traj.getTotalTime();
        };

        return new FunctionalCommand(onInit, onExecute, onEnd, isFinished, this);
    }


    public Command swervePIDTuningSeq(Pose2d scorePose, Field2d field2d) {
        Optional<Trajectory<SwerveSample>> trajOpt = Choreo.loadTrajectory("Start_Right_E_short");

        boolean shouldMirror = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red);

        if (trajOpt.isPresent()) {
            var traj = trajOpt.get();
            return Commands.sequence(
                moveToPose(traj.getInitialPose(shouldMirror).get(), field2d),
                followTrajectory(traj),
                moveToPose(scorePose, field2d)
            );
        }
        return Commands.none();
    }

    private void trajLogger(Trajectory<SwerveSample> traj, boolean startOrFinish) {
        log_trajectory.accept(traj.getPoses());
        boolean shouldMirror = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red);

        if (traj.getInitialPose(shouldMirror).isPresent()) {
            log_trajStartPose.accept(traj.getInitialPose(shouldMirror).get());
        }

        if (traj.getFinalPose(shouldMirror).isPresent()) {
            log_trajEndPose.accept(traj.getFinalPose(shouldMirror).get());
        }
    }

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // SmartDashboard.putData(kTopicPrefix + "AA_X", kAutoAlignXController);
        // SmartDashboard.putData(kTopicPrefix + "AA_Y", kAutoAlignYController);
        // SmartDashboard.putData(kTopicPrefix + "AA_Z", kAutoAlignThetaController);

        // SmartDashboard.putData(kTopicPrefix + "Choreo_X", m_pathXController);
        // SmartDashboard.putData(kTopicPrefix + "Choreo_Y", m_pathYController);
        // SmartDashboard.putData(kTopicPrefix + "Choreo_Z", m_pathThetaController);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory(this::trajLogger);
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this,
            trajLogger
        );
    }


    public void setNeutralMode(NeutralModeValue mode) {
        this.configNeutralMode(mode);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command stopCmd() {
        return runOnce(() -> setControl(swreq_brake));
    }

    private void followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        var pose = getState().Pose;
        var samplePose = sample.getPose();

        var speed = getState().Speeds;
        var targetSpeeds = sample.getChassisSpeeds();

        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                // .withWheelForceFeedforwardsX(sample.moduleForcesX())
                // .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );

        log_choreoActualRobotPose.accept(pose);
        log_choreoDesiredRobotPose.accept(samplePose);

        log_errorX.accept(samplePose.getX() - pose.getX());
        log_errorY.accept(samplePose.getY() - pose.getY());

        log_chassisSpeedVXError.accept(targetSpeeds.vxMetersPerSecond - speed.vxMetersPerSecond);
        log_chassisSpeedVYError.accept(targetSpeeds.vyMetersPerSecond - speed.vyMetersPerSecond);
    }

    public Command directAutoAlignEleUp(
        Supplier<Pose2d> end) {
        return moveToPose(this, end, ChassisSpeeds::new, () -> AutoAlignmentK.kXYConstraintsAuton);
    }

    // transform2d will have to be put in a supplier if you want to ever change it after the program starts running
    // i know it will always be the same offset so i don't have to worry about that though
    public Command autoAlignWithIntermediatePose(
        Supplier<Pose2d> end,
        Transform2d translationToIntermediate) {
        return autoAlignWithIntermediatePose(() -> end.get().transformBy(translationToIntermediate) , end);
    }   

    public Command autoAlignWithIntermediatePose(
        Supplier<Pose2d> intermediate,
        Supplier<Pose2d> end) {
        return Commands.print("start auto align")
            .andThen(moveToPose(this, intermediate, ChassisSpeeds::new))
            .until(() -> isInTolerance(getState().Pose, intermediate.get()))
            .andThen(Commands.print("finished going to intermediate"))
            .andThen(moveToPose(this, end, ChassisSpeeds::new));
    }


    /**
     * Given a destintaion pose, it uses PID to move to that pose. Optimized for auto alignment, so short distances and small rotations.
     * @param destinationPose Give it a destination to go to
     * @return Returns a command that loops until it gets near
     */
    public Command moveToPose(Pose2d destinationPose) {
        return moveToPose(destinationPose, null);
    }

    /**
     * Given a destintaion pose, it uses PID to move to that pose. Optimized for auto alignment, so short distances and small rotations.
     * @param destinationPose Give it a destination to go to
     * @param visionSim visionSim object to get simField from to do sim debugging
     * @return Returns a command that loops until it gets near
     */
    public Command moveToPose(Pose2d destinationPose, Field2d field) {
        if (field != null) {
            field.getObject("destinationPose").setPose(destinationPose);
        }
        log_autoAlignDestinationPose.accept(destinationPose);

        return moveToPose(this, () -> destinationPose, () -> new ChassisSpeeds());
    }

    public static Command moveToPose(
            Swerve swerve,
            Supplier<Pose2d> target,
            Supplier<ChassisSpeeds> speedsModifier) {
        return moveToPose(swerve, target, speedsModifier, () -> AutoAlignmentK.kXYConstraints);
    }

    // these parameters are suppliers because even though this method only uses each once
    // the returned command might be used multiple times
    // the stuff at the beginning is just stuff that can be initialized when the command is bound
    // unfortunately though you need to use final shenanigans to screw with lambdas
    public static Command moveToPose(
            Swerve swerve,
            Supplier<Pose2d> target,
            Supplier<ChassisSpeeds> speedsModifier,
            Supplier<TrapezoidProfile.Constraints> xyConstraints) {
        // This feels like a horrible way of getting around lambda final requirements
        // Is there a cleaner way of doing this?
        final Pose2d cachedTarget[] = {Pose2d.kZero};
        // interestingly no kD in the heading controller
        final ProfiledPIDController headingController =
            // assume we can accelerate to max in 2/3 of a second
            new ProfiledPIDController(
                AutoAlignmentK.kThetaKP, 0.0, 0.0, 
                AutoAlignmentK.kThetaConstraints);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        // ok, use passed constraints on X controller
        final ProfiledPIDController vxController =
            new ProfiledPIDController(AutoAlignmentK.kXKP, 0.00, 0.02, xyConstraints.get());
        // use constraints from constants for y controller?
        // why define them with different constraints?? it's literally field relative
        // the difference in x and y dimensions almost definitely do not mean anything to robot movement
        final ProfiledPIDController vyController =
            new ProfiledPIDController(AutoAlignmentK.kYKP, 0.00, 0.02, xyConstraints.get());

        // this is created at trigger binding, not created every time the command is scheduled
        final SwerveRequest.ApplyFieldSpeeds swreq_driveFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

        return Commands.runOnce(
            () -> {                
                cachedTarget[0] = target.get();
                log_autoAlignDestinationPose.accept(cachedTarget[0]);
                Robot.robotField.getObject("auto align destination").setPose(cachedTarget[0]);

                SwerveDriveState curState = swerve.getState();
                Pose2d curPose = curState.Pose;
                ChassisSpeeds fieldRelativeChassisSpeeds = Swerve.getFieldRelativeChassisSpeeds(curState);
                // for some reason only do logging in simulation?
                // very smart of them to cache whether the robot is in simulation though rather than
                // checking every time though
                // reset profiled PIDs to have the correct speeds
                // we can likely use the Swerve::getVelocityFieldRelative i stuck in there previously
                // stolen from someone elses code
                // this code sets all the setpoints of the controllers

                headingController.reset(
                    curPose.getRotation().getRadians(),
                    fieldRelativeChassisSpeeds.omegaRadiansPerSecond);
                vxController.reset(
                    curPose.getX(), fieldRelativeChassisSpeeds.vxMetersPerSecond);
                vyController.reset(
                    curPose.getY(), fieldRelativeChassisSpeeds.vyMetersPerSecond);
            })
        .andThen(
            // so does this keep running over and over again?
            // i assume it has to make sure that the speeds actually update as
            swerve.applyRequest(
                () -> {
                // get difference between target pose and current pose
                // (this is the transform that maps current pose to target pose)
                // this is only used for tolerances right here.
                final Pose2d curPose = swerve.getState().Pose;
                final Transform2d diff = curPose.minus(cachedTarget[0]);
                final ChassisSpeeds speeds =
                    // for some reason not using tolerance constatnts?? who knows why
                    MathUtil.isNear(0.0, diff.getX(), Units.inchesToMeters(0.75))
                        && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(0.75))
                        && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 0.5)
                    // there is no case in code where speedsModifier is nonzero
                    ? new ChassisSpeeds().plus(speedsModifier.get())
                    : new ChassisSpeeds(
                        // these add the setpoint to velocity for some reason?
                        // i just trust that they know how motion profiles work better
                        // than i do
                        // also why do they include the goal in every call? they shouldn't
                        // have to
                        vxController.calculate(
                                curPose.getX(), cachedTarget[0].getX())
                            + vxController.getSetpoint().velocity,
                        vyController.calculate(
                                curPose.getY(), cachedTarget[0].getY())
                            + vyController.getSetpoint().velocity,
                        headingController.calculate(
                                curPose.getRotation().getRadians(),
                                cachedTarget[0].getRotation().getRadians())
                            + headingController.getSetpoint().velocity)
                        // again there is no case existing in code speedsModifier is nonzero
                    .plus(speedsModifier.get());
                // these people hate logging when the robot is real. do they just go to comp and
                // say screw it we ball?????
                // gng why
                //   if (Robot.ROBOT_TYPE != RobotType.REAL)
                //     Logger.recordOutput(
                //         "AutoAim/Target Pose",
                //         new Pose2d(
                //             vxController.getSetpoint().position,
                //             vyController.getSetpoint().position,
                //             Rotation2d.fromRadians(headingController.getSetpoint().position)));
                  return swreq_driveFieldSpeeds.withSpeeds(speeds);
                }));
  }

    // highlander robotics implementation of nearPose is much cooler
    public static boolean isInTolerance(Pose2d pose, Pose2d pose2) {
        final Transform2d diff = pose.minus(pose2);
        return MathUtil.isNear(
                0.0, Math.hypot(diff.getX(), diff.getY()), AutoAlignmentK.kFieldTranslationTolerance)
            && MathUtil.isNear(
                0.0, diff.getRotation().getRadians(), AutoAlignmentK.kFieldRotationTolerance);
    }

    public static ChassisSpeeds getFieldRelativeChassisSpeeds(SwerveDriveState swerveDriveState) {
        Pose2d pose = swerveDriveState.Pose;
        ChassisSpeeds robotRelChassisSpeeds = swerveDriveState.Speeds;

        return new ChassisSpeeds(
                robotRelChassisSpeeds.vxMetersPerSecond * pose.getRotation().getCos()
                        - robotRelChassisSpeeds.vyMetersPerSecond * pose.getRotation().getSin(),
                robotRelChassisSpeeds.vyMetersPerSecond * pose.getRotation().getCos()
                        + robotRelChassisSpeeds.vxMetersPerSecond * pose.getRotation().getSin(),
                robotRelChassisSpeeds.omegaRadiansPerSecond);
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return getFieldRelativeChassisSpeeds(getState());
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public Pose2d[] extractModulePoses(SwerveDriveState state) {
        Translation2d[] moduleTranslations = {
            new Translation2d(TunerConstants.kFrontLeftXPos.in(Meters), TunerConstants.kFrontLeftYPos.in(Meters)),
            new Translation2d(TunerConstants.kFrontRightXPos.in(Meters), TunerConstants.kFrontRightYPos.in(Meters)),
            new Translation2d(TunerConstants.kBackLeftXPos.in(Meters), TunerConstants.kBackLeftYPos.in(Meters)),
            new Translation2d(TunerConstants.kBackRightXPos.in(Meters), TunerConstants.kBackRightYPos.in(Meters))
        };
        Pose2d[] modulePoses = new Pose2d[getModules().length];
        for (int i = 0; i < getModules().length; i++) {
            modulePoses[i] = 
                state.Pose.transformBy(new Transform2d(
                    moduleTranslations[i], state.ModulePositions[i].angle)
                );
        }
        return modulePoses;
    }


    public Command wheelRadiusCharacterization(double omegaDirection) {

        /* wheel radius characterization schtuffs */
        final DoubleSupplier m_gyroYawRadsSupplier = () -> Units.degreesToRadians(getPigeon2().getYaw().getValueAsDouble());
        // () -> getState().Pose.getRotation().getRadians();
        final SlewRateLimiter m_omegaLimiter = new SlewRateLimiter(0.5);
        final SwerveRequest.RobotCentric m_characterizationReq = new SwerveRequest.RobotCentric()
		    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final double m_characterizationSpeed = 1.5;

		var initialize = runOnce(() -> {
			lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
			accumGyroYawRads = 0;
			currentEffectiveWheelRadius = 0;
            averageWheelPosition = 0;
			for (int i = 0; i < getModules().length; i++) {
				var pos = getModules()[i].getPosition(true);
				startWheelPositions[i] = pos.distanceMeters * TunerConstants.kDriveRotationsPerMeter;
			}
			m_omegaLimiter.reset(0);
		});

		var executeEnd = runEnd(
			() -> {
				setControl(m_characterizationReq
					.withRotationalRate(m_omegaLimiter.calculate(m_characterizationSpeed * omegaDirection)));
				accumGyroYawRads += MathUtil.angleModulus(m_gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
				lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
				averageWheelPosition = 0;
				double[] wheelPositions = new double[4];
				for (int i = 0; i < getModules().length; i++) {
					var pos = getModules()[i].getPosition(true);
					wheelPositions[i] = pos.distanceMeters * TunerConstants.kDriveRotationsPerMeter;
					averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
				}
				averageWheelPosition = averageWheelPosition / 4.0;
				currentEffectiveWheelRadius = (accumGyroYawRads * TunerConstants.kDriveRadius) / averageWheelPosition;
                // System.out.println("effective wheel radius: " + currentEffectiveWheelRadius);
                System.out.println("Average Wheel Position: " + averageWheelPosition);
				log_lastGyro.accept(lastGyroYawRads);
				log_avgWheelPos.accept(averageWheelPosition);
				log_accumGyro.accept(accumGyroYawRads);
				log_currentEffectiveWheelRad.accept(currentEffectiveWheelRadius);
			}, () -> {
				setControl(m_characterizationReq.withRotationalRate(0));
				if (Math.abs(accumGyroYawRads) <= Math.PI * 2.0) {
					System.out.println("not enough data for characterization " + accumGyroYawRads
                    + "\navgWheelPos: " + averageWheelPosition + "radians");
				} else {
					System.out.println(
						"effective wheel radius: "
							+ currentEffectiveWheelRadius
							+ " inches" + 
                            "\naccumGryoYawRads: " + accumGyroYawRads + "radians" 
                            + "\navgWheelPos: " + averageWheelPosition + "radians");
				}
			});

		return Commands.sequence(
			initialize, executeEnd);
	}

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // cache the state for logging
        var swerveState = getState();

        log_choreoActualRobotPose.accept(swerveState.Pose);

        double[] wheelDistance = new double[swerveState.ModulePositions.length];
        double[] wheelRotations = new double[swerveState.ModulePositions.length];
        double[] wheelSpeeds = new double[swerveState.ModuleStates.length];
        double[] wheelSpeedTargets = new double[swerveState.ModuleTargets.length];
        for (int i = 0; i < swerveState.ModulePositions.length; i++) {
            var modPos = swerveState.ModulePositions[i];
            var modState = swerveState.ModuleStates[i];
            var modTarg = swerveState.ModuleTargets[i];

            // position
            var inches = Units.metersToInches(modPos.distanceMeters); 
            var rots =  modPos.distanceMeters / Units.inchesToMeters(TunerConstants.kWheelDiameterInches * Math.PI);
            wheelRotations[i] = rots;
            wheelDistance[i] = inches;
        }
        log_wheelDistance.accept(wheelDistance);
        log_wheelRotations.accept(wheelRotations);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
