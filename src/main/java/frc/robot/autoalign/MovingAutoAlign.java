package frc.robot.autoalign;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.MovingAutoAlignK;
import frc.robot.subsystems.Swerve;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;

public class MovingAutoAlign {
    private static final String kTopicPrefix = "Robot/MovingAutoAlign/";
    private static final StructPublisher<Pose2d> log_destinationPose = NetworkTableInstance.getDefault()
        .getStructTopic(kTopicPrefix + "destination pose", Pose2d.struct).publish();
    private static final DoubleLogger log_errorX = WaltLogger.logDouble(MovingAutoAlignK.kLogTab, "x error");
    private static final DoubleLogger log_errorY = WaltLogger.logDouble(MovingAutoAlignK.kLogTab, "y error");
    private static final DoubleLogger log_errorRot = WaltLogger.logDouble(MovingAutoAlignK.kLogTab, "rotation error degrees");

    /**
     * <p> Returns a Command that automatically aligns with an intermediate pose and target pose that will finish when it is
     *  in tolerances.
     * @param drivetrain The drivetrain
     * @param target Supplier for the target pose - so it can change in run time while preserving the command for triggers and things
     * @param intermediateTransform Transform for transforming target to intermediate - supplier so it change value in run time
     * @return Returns a Command that finishes when it reaches the final target with tolerances in MovingAutoAlignK
     */
    public static Command autoAlignWithIntermediateTransformUntilInTolerances(
            Swerve drivetrain,
            Supplier<Pose2d> target, 
            Supplier<Transform2d> intermediateTransform) {
        return autoAlignWithIntermediatePoseUntilInTolerances(
            drivetrain, 
            target, 
            () -> target.get().transformBy(intermediateTransform.get()));
    }

    /**
     * <p> Returns a Command that will finish on its own given a supplier for the target pose and a supplier
     *  for the intermediate pose. Will stop at intermediate pose.
     * @param drivetrain The drivetrain
     * @param target Supplier for the target position - so that the target can be changed on multiple activations of the command
     * @param intermediate Supplier for the intermediate position - so that the intermediate position can be changed
     *  on multiple activations of the command
     * @return Returns a Command that will finish on its own when it tolerances of the final target and stop at intermediate
     *  pose along the way.
     */
    public static Command autoAlignWithIntermediatePoseUntilInTolerances(
            Swerve drivetrain,
            Supplier<Pose2d> target,
            Supplier<Pose2d> intermediate) {
        return moveToPoseUntilInTolerances(drivetrain, intermediate, ChassisSpeeds::new, () -> MovingAutoAlignK.kXYConstraints)
            .andThen(moveToPoseUntilInTolerances(drivetrain, target, ChassisSpeeds::new, () -> MovingAutoAlignK.kXYConstraints));
    }

    /**
     * <p> This is just {@link #moveToPose(Swerve, Supplier, Supplier, Supplier)} except with an until that checks if it is in
     *  tolerances.
     * @param swerve drivetrain
     * @param target supplier for destination - this is supplier so you can reuse the Command (bindings)
     * @param speedsModifier supplier for speeds modifier - 8033 did this, supplier so you can reuse the Command (bindings)
     * @param xyConstraints supplier for xyConstraints - this is so you can slow down and speed up auto align in particular calls.
     *  supplier because suppliers are confusing
     * @return returns a Command that performs moving auto align until it thinks its in tolerances.
     */
    public static Command moveToPoseUntilInTolerances(
            Swerve swerve,
            Supplier<Pose2d> target,
            Supplier<ChassisSpeeds> speedsModifier,
            Supplier<TrapezoidProfile.Constraints> xyConstraints) {
        return moveToPose(swerve, target, speedsModifier, xyConstraints)
            .until(() -> AutoAlignUtils.isInTolerance(swerve.getState().Pose, target.get(), swerve.getState().Speeds));
    }

    /**
     * <p> Performs moving auto align to attempt to move towards a pose. Does not handle finishing, must be interrupted.
     *  Commenting and formatting might be weird because this was largely taken from 8033's code.
     * @param swerve drivetrain
     * @param target supplier for destination - this is supplier so you can reuse the Command (bindings)
     * @param speedsModifier supplier for speeds modifier - 8033 did this, supplier so you can reuse the Command (bindings)
     * @param xyConstraints supplier for xyConstraints - this is so you can slow down and speed up auto align in particular calls.
     *  supplier because suppliers are confusing
     * @return Returns a Command which commands the drivetrain to move towards a pose using moving auto align. will not
     *  finish without being interrupted.
     */
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
                MovingAutoAlignK.kThetaKP, 0.0, 0.0, 
                MovingAutoAlignK.kThetaConstraints);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        // ok, use passed constraints on X controller
        final ProfiledPIDController vxController =
            new ProfiledPIDController(MovingAutoAlignK.kXKP, 0.01, 0.01, xyConstraints.get());
        // use constraints from constants for y controller?
        // why define them with different constraints?? it's literally field relative
        // the difference in x and y dimensions almost definitely do not mean anything to robot movement
        final ProfiledPIDController vyController =
            new ProfiledPIDController(MovingAutoAlignK.kYKP, 0.01, 0.01, xyConstraints.get());

        // this is created at trigger binding, not created every time the command is scheduled
        final SwerveRequest.ApplyFieldSpeeds swreq_driveFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);

        return Commands.runOnce(
            () -> {                
                cachedTarget[0] = target.get();
                log_destinationPose.accept(cachedTarget[0]);
                // Robot.robotField.getObject("auto align destination").setPose(cachedTarget[0]);

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
                log_errorX.accept(diff.getX());
                log_errorY.accept(diff.getY());
                log_errorRot.accept(diff.getRotation().getDegrees());
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
}
