package frc.robot.autoalign;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SharedAutoAlignK;
import frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.LegacyAutoAlignK;

public class LegacyAutoAlign {
    private static final String kTopicPrefix = "Robot/LegacyAutoAlign/";
    private static final StructPublisher<Pose2d> log_destinationPose = NetworkTableInstance.getDefault()
        .getStructTopic(kTopicPrefix + "destination pose", Pose2d.struct).publish();

    private static final SwerveRequest.FieldCentric swreq_driveFieldCentricBlue = new SwerveRequest.FieldCentric()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public static Command moveToPose(Swerve drivetrain, Supplier<Pose2d> destinationPose) {
        final Pose2d[] cachedTarget = {Pose2d.kZero};

        // see construction of PID controllers and constants in Constants.LegacyAutoAlignK
        
        return Commands.runOnce(() -> {
                cachedTarget[0] = destinationPose.get();
                log_destinationPose.accept(cachedTarget[0]);
            }, 
            drivetrain
        ).andThen(Commands.run(
            () -> {
                Pose2d curPose = drivetrain.getState().Pose;

                double xSpeed = LegacyAutoAlignK.kAutoAlignXController.calculate(curPose.getX(), cachedTarget[0].getX());
                double ySpeed = LegacyAutoAlignK.kAutoAlignYController.calculate(curPose.getY(), cachedTarget[0].getY());
                double thetaSpeed = LegacyAutoAlignK.kAutoAlignThetaController.calculate(curPose.getRotation().getRadians(), cachedTarget[0].getRotation().getRadians());
                xSpeed = MathUtil.clamp(xSpeed, -LegacyAutoAlignK.kMaxXYSpeedAutoalign, 
                    LegacyAutoAlignK.kMaxXYSpeedAutoalign);
                ySpeed = MathUtil.clamp(ySpeed, -LegacyAutoAlignK.kMaxXYSpeedAutoalign, 
                    LegacyAutoAlignK.kMaxXYSpeedAutoalign);
                drivetrain.setControl(swreq_driveFieldCentricBlue.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(thetaSpeed));
            }, drivetrain
        )).until(() -> AutoAlignUtils.isInTolerance(drivetrain.getState().Pose, destinationPose.get()));
    }

    public static Command moveToPoseUntilInTolerance(Swerve drivetrain, Supplier<Pose2d> destinationPose) {
        return moveToPose(drivetrain, destinationPose)
            .until(() -> AutoAlignUtils.isInTolerance(drivetrain.getState().Pose, destinationPose.get()));
    }

    // MAX ROTATION TOLERANCE IN RADIANS
    public static Command moveToPoseUntilInTimeScaledTolerance(
            Swerve drivetrain, 
            Supplier<Pose2d> destinationPose, 
            DoubleSupplier maxToleranceTime, 
            DoubleSupplier maxLinearTolerance, 
            DoubleSupplier maxRotationTolerance) {
        final double[] initialTime = {Double.MAX_VALUE};
        return Commands.runOnce(() -> {
            initialTime[0] = Timer.getFPGATimestamp();
        })
            .andThen(moveToPose(drivetrain, destinationPose))
            .until(() -> {
                double currentTime = Timer.getFPGATimestamp();
                double deltaTime = currentTime - initialTime[0];

                double deltaLinearTolerance = maxLinearTolerance.getAsDouble() - SharedAutoAlignK.kFieldTranslationTolerance.in(Meters);
                double deltaRotationTolerance = maxRotationTolerance.getAsDouble() - SharedAutoAlignK.kFieldRotationTolerance.in(Radians);
                
                double percentageTimeComplete = MathUtil.clamp(deltaTime / maxToleranceTime.getAsDouble(), 0, 1);

                double linearTolerance = percentageTimeComplete * deltaLinearTolerance 
                    + SharedAutoAlignK.kFieldTranslationTolerance.in(Meters);
                double rotationTolerance = percentageTimeComplete * deltaRotationTolerance
                    + SharedAutoAlignK.kFieldRotationTolerance.in(Radians);
                
                return AutoAlignUtils.isInTolerance(drivetrain.getState().Pose, destinationPose.get(), 
                    linearTolerance, rotationTolerance);
            });
    }
}
