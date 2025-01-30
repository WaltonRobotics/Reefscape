// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autons.WaltAutonFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController manipulator = new CommandXboxController(1);

  public final Swerve drivetrain = TunerConstants.createDrivetrain();
  private final Algae algae = new Algae();
  private final Elevator elevator = new Elevator();

  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  private final WaltAutonFactory waltAutonFactory = new WaltAutonFactory(autoFactory, drivetrain, elevator);
  private final AutoChooser autoChooser = new AutoChooser();

  public Robot() {
     /* autossss */

      SmartDashboard.putData("Auto Chooser", autoChooser);

      configureBindings();
  }

  private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //joystick.rightBumper().whileTrue(drivetrain.wheelRadiusCharacterization(1));
        //joystick.rightTrigger().whileTrue(drivetrain.wheelRadiusCharacterization(-1));

        /*TODO: test to see if this actually works */
        // wrist position controls
        manipulator.y().onTrue(algae.toAngle(Algae.WristPosition.HOME));
        manipulator.x().onTrue(algae.toAngle(Algae.WristPosition.INTAKE));
        manipulator.b().onTrue(algae.toAngle(Algae.WristPosition.PROCESSOR_SHOOT));

        // intake controls
        // manipulator.rightTrigger()
        //     .whileTrue(algae.setWheelAction(Algae.IntakeSpeed.INTAKE));
        // manipulator.leftTrigger()
        //     .whileTrue(algae.setWheelAction(Algae.IntakeSpeed.PROCESSOR_SHOOT));
        


        joystick.povDown().onTrue(elevator.setPosition(Elevator.EleHeights.HOME));
        joystick.povLeft().onTrue(elevator.setPosition(Elevator.EleHeights.L1));
        joystick.povRight().onTrue(elevator.setPosition(Elevator.EleHeights.L2));
        joystick.povUp().onTrue(elevator.setPosition(Elevator.EleHeights.L3));
        joystick.x().onTrue(elevator.setPosition(Elevator.EleHeights.L4));
        joystick.y().onTrue(elevator.setPosition(Elevator.EleHeights.CS));

        joystick.a().onTrue(elevator.setPosition(Elevator.EleHeights.CLIMB_UP));
        joystick.b().onTrue(elevator.setPosition(Elevator.EleHeights.CLIMB_DOWN));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.selectedCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
