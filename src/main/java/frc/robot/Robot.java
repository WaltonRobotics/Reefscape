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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autons.AutonChooser;
import frc.robot.autons.TrajsAndLocs;
import frc.robot.autons.WaltAutonFactory;
import frc.robot.autons.AutonChooser.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.EleHeights;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Swerve drivetrain = TunerConstants.createDrivetrain();

    public final Elevator elevator = new Elevator();

    private final AutoFactory autoFactory = drivetrain.createAutoFactory();
    private final WaltAutonFactory waltAutonFactory = new WaltAutonFactory(autoFactory, drivetrain, elevator);
    private final AutoChooser autoChooser = new AutoChooser();

    private void mapAutonCommands(){
      AutonChooser.assignAutonCommand(StartAuton.MEOW, TrajsAndLocs.FirstScoringLocs.REEF_1);
      AutonChooser.assignAutonCommand(StartAuton.GROWL, TrajsAndLocs.FirstScoringLocs.REEF_2);
      AutonChooser.assignAutonCommand(StartAuton.BARK, TrajsAndLocs.FirstScoringLocs.REEF_3);
      
      //ele heights choices
      AutonChooser.assignAutonCommand(EleAutonHeights.HOME, EleHeights.HOME);
      AutonChooser.assignAutonCommand(EleAutonHeights.L1, EleHeights.L1);
      AutonChooser.assignAutonCommand(EleAutonHeights.L2, EleHeights.L2);
      AutonChooser.assignAutonCommand(EleAutonHeights.L3, EleHeights.L3);
      AutonChooser.assignAutonCommand(EleAutonHeights.L4, EleHeights.L4);

      //coral station choices
      AutonChooser.assignAutonCommand(CSOptions.LEFT, TrajsAndLocs.CS.CS_LEFT);
      AutonChooser.assignAutonCommand(CSOptions.RIGHT, TrajsAndLocs.CS.CS_RIGHT);
    }


  public Robot() {
     /* autossss */

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

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

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
  public void robotInit(){
    mapAutonCommands();
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
