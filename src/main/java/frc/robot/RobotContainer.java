// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//hi arden
package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExgestAlgea;
import frc.robot.commands.IntakeAlgea;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ExgestCoral;
import frc.robot.commands.WristScoreHigh;
import frc.robot.commands.WristScoreLow;
import frc.robot.commands.WristScoreMid;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.elevatorPositions;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.Intake.wristPositions;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;


import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public final IntakeIOSparkMax m_intakeIO = new IntakeIOSparkMax();
  public final Intake m_intake = new Intake(m_intakeIO);
  public final VisionSubsystem m_vision = new VisionSubsystem();
  //private final Elevator elevator;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick m_manipController =
  new CommandJoystick(1);

  private final SendableChooser<Command> autoChooser;

  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

                SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> m_driverController.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2)*-1)
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (RobotBase.isReal()){
      //elevator = new Elevator(new ElevatorIOSparkMax());
    }
    else if (RobotBase.isSimulation()){
      //elevator = new Elevator(new ElevatorIO() {});
    }
    else{
      //elevator = new Elevator(new ElevatorIO() {});
    }

    // Configure the trigger bindings
    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("ElvatorToL2", new ElevatorToPosition(m_elevator, elevatorPositions.L2_HEIGHT));

    NamedCommands.registerCommand("ElevatorToL2", new ElevatorToPosition(m_elevator, elevatorPositions.L3_HEIGHT));

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    //Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    //Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    
    
   // SmartDashboard.putData("Auto Chooser", autoChooser);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      //m_driverController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      //m_driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    
      m_driverController.square().whileTrue(new ClimberUpCommand(m_climber));
      m_driverController.circle().whileTrue(new ClimberDownCommand(m_climber));
      
    } else
    {
      m_intake.setDefaultCommand(new RunCommand(() -> m_intake.wristAngle(wristPositions.HOLD_ANGLE), m_intake));
      //drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      m_driverController.square().whileTrue(new ClimberUpCommand(m_climber));
      m_driverController.circle().whileTrue(new ClimberDownCommand(m_climber));
      m_driverController.triangle().whileTrue(new RunCommand(
        () -> drivebase.visionReef(m_vision.align_left_branch_supplier()), drivebase));
      m_driverController.cross().onTrue(Commands.runOnce(drivebase::zeroGyro));

      m_manipController.button(1).onTrue(new ElevatorToPosition(m_elevator, elevatorPositions.L1_HEIGHT));
      m_manipController.button(3).onTrue(new ElevatorToPosition(m_elevator, elevatorPositions.L2_HEIGHT));
      m_manipController.button(4).onTrue(new ElevatorToPosition(m_elevator, elevatorPositions.L3_HEIGHT));
      m_manipController.button(2).onTrue(new ElevatorToPosition(m_elevator, elevatorPositions.L4_HEIGHT));
      m_manipController.povLeft().whileTrue((new WristScoreMid(m_intake)));
      m_manipController.povDown().whileTrue(new WristScoreLow(m_intake));
      m_manipController.povRight().whileTrue
      (new WristScoreHigh(m_intake));
      m_manipController.povUp().whileTrue(new WristScoreMid(m_intake));
    //  m_manipController.button(6).onTrue(new IntakeCoral(m_intake));
    //  m_manipController.axisGreaterThan(3, .2).whileTrue(new ExgestCoral(m_intake));
      m_manipController.button(5).onTrue(new IntakeAlgea(m_intake));
      m_manipController.axisGreaterThan(2, .2).whileTrue(new ExgestAlgea(m_intake));
      
    }

  
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
