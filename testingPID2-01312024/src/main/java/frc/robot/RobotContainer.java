// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.aimAtTarget;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.Swerve.flywheel;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands
  private final Joystick driver = new Joystick(1);
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int triggerLAxis = XboxController.Axis.kLeftTrigger.value;
  private final int triggerRAxis = XboxController.Axis.kRightTrigger.value;
  private final JoystickButton zeroGyro = 
    new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton aim = 
    new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton speaker = 
    new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton amp = 
    new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton lolintake =
    new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton slow = 
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final Swerve s_Swerve = new Swerve();
  private final Flywheel fwheel = new Flywheel();
  private final PhotonCamera cam = new PhotonCamera("Global_Shutter_Camera");
  private final aimAtTarget aimCommand = new aimAtTarget(cam, s_Swerve, s_Swerve::getPose);
  private final SendableChooser<Command> autoChooser;
  private final Intake intake = new Intake();  
  /** The container for the robot. subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        ()->false,//() -> robotCentric.getAsBoolean(),
        () -> slow.getAsBoolean()));
    //intake.setDefaultCommand(
      //intake.moveTo(Constants.Swerve.intake.IDLE, false)
    //);
    /*fwheel.setDefaultCommand(
      fwheel.moveTo(
        ()->amp.getAsBoolean(), 
        ()->speaker.getAsBoolean(), 
        ()-> driver.getRawAxis(triggerLAxis)));*/
    // Configure the button bindings
    configureButtonBindings();
    
  }

  //robotCentric.get();
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private boolean speaekerToggle = false;
  private void configureButtonBindings() {
    //aim.whileTrue(aimCommand);
    //zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    speaker.whileTrue(fwheel.moveTo(flywheel.SPEAKER, flywheel.SPEAKER, false));
    //amp.whileTrue(fwheel.moveTo(flywheel.AMP, flywheel.AMP, false));
    //speaker.onTrue(new InstantCommand(()->{speaekerToggle = true;}));
    //amp.onTrue(new InstantCommand(()->{speaekerToggle = false;}));
    //if (driver.getRawAxis(triggerRAxis) > 0.3){intake.Out();}
    //lolintake.whileTrue(intake.Out());
    lolintake.whileTrue(intake.moveTo(Constants.Swerve.intake.INTAKE, true));
    aim.whileTrue(intake.moveTo(Constants.Swerve.intake.IDLE, false));
    amp.whileTrue(intake.moveTo(Constants.Swerve.intake.AMPSHOT, false));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){return autoChooser.getSelected();}
  public Command testPath(){return new PathPlannerAuto("testAuto");}
  // Left side autos
  public Command Left1Note(){return new PathPlannerAuto("Left1Note");}
  public Command Left2Note(){return new PathPlannerAuto("Left2Note");}
  public Command Left3Note(){return new PathPlannerAuto("Left3Note");}
  public Command Left3NoteFar(){return new PathPlannerAuto("Left3NoteFar");}
  public Command Left4Note(){return new PathPlannerAuto("Left4Note");}
  // Mid side autos
  public Command Mid1Note(){return new PathPlannerAuto("Mid1Note");}
  public Command Mid2Note(){return new PathPlannerAuto("Mid2Note");}
  public Command Mid3NoteFar(){return new PathPlannerAuto("Mid3NoteFar");}
  public Command Mid4Note(){return new PathPlannerAuto("Mid4Note");}
  public Command MidLeft3Note(){return new PathPlannerAuto("MidLeft3Note");}
  public Command MidRight3Note(){return new PathPlannerAuto("MidRight3Note");}
  //Right side autos
  public Command Right1Note(){return new PathPlannerAuto("Right1Note");}
  public Command Right2Note(){return new PathPlannerAuto("Right2Note");}
  public Command Right2NoteFar(){return new PathPlannerAuto("Right2NoteFar");}
  public Command Right3NoteFar(){return new PathPlannerAuto("Right3NoteFar");}
}
