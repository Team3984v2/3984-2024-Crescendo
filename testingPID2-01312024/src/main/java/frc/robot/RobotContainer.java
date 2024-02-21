// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Swerve.flywheel;
import frc.robot.autos.Auton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.aimAtTarget;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Climber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick driver = new Joystick(1);
  //private final SingleJointedArmSim armM = new Single
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final JoystickButton zeroGyro = 
    new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton aim = 
  new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton halfSpeed = 
    new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton speaker = 
    new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton amp = 
    new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton intake =
    new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  //private final JoystickButton resetwheels = 
  //  new JoystickButton(driver, XboxController.Button.kX.value);
  private final Swerve s_Swerve = new Swerve();
  private final Flywheel fwheel = new Flywheel();
  private final PhotonCamera cam = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)"); //NAME CAMERA  
  private final aimAtTarget aimCommand = new aimAtTarget(cam, s_Swerve, s_Swerve::getPose);
  private final Auton autonChooser = new Auton(s_Swerve);


  // Xbox controller
  
  
   
  // Button board 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        ()->false,//() -> robotCentric.getAsBoolean(),
        () -> halfSpeed.getAsBoolean()));
    fwheel.setDefaultCommand(
      fwheel.moveTo(flywheel.AMP/2, flywheel.AMP/2)
    );
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
  private void configureButtonBindings() {
    aim.whileTrue(aimCommand);
    //zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    speaker.whileTrue(fwheel.moveTo(flywheel.SPEAKER, flywheel.SPEAKER));
    amp.whileTrue(fwheel.moveTo(flywheel.AMP, flywheel.AMP));
    
    //intake.whileTrue(); // TODO
    //resetwheels.onTrue(new InstantCommand(() -> s_Swerve.resetWheels()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    //return new SequentialCommandGroup(
    //  Armm.moveTo(arm.INTAKE[0], arm.INTAKE[1]).withTimeout(4), claw.Outtake().withTimeout(2), Armm.moveTo(arm.RETRACTED[0], arm.RETRACTED[1]));
    // An ExampleCommand will run in autonomous
    //return new SimpleAuto(s_Swerve, claw, Armm);
    return autonChooser.getSelected();
    //return new exampleAuto(s_Swerve, Armm, claw);
  }
}
