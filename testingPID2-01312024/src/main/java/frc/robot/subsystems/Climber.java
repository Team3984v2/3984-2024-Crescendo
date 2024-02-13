package frc.robot.subsystems;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.arm;
import frc.robot.Constants.Swerve.arm.Shoulder;
import frc.robot.Constants.Swerve.climber;


public class Climber extends SubsystemBase{


    private CANSparkMax climbMotor; 
    private RelativeEncoder EncoderClimb;
    private ArmFeedforward ClimbFF;
    private SparkPIDController ClimbPID;



    public Climber() {
        // Initialize the Motors
        climbMotor = new CANSparkMax(
            climber.rotMotorID, 
            MotorType.kBrushless
        
        );
        climbMotor.restoreFactoryDefaults();
        climbMotor.setIdleMode(IdleMode.kBrake);

        
        // Initialize the built in shoulder encoder
        EncoderClimb = climbMotor.getEncoder(
            SparkRelativeEncoder.Type.kHallSensor, 
            42
        );
        // Set Conversion factor for Encoder -> degrees
        EncoderClimb.setPositionConversionFactor(
            360.0/arm.Shoulder.gearRatio
        );
        // Set Velocity Conversion factor -> degrees/second
        EncoderClimb.setVelocityConversionFactor(
            (360 / arm.Shoulder.gearRatio) / 60.0 
        );
        // Set the position to zero
        EncoderClimb.setPosition(0);

       
        // Set Valocity Conversion factor -> degrees/second
        EncoderClimb.setVelocityConversionFactor(
            (360 / arm.Shoulder.gearRatio) / 60.0
        );
        // Set position to zero
      
        ClimbPID = climbMotor.getPIDController();
        ClimbPID.setP(Constants.Swerve.climber.kP);
        ClimbPID.setI(Constants.Swerve.climber.kI);
        ClimbPID.setD(Constants.Swerve.climber.kD);


        // Initialize the built in Motor PID controllers

        // burn flash
        climbMotor.burnFlash();
     
        
    }

    public Command Up(){
        return run(() 
        -> climbMotor.set(0.1));
        
    }
    public Command Down(){
        return run(() 
        -> climbMotor.set(-0.1));
    }
    public Command Stop(){
        return run(() 
        -> climbMotor.set(-0.1));
    }

    public Rotation2d[] getPos(){
        Rotation2d posClimb = Rotation2d.fromDegrees(EncoderClimb.getPosition()); // returns a value 1/4 of actual angle for some reason
        Rotation2d[] angles = new Rotation2d[]{posClimb,};
        return angles;
    }
    public Rotation2d[] getErrors(Rotation2d[] goal){
        Rotation2d[] currPos = new Rotation2d[] {getPos()[0], getPos()[1]};
        Rotation2d[] error = new Rotation2d[] {
            Rotation2d.fromDegrees(currPos[0].getDegrees() - goal[0].getDegrees()), 
            Rotation2d.fromDegrees(currPos[1].getDegrees() - 90/*goal[1].getDegrees()*/)
        };
        return error;
    }
   
    public void periodic(){
        SmartDashboard.putNumber("ClimbPos", 1);
    }

}
