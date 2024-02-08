package frc.robot.subsystems;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.arm;
import frc.robot.Constants.Swerve.arm.Shoulder;


public class Climber extends SubsystemBase{

    public Climber() {
        // Initialize the Motors
        climbMotor = new CANSparkMax(
            climber.Climb.rotMotorID, 
            MotorType.kBrushless
        );
        climbMotor.restoreFactoryDefaults();
        climbMotor.setIdleMode(IdleMode.kBrake);
        // Initialize the built in climb encoder
        EncoderClimb = climbMotor.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        // Set Conversion factor for Encoder -> degrees
        EncoderClimb.setPositionConversionFactor(
            360.0/arm.Climb.gearRatio
        );
        // Set Velocity Conversion factor -> degrees/second
        EncoderShoulder.setVelocityConversionFactor(
            (360 / arm.Shoulder.gearRatio) / 60.0 
        );
        // Set the position to zero
        EncoderShoulder.setPosition(0);
        // burn flash
        shoulderMotor.burnFlash();
       
        
    }
}