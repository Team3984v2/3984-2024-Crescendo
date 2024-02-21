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

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.intake.intakeArm;
import frc.robot.Constants.Swerve.intake.intakeMotor;

public class Intake extends SubsystemBase{
    
    private CANSparkMax lolIntakeArm;
    private RelativeEncoder intakeArmEncoder;
    private SparkPIDController intakeArmPID;
    private Spark intakeMotorL;
    private Spark intakeMotorR;

    public void Intake(){

        lolIntakeArm = new CANSparkMax(
            intakeArm.rotMotorID,
            MotorType.kBrushless
        );

        intakeMotorL = new Spark(
            intakeMotor.IDL
        );
        intakeMotorR = new Spark(
            intakeMotor.IDR
        );


        intakeMotorL.set(0);
    }
    public Command Out(){
         return run(()->{
            intakeMotorL.set(0.1);
            intakeMotorR.set(0.1);
         });
    }
    public Command In(){
        return run(()->{
            intakeMotorL.set(-0.1);
            intakeMotorR.set(-0.1);
        });
    }
    public Command Stop(){
        return run(()->{
            intakeMotorL.stopMotor();
            intakeMotorR.stopMotor();
        });
    }
}
