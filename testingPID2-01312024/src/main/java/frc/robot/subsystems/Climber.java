package frc.robot.subsystems;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.climber;


public class Climber extends SubsystemBase{


    private CANSparkMax climbMotor; 
    private RelativeEncoder EncoderClimb;
    private SparkPIDController ClimbPID;
    private CANSparkMax climbMotor2;
    private RelativeEncoder EncoderClimb2;
    private SparkPIDController ClimbPID2;


    public Climber() {
        // Initialize the Motors
        climbMotor = new CANSparkMax(
            climber.rotMotorID, 
            MotorType.kBrushless
        
        );
        climbMotor2 = new CANSparkMax(
            climber.rotMotorID, 
            MotorType.kBrushless
        );
        
        climbMotor2.restoreFactoryDefaults();
        climbMotor.restoreFactoryDefaults();
        climbMotor2.setIdleMode(IdleMode.kBrake);
        climbMotor.setIdleMode(IdleMode.kBrake);
        
        // Initialize the built in shoulder encoder
        EncoderClimb = climbMotor.getEncoder(
            SparkRelativeEncoder.Type.kHallSensor, 
            42
        );
        EncoderClimb2 = climbMotor2.getEncoder(
            SparkRelativeEncoder.Type.kHallSensor,
            42
        );

        // Set Conversion factor for Encoder -> degrees
        EncoderClimb.setPositionConversionFactor(
            360.0/climber.gearRatio //TODO
        ); 
        // Set Velocity Conversion factor -> degrees/second
        EncoderClimb.setVelocityConversionFactor(
            (360 / climber.gearRatio) / 60.0  //TODO
        );
        // Set the position to zero
        EncoderClimb.setPosition(0);

        EncoderClimb2.setPositionConversionFactor(
            360.0/climber.gearRatio //TODO
        ); 
        // Set Velocity Conversion factor -> degrees/second
        EncoderClimb2.setVelocityConversionFactor(
            (360 / climber.gearRatio) / 60.0  //TODO
        );
        // Set the position to zero
        EncoderClimb2.setPosition(0);
        // Initialiation of PIDs
        ClimbPID = climbMotor.getPIDController();
        ClimbPID.setP(Constants.Swerve.climber.kP);
        ClimbPID.setI(Constants.Swerve.climber.kI);
        ClimbPID.setD(Constants.Swerve.climber.kD);

        ClimbPID2 = climbMotor2.getPIDController();
        ClimbPID2.setP(Constants.Swerve.climber.kP);
        ClimbPID2.setI(Constants.Swerve.climber.kI);
        ClimbPID2.setD(Constants.Swerve.climber.kD);

        // burn flash
        climbMotor.burnFlash();
        climbMotor2.burnFlash();
    }
    // Manual Controls:
    public Command Up(){
        return run(()->{
            climbMotor.set(0.1);
            climbMotor2.set(0.1);
        });
    }
    public Command Down(){
        return run(()->{
            climbMotor.set(-0.1);
            climbMotor2.set(-0.1);
        });
    }
    public Command Stop(){
        return run(()->{
            climbMotor.stopMotor();
            climbMotor2.stopMotor();
        });
    }
    public Rotation2d[] getPos(){
        Rotation2d posClimb = Rotation2d.fromDegrees(EncoderClimb.getPosition()); 
        Rotation2d posClimb2 = Rotation2d.fromDegrees(EncoderClimb2.getPosition()); 
        Rotation2d[] angles = new Rotation2d[]{posClimb, posClimb2};
        return angles;
    }
    public Rotation2d[] getErrors(Rotation2d[] goal){
        Rotation2d[] currPos = new Rotation2d[] {getPos()[0], getPos()[1]};
        Rotation2d[] error = new Rotation2d[] {
            Rotation2d.fromDegrees(currPos[0].getDegrees() - goal[0].getDegrees()), 
            Rotation2d.fromDegrees(currPos[1].getDegrees() - goal[1].getDegrees())
        };
        return error;
    }
    // Run PID
    public void GoTo(Rotation2d[] goal){
        ClimbPID.setReference(
            goal[0].getDegrees(), 
            ControlType.kPosition, 0
        );
        ClimbPID2.setReference(
            goal[0].getDegrees(), 
            ControlType.kPosition, 0
        );
    }
    // Convert PID method into a runable command
    public Command moveTo(Rotation2d[] goal){
        
        //Rotation2d[] a = new Rotation2d[]{Rotation2d.fromDegrees(Sangle)/*(49.26)/*getAngles(x, y)[0]*/, Rotation2d.fromDegrees(Jangle)};/*(61.97)};// getAngles(x, y)[1]};*/
        return runOnce(()->{
            SmartDashboard.putNumber("Right Goal", goal[0].getDegrees());
            SmartDashboard.putNumber("Left Goal", goal[1].getDegrees());
        }).andThen(run(
            () -> GoTo(goal)
        ).until(
            ()->(
                Math.abs(getErrors(goal)[0].getDegrees()) < Constants.Swerve.climber.tolerance 
                && Math.abs(getErrors(goal)[1].getDegrees()) < Constants.Swerve.climber.tolerance
            ))
        );
    }
   
    public void periodic(){
        SmartDashboard.putNumber("ClimbPos", getPos()[0].getDegrees());
    }

}
