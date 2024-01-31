package frc.robot.subsystems;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.arm;
import frc.robot.Constants.Swerve.arm.Shoulder;
import frc.robot.Constants.Swerve.flywheel.FWtop;


public class flywheel extends SubsystemBase{
    private CANSparkMax top;
    private CANSparkMax bott;
    private RelativeEncoder topEncoder;
    private RelativeEncoder bottEncoder;
    private SparkMaxPIDController FWtopPID;
    private SparkMaxPIDController FWbottPID;
    public flywheel() {
        // initialies all the variables and constants 
        top = new CANSparkMax(frc.robot.Constants.Swerve.flywheel.FWtop.FWid, MotorType.kBrushless );
        bott = new CANSparkMax(frc.robot.Constants.Swerve.flywheel.FWbott.FWid, MotorType.kBrushless );
        topEncoder = top.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        bottEncoder = bott.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        
        FWtopPID = top.getPIDController();
        FWtopPID.setP(Constants.Swerve.flywheel.FWtop.kP);
        FWtopPID.setI(Constants.Swerve.flywheel.FWtop.kI);
        FWtopPID.setD(Constants.Swerve.flywheel.FWtop.kD);

        FWbottPID = bott.getPIDController();
        FWbottPID.setP(Constants.Swerve.flywheel.FWbott.kP);
        FWbottPID.setI(Constants.Swerve.flywheel.FWbott.kI);
        FWbottPID.setD(Constants.Swerve.flywheel.FWbott.kD);
    }

    public double[] getVelocity(){
        double topVelocity = topEncoder.getVelocity();
        double bottVelocity = bottEncoder.getVelocity();
        double[] combinedVelocity = 
        new double[] {topVelocity , bottVelocity};
        return combinedVelocity;
    }
    public double[] getErrors(double[] goal){
        double[] currVelocity = new double[] {getVelocity()[0], getVelocity()[1]};
        double[] error = new double[] {
            goal[0] - currVelocity[0] , goal[1] - currVelocity[1]
        };
        return error;
    }
    public void GoTo(Rotation2d ShoulderGoal, Rotation2d JointGoal){
        FWtopPID.setReference(
            ShoulderGoal.getDegrees(), 
            ControlType.kPosition, 0
            /*ShoulderFF.calculate(ShoulderGoal.getRadians() 
            /* subtract angle offset from horizontal position later *,
             0)*/
        );
        FWbottPID.setReference(
            JointGoal.getDegrees(), 
            ControlType.kPosition, 0
            /*ShoulderFF.calculate(ShoulderGoal.getRadians() 
            /* subtract angle offset from horizontal position later *,
             0)*/
        );
    }

    public Command moveTo(double Sangle, double Jangle){
        Rotation2d[] a = new Rotation2d[]{Rotation2d.fromDegrees(Sangle)/*(49.26)/*getAngles(x, y)[0]*/, Rotation2d.fromDegrees(Jangle)};/*(61.97)};// getAngles(x, y)[1]};*/
        return runOnce(()->{
            SmartDashboard.putNumber("Shoulder Goal", a[0].getDegrees());
            SmartDashboard.putNumber("Joint Goal", a[1].getDegrees());
        }).andThen(run(
            () -> GoTo(
                a[0], a[1]
            )
        ).until(
            ()->(
                Math.abs(getErrors(a)[0].getDegrees()) < 1 
                && Math.abs(getErrors(a)[1].getDegrees()) < 1
            ))
        );
    }
    public void periodic(){

    }

}
