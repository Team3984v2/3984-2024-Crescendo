package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


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
        topEncoder.setVelocityConversionFactor(0);
        bottEncoder = bott.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        bottEncoder.setVelocityConversionFactor(0);
        
        FWtopPID = top.getPIDController();
        FWtopPID.setP(Constants.Swerve.flywheel.FWtop.kP);
        FWtopPID.setI(Constants.Swerve.flywheel.FWtop.kI);
        FWtopPID.setD(Constants.Swerve.flywheel.FWtop.kD);

        FWbottPID = bott.getPIDController();
        FWbottPID.setP(Constants.Swerve.flywheel.FWbott.kP);
        FWbottPID.setI(Constants.Swerve.flywheel.FWbott.kI);
        FWbottPID.setD(Constants.Swerve.flywheel.FWbott.kD);

        top.burnFlash();
        bott.burnFlash();
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
    public void GoTo(double ShoulderGoal, double JointGoal){
        FWtopPID.setReference(
            ShoulderGoal, 
            ControlType.kPosition, 0
            /*ShoulderFF.calculate(ShoulderGoal.getRadians() 
            /* subtract angle offset from horizontal position later *,
             0)*/
        );
        FWbottPID.setReference(
            JointGoal, 
            ControlType.kPosition, 0
            /*ShoulderFF.calculate(ShoulderGoal.getRadians() 
            /* subtract angle offset from horizontal position later *,
             0)*/
        );
    }

    public Command moveTo(double speedhi, double speedlo){
        double[] v = new double[]{speedhi, speedlo};
        return runOnce(()->{
            SmartDashboard.putNumber("TopVelocity", v[0]);
            SmartDashboard.putNumber("BottomVelocity", v[1]);
        }).andThen(run(
            () -> GoTo(
                v[0], v[1]
            ))
        ).until(
            ()->(
                Math.abs(getErrors(v)[0])<1 && Math.abs(getErrors(v)[1]) < 1
            )
        );
    }
    public void periodic(){

    }

}
