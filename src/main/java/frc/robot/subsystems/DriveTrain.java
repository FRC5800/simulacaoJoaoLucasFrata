package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase{

    private Field2d field2d = new Field2d();
    
    
    private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.ConstantsDriveTrain.kLeftMaster);
    private final WPI_TalonSRX leftSlave = new WPI_TalonSRX(Constants.ConstantsDriveTrain.kLeftSlave);
    private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.ConstantsDriveTrain.kRightMaster);
    private final WPI_TalonSRX rightSlave = new WPI_TalonSRX(Constants.ConstantsDriveTrain.kRightSlave);
    


    private DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

    private DifferentialDrivetrainSim diffDriveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),
    10.71,
    5.708124999999999,
    60,
    Units.inchesToMeters(6),
    0.685,
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));



    public DriveTrain(){   
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    
    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
    rightMaster.setInverted(false);
    rightSlave.setInverted(true);


    SmartDashboard.putData("field", field2d); 
    }

    private double rightVoltage = 0;
    private double leftVoltage = 0;


    public void Drive(double speed, double rotation){
        diffDrive.arcadeDrive(speed, rotation);
        leftVoltage = (speed - rotation) * 5;
        rightVoltage = (speed + rotation) * 5;

    }

    public void stop(){
        diffDrive.stopMotor();
        leftVoltage = 0;
        rightVoltage = 0;
    }


    @Override
    public void periodic(){
    } 

    @Override
    public void simulationPeriodic(){
        diffDriveSim.setInputs(
            - leftVoltage,
            - rightVoltage
        );

        diffDriveSim.update(0.02);
        field2d.setRobotPose(diffDriveSim.getPose());
    }

}