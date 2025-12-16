package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevador extends SubsystemBase {

    public PIDController pidElevador = new PIDController(Constants.ConstantsElevador.kp, Constants.ConstantsElevador.ki,
            Constants.ConstantsElevador.kd);

    private RelativeEncoder masterEncoder;
    private RelativeEncoder slaveEncoder;

    private SparkMax motorMaster = new SparkMax(Constants.ConstantsElevador.Master_ID_Elevador, MotorType.kBrushless);
    private SparkMax motorSlave = new SparkMax(Constants.ConstantsElevador.Slave_ID_Elevador, MotorType.kBrushless);

    private final Mechanism2d mechanism2d = new Mechanism2d(90, 90);
    private final MechanismRoot2d root = mechanism2d.getRoot("Base", 45, 5);
    private final MechanismLigament2d parte1 = new MechanismLigament2d("Parte 1", 30, 90);
    private final MechanismLigament2d parte2 = new MechanismLigament2d("Parte 2", 15, 15);

    private double tensao = 0;

    private static final double GEAR_RATIO = 2.25;
    private static final double MASS_KG = 5;
    private static final double DRUM_RADIUS_M = 0.03;
    private static final double MIN_HEIGHT_M = 0.0;
    private static final double MAX_HEIGHT_M = 2;

    private final ElevatorSim elevadorSim = new ElevatorSim(
            DCMotor.getNEO(2),
            GEAR_RATIO,
            MASS_KG,
            DRUM_RADIUS_M,
            MIN_HEIGHT_M,
            MAX_HEIGHT_M,
            true,
            0.0);

    public Elevador() {

        pidElevador.setTolerance(Constants.ConstantsElevador.pidTolerencia);

        root.append(parte1);
        parte1.append(parte2);

        SparkMaxConfig masterConfig = new SparkMaxConfig();

        masterConfig.inverted(false);
        masterConfig.idleMode(IdleMode.kBrake);
        masterConfig.disableFollowerMode();

        SparkMaxConfig slaveConfig = new SparkMaxConfig();
        slaveConfig.idleMode(IdleMode.kBrake);

        motorMaster.configure(masterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        motorSlave.configure(slaveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        masterEncoder = motorMaster.getEncoder();
        slaveEncoder = motorSlave.getEncoder();

        SmartDashboard.putData("Field Elevador", mechanism2d);
    }

    public void ResetEncoder() {
        masterEncoder.setPosition(0);
        slaveEncoder.setPosition(0);
    }

    public void runPID(double target) {
        //pidElevador.setSetpoint(target);
        //double speed = MathUtil.clamp(pidElevador.calculate(getHeight()), -0.1, 0.3);
        //run(speed);
        run(0.5);
    }

    public void run(double speed) {
        tensao = speed * 12;
        motorMaster.set(speed);
        motorSlave.set(-speed);
    }

    public void PIDControllerElevator(double target) {
        pidElevador.setSetpoint(target);
        motorMaster.set(pidElevador.calculate(getHeight()));
        motorSlave.set(-pidElevador.calculate(getHeight()));

    }

    // public void ControleElevador(double speed, Joystick controller){
    // motorMaster.set(speed * 0.25);
    // motorSlave.set(-speed * 0.25);
    // }

    public double ticksToMeters(double ticks) {
        return ticks * Constants.ConstantsElevador.calculo_Ticks_Elevador;
    }

    public boolean isAtSetPoint() {
        return pidElevador.atSetpoint();
    }

    public double getHeight() {
        if (ticksToMeters(masterEncoder.getPosition()) - ticksToMeters(slaveEncoder.getPosition()) / 2 < 0) {
            return 0;
        }
        return (ticksToMeters(masterEncoder.getPosition()) - ticksToMeters(slaveEncoder.getPosition())) / 2;
    }

    public boolean atSetPoint() {
        return pidElevador.atSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("PID do Elevador", pidElevador);
        SmartDashboard.putNumber("Encoder Elevador", getHeight());
    }

    @Override
    public void simulationPeriodic() {
        elevadorSim.setInput(tensao);
        elevadorSim.update(0.02);

        double posSimMetros = elevadorSim.getPositionMeters();
        double posSimTicks = posSimMetros * 2 / Constants.ConstantsElevador.calculo_Ticks_Elevador;
        
        masterEncoder.setPosition(posSimTicks); 
        slaveEncoder.setPosition(0);

        parte1.setLength(posSimMetros * 10);

        SmartDashboard.putNumber("Tensão Elevador Simulado", tensao);
        SmartDashboard.putNumber("Posicao Elevador Sim", posSimMetros);
    }
}