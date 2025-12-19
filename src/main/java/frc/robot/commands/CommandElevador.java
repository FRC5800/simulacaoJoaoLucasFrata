package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevador;

public class CommandElevador extends Command {
  Elevador elevador;
  double target = 0;
  Joystick controller;

  public CommandElevador(Elevador elevador, Joystick controller) {
    this.elevador = elevador;
    this.controller = controller;
    addRequirements(elevador);
  }

  @Override
  public void initialize() {
    elevador.ResetEncoder();
  }

  @Override
  public void execute() {
    if (RobotBase.isSimulation()) {
      if (controller.getPOV() == 0) {
        target = 1;
      } else if (controller.getPOV() == 180) {
        target = 0;
      } else if (controller.getPOV() == 90) {
        target = 0.6;
      } else if (controller.getPOV() == 270) {
        target = 0.3;
      }
      SmartDashboard.putNumber("Altura", target);

    }
    // else{
    // if(controller.getPOV() == 180){
    // target = 0;
    // }else if(controller.getPOV()==90)

    // {
    // target = 93;
    // }else if(controller.getPOV()==270)
    // {
    // target = 45;
    // }elevador.runPID(target);
    // }
    // }

    SmartDashboard.putBoolean("Estado PID", elevador.atSetPoint());
    elevador.runPID(target);
  }

  @Override
  public void end(boolean interrupted) {
    elevador.run(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}