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
  public void initialize() {}

  @Override
  public void execute() {
    if(RobotBase.isSimulation()){
      if(controller.getPOV() == 0){
        target = 2;
      }
      else if(controller.getPOV() == 180){
        target = 0;
      }
      else if(controller.getPOV() == 90){
        target = 1.5;
      }
      else if(controller.getPOV() == 270){
        target = 1;
      }

    }
    else{
    if(controller.getPOV() == 0){
      target = 400;
    }
    else if(controller.getPOV() == 180){
      target = 0;
    }
    else if(controller.getPOV() == 90){
      target = 328;
    }
    else if(controller.getPOV() == 270){
      target = 202;
    }
    }

    SmartDashboard.putBoolean("Estado PID", elevador.atSetPoint() );
    if(target == 0 && elevador.atSetPoint()){
      elevador.run(0);
    }
    else{
      elevador.runPID(target);
    }
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