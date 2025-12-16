package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class CommandDrive extends Command{
    private DriveTrain driveTrain;
    private Joystick controller;

    public CommandDrive(DriveTrain driveTrain, Joystick controller){
        this.driveTrain = driveTrain;
        this.controller = controller;
        addRequirements(driveTrain);
    }

    public void initialize(){
    }

    public void execute(){
        driveTrain.Drive(controller.getY() * 0.7, controller.getZ() *0.7);
    }
    
    public void end(boolean interrupted){
    }

    public boolean isFinished(){
        return false;
    }
}
