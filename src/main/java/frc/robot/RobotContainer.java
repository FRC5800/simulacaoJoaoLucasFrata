// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.CommandElevador;
import frc.robot.commands.CommandDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevador;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {

  private Joystick controller = new Joystick(0);

  private final DriveTrain driveTrain = new DriveTrain();
  private final Elevador elevador = new Elevador();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings(controller);
  }

  
  private void configureBindings(Joystick controller) {
      driveTrain.setDefaultCommand(new CommandDrive(driveTrain,controller));
      elevador.setDefaultCommand(new CommandElevador(elevador, controller));
     }
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
