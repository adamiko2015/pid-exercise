// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ProportionalControl extends CommandBase {

  private Drivetrain drivetrain;
  private int setpoint;

  public ProportionalControl(Drivetrain drivetrain, int setpoint) {
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double output = Constants.PID.kP * (setpoint - drivetrain.getEncoder());
    drivetrain.setVictor(output);
    System.out.println(output);

    SmartDashboard.putNumber("Proportional Output", output);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
