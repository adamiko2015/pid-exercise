// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BangbangControl extends CommandBase {

  private Drivetrain drivetrain;
  private int setpoint;

  public BangbangControl(Drivetrain drivetrain, int setpoint) {
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(drivetrain.getEncoder() < setpoint) {
      drivetrain.setVictor(1.0);
    } else if(drivetrain.getEncoder() > setpoint) {
      drivetrain.setVictor(-1.0);
    }

    SmartDashboard.putNumber("BangBang Output", drivetrain.getEncoder());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
