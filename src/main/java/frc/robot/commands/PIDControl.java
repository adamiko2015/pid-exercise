// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class PIDControl extends CommandBase {

  private Drivetrain drivetrain;
  private PIDController pid;

  private int setpoint;

  public PIDControl(Drivetrain drivetrain, int setpoint) {
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;

    pid = new PIDController(Constants.PID.kP, Constants.PID.kI, Constants.PID.kD);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double output = pid.calculate(drivetrain.getEncoder(), setpoint);
    drivetrain.setVictor(output);

    SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("Error", pid.getPositionError());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}