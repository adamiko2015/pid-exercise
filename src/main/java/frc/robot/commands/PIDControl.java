// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PIDControl extends CommandBase {

  private Drivetrain drivetrain;

  private int setpoint;
  private double kp, ki, kd;

  private double integralSum = 0;

  private long prev_time = System.currentTimeMillis();
  private int prev_error;

  public PIDControl(Drivetrain drivetrain, int setpoint, double kp, double ki, double kd) {
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;

    this.kp = kp;
    this.ki = ki;
    this.kd = kd;

    addRequirements(drivetrain);
  }

  public double calculateOutput(int encoder, int setpoint) {
    int current_error = setpoint - encoder;
    integralSum += current_error;

    long current_time = System.currentTimeMillis(); 

    double p_out =  current_error * kp;
    double i_out = integralSum * ki;
    double d_out = ((double) (current_error - prev_error) / (current_time - prev_time)) * kd;

    prev_time = current_time;
    prev_error = current_error;

    return p_out + i_out + d_out;
  }

  public int getError() {
    return setpoint - drivetrain.getEncoder();
  }
  
  @Override
  public void initialize() {
    prev_error = drivetrain.getEncoder() - setpoint;
  }

  @Override
  public void execute() {
    double output = calculateOutput(drivetrain.getEncoder(), setpoint);
    drivetrain.setVictor(output);

    SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("Error", getError());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
