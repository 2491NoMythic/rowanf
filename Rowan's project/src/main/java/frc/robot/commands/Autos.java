// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  private DriveTrain drivetrain;
  public static SwerveAutoBuilder autoBuilder;
  private HashMap<String, Command> eventMap;

  private static Autos autos;
  /** Example static factory for an autonomous command. */

  public static Autos getInstance() {
    if (autos == null) {
      autos = new Autos();
    }
    return autos;
  }
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem)); }

  public void AutoInit(SendableChooser<Command> autoChooser, HashMap<String, Command> eventMap, DriveTrain drivetrain) {
    this.drivetrain = drivetrain;
    this.eventMap = eventMap;

    Autos.autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPose,
      drivetrain::resetOdemetry,
      drivetrain.kinematics,
      new PIDConstants(
          DriveConstants.k_XY_P,
          DriveConstants.k_XY_I,
          DriveConstants.k_XY_D),
        new PIDConstants(
          DriveConstants.k_THETA_P,
          DriveConstants.k_THETA_I,
          DriveConstants.k_THETA_D),
      drivetrain::setModuleStates,
      eventMap,
      true,
      drivetrain
        );

      autoChooser.addOption("example auto", exampleAuto(null));
  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
