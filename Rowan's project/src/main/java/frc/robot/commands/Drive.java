// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends CommandBase {
  private final DriveTrain drivetrain;
  private final BooleanSupplier robotCentricMode;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  /** Creates a new Drive. */
  public Drive(DriveTrain drivetrainSubsystem,
  BooleanSupplier robotCentricMode,
  DoubleSupplier translationXSupplier,
  DoubleSupplier translationYSupplier,
  DoubleSupplier rotationSupplier) {
    this.drivetrain = drivetrainSubsystem;
    this.robotCentricMode = robotCentricMode;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }



// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (robotCentricMode.getAsBoolean()) {
      drivetrain.drive(new ChassisSpeeds(
        translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
      ));
    } else {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      drivetrain.getGyroscopeRotation()
      )
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
