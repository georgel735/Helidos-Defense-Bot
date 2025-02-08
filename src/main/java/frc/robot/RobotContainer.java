// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIOSparkMax;


public class RobotContainer
{
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final SlewRateLimiter filter = new SlewRateLimiter(0.45);
    DriveSubsystem driveSubsystem;
    public RobotContainer()
    {
        driveSubsystem = new DriveSubsystem(
            new GyroIOPigeon2(),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[0]),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[1]),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[2]),
            new ModuleIOSparkMax(DriveConstants.MODULE_CONSTANTS[3])
        );
        configureBindings();
    }
    
    private void configureBindings() {
        driveSubsystem.setDefaultCommand(DriveCommands.joystickDrive(
            driveSubsystem,
            () -> filter.calculate(-driveController.getLeftY()),
            () -> filter.calculate(-driveController.getLeftX()),
            () -> -driveController.getRightX()
        ));
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
