// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    private DigitalOutput timer;
    private RobotContainer robotContainer;

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        Logger.addDataReceiver(new NT4Publisher());
        new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // enable power logging

        Logger.start();

        robotContainer = new RobotContainer();
        timer = new DigitalOutput(0);
    }
    
    
    @Override
    public void robotInit() {}
    
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();

    }
    
    
    @Override
    public void disabledInit() {
        timer.set(false);
    }
    
    
    @Override
    public void disabledPeriodic() {
        timer.set(false);
    }
    
    
    @Override
    public void disabledExit() {
        timer.set(true);
    }
    
    
    @Override
    public void autonomousInit()
    {
        autonomousCommand = robotContainer.getAutonomousCommand();
        
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
        timer.set(true);
    }
    
    
    @Override
    public void autonomousPeriodic() {
        timer.set(true);
    }
    
    
    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
        timer.set(true);
    }
    
    
    @Override
    public void teleopPeriodic() {
        timer.set(true);
    }
    
    
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}
}
