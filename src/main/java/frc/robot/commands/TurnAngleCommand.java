// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnAngleCommand extends CommandBase {
    /** Creates a new TurnAngleCommand. */
    ;

    double desiredAngle;
    PIDController anglePID;
    Drivetrain drivetrain;

    public TurnAngleCommand(Drivetrain drivetrain, double desiredAngle) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        anglePID = new PIDController(0, 0, 0);
        anglePID.setTolerance(3);
        anglePID.enableContinuousInput(0, 360);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var CurrentAngle = drivetrain.mGyro.getAngle();
        var desiredTurning = anglePID.calculate(CurrentAngle, desiredAngle);
        drivetrain.drive(0, 0, desiredTurning, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if (anglePID.atSetpoint()) {
            return true;
        }
        return false;
    }
}
