// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceCommand extends CommandBase {
    double desiredMeterDistance;
    Drivetrain drivetrain;
    PIDController distanceController;

    /** Creates a new DriveDistanceCommand. */
    public DriveDistanceCommand(Drivetrain drivetrain, double desiredMeterDistance) {
        // Use addRequirements() here to declare subsystem dependencies.

        addRequirements(drivetrain);

        this.desiredMeterDistance = desiredMeterDistance;
        this.drivetrain = drivetrain;
        distanceController = new PIDController(0, 0, 0);
        distanceController.setTolerance(0.3);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentDistance = 0;

        var desiredSpeed = distanceController.calculate(currentDistance, desiredMeterDistance);
        // drivetrain.mGyro.getRawGyro(null).
        drivetrain.drive(0, desiredSpeed, 0, false);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, true);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (distanceController.atSetpoint()) {
            return true;
        }

        return false;
    }
}
