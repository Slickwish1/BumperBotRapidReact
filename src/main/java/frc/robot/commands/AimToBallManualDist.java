package frc.robot.commands;

import java.util.Iterator;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class AimToBallManualDist extends CommandBase {
    private DriveSystem m_subsystem;
    private boolean m_finished = false;

    final double LINEAR_P = 0.1;

    final double LINEAR_D = 0.0;

    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.8;// SmartDashboard.getNumber("Angular Pos", 0.1);

    final double ANGULAR_D = 0.01;

    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    private NetworkTable table;

    GenericHID controller;

    private Boolean isBlue;

    public AimToBallManualDist(DriveSystem subsystem, GenericHID driveController) {
        m_subsystem = subsystem;
        m_finished = true;
        controller = driveController;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    public AimToBallManualDist(DriveSystem subsystem) {
        m_subsystem = subsystem;
        m_finished = true;
        controller = null;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    public AimToBallManualDist(DriveSystem subsystem, GenericHID driveController, Boolean isTeamBlue) {
        m_subsystem = subsystem;
        m_finished = true;
        controller = driveController;
        isBlue = isTeamBlue;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    public AimToBallManualDist(DriveSystem subsystem, Boolean isTeamBlue) {
        m_subsystem = subsystem;
        m_finished = true;
        controller = null;
        isBlue = isTeamBlue;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }


    @Override
    public void initialize() {
        System.out.println("Aiming to Ball");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("ML");
    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_finished = false;
        NetworkTableEntry xEntry = table.getEntry(isBlue ? "best_blue_x": "best_red_x");
        System.out.println(xEntry);
        NetworkTableEntry widthEntry = table.getEntry("width");
        if ((Double) xEntry.getNumber(Double.valueOf(0.0)) > Double.valueOf(1.0)) {
            Double width = (Double) widthEntry.getNumber(Double.valueOf(0));
            Double best_target_x = (Double) xEntry.getNumber(0);
            System.out.println(best_target_x);
            double error = best_target_x - width / 2.0;
            m_finished = error < 10;
            if (Math.abs(error) > 5) {
                System.out.println(error);
                // error = error / 160.0;
                double turnSpeed = turnController.calculate(error, 0);
                turnSpeed = turnSpeed * (1.0 / 160.0) * 0.5;
                SmartDashboard.putNumber("turnSpeed", turnSpeed);
                if (controller != null) {
                    m_subsystem.drive(controller.getRawAxis(1) > 0.1 || controller.getRawAxis(1) < -0.1
                            ? -controller.getRawAxis(1)
                            : 0.0, turnSpeed);

                } else {
                    m_subsystem.drive(0.0, turnSpeed);
                }
            }

        }
    }
}