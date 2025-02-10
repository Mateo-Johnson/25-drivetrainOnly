package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.utils.Constants.InterfaceConstants;

public class RobotContainer {

    // The robot's subsystems
    private final Drivetrain drivetrain = new Drivetrain(); // Drivetrain Subsystem

    // The driver's controller
    public static final CommandXboxController primary = new CommandXboxController(InterfaceConstants.primaryPort);

    /**
     * The container for the robot. Contains subsystems, interface devices, and commands.
     */
    public RobotContainer() {
    
        // Configure the button bindings
        configureButtonBindings();
    
        // Configure default commands
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> {
                    // Get the controller inputs
                    double ySpeed = -MathUtil.applyDeadband(primary.getLeftY(), InterfaceConstants.driveDeadband);
                    double xSpeed = -MathUtil.applyDeadband(primary.getLeftX(), InterfaceConstants.driveDeadband);
                    double rot = -MathUtil.applyDeadband(primary.getRightX(), InterfaceConstants.driveDeadband);
    
                    // Apply slow factor if B button is pressed
                    if (primary.b().getAsBoolean()) {
                        ySpeed *= InterfaceConstants.slowFactor;
                        xSpeed *= InterfaceConstants.slowFactor;
                        rot *= InterfaceConstants.slowFactor;
                    }
    
                    drivetrain.drive(ySpeed, xSpeed, rot, true);
                },
                drivetrain));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating something like a primary.a().whileTrue(new ExampleCommand());
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}