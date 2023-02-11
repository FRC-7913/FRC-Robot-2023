package frc7913.robot.commands

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc7913.robot.subsystems.DriveSubsystem

/**
 * Binds commands to given XboxController
 *
 * @param xboxController the controller to bind to
 */
fun bindXboxCommands(xboxController: CommandXboxController) = xboxController.apply {
    // Add button to command mappings here.
    //  See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html

    /* Conventions

    When binding to a button, use the button name and then the binding type, with no prefix
        e.g. y().onTrue(...)
        e.g. povDown().whileTrue(...)

    When accessing variables, use xboxController.<variableName> for clarity of where the variable is from
        e.g. xboxController.leftX
        e.g. xboxController.getRawAxis(0)
     */

    y().onTrue(Commands.runOnce({ println("-----") }))

    x().onTrue(Commands.runOnce({ DriveSubsystem.printEncoders() }))

    b().onTrue(Commands.runOnce({ DriveSubsystem.resetEncoders() }))

    DriveSubsystem.defaultCommand = Commands.run(
        { DriveSubsystem.driveTrain.tankDrive(-xboxController.leftX, xboxController.rightX) },
        DriveSubsystem
    )
}
