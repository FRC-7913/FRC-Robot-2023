package frc7913.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc7913.robot.subsystems.DriveSubsystem

enum class AutoModes(val optionName: String, val command: Command) {

    /* Conventions

    When declaring an autonomous mode, follow this format

        SIMPLE_INTERNAL_NAME(
            "Display Name For Driver's Station Dashboard",
            CommandToRun()
        )

    The internal name is referenced only in code, but should be understandable
    The display name is sent to the driver's station, so it should be descriptive but not too long
    The Command to run should, unless very simple (<3 lines), reference a command placed in another file

     */

    CUSTOM_AUTO_1(
        "Custom Auto Mode 1",
        ExampleCommand()
    ),
    CUSTOM_AUTO_2(
        "Custom Auto Mode 2",
        PrintCommand("Auto Mode 2")
    ),
    TEST_RAMSETE(
        "Ramsete Controller Test",
        DriveSubsystem.NavigateCommand(
            start = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),

            Translation2d(-0.5, 1.0),

            end = Pose2d(2.0, 2.0, Rotation2d.fromDegrees(90.0)),
        )
    )
    ;

    companion object {
        val default = CUSTOM_AUTO_1
    }
}
