package frc7913.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand

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
    ;

    companion object {
        val default = CUSTOM_AUTO_1
    }
}
