package frc7913.robot

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc7913.robot.commands.ExampleCommand

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
object RobotContainer
{
    private val autoModeChooser = SendableChooser<AutoMode>().apply {
        AutoMode.values().forEach { addOption(it.optionName, it) }
        setDefaultOption(AutoMode.default.optionName, AutoMode.default)
    }

    /**
     * A enumeration of the available autonomous modes.
     *
     * @param optionName The name for the [autoModeChooser] option.
     * @param command The [Command] to run for this mode.
     */
    private enum class AutoMode(val optionName: String, val command: Command)
    {
        // TODO: Replace with real auto modes and their corresponding commands
        CUSTOM_AUTO_1("Custom Auto Mode 1", ExampleCommand()),
        CUSTOM_AUTO_2("Custom Auto Mode 2", PrintCommand("Auto Mode 2")),
        ;

        companion object
        {
            /** The default auto mode. */
            val default = CUSTOM_AUTO_1
        }
    }

    /** The command to run in autonomous. */
    val selectedAutonomousCommand: Command
        get() = autoModeChooser.selected?.command ?: AutoMode.default.command

    init
    {
        configureButtonBindings()
        SmartDashboard.putData("Auto choices", autoModeChooser)
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID][edu.wpi.first.wpilibj.GenericHID] or one of its subclasses such
     * as [Joystick][edu.wpi.first.wpilibj.Joystick] or [XboxController][edu.wpi.first.wpilibj.XboxController],
     * and then passing it to a [JoystickButton][edu.wpi.first.wpilibj2.command.button.JoystickButton].
     */
    private fun configureButtonBindings()
    {
        // TODO: Add button to command mappings here.
        //       See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
    }
}