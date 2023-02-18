package frc7913.robot

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc7913.robot.commands.AutoModes
import frc7913.robot.commands.bindXboxCommands
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
object RobotContainer {
    private val autoModeChooser = SendableChooser<AutoModes>().apply {
        AutoModes.values().forEach { addOption(it.optionName, it) }
        setDefaultOption(AutoModes.default.optionName, AutoModes.default)
    }

    /** The command to run in autonomous. */
    val selectedAutonomousCommand: Command
        get() = autoModeChooser.selected?.command ?: AutoModes.default.command

    init
    {
        XboxController = CommandXboxController(0)
        configureButtonBindings()
        SmartDashboard.putData("Auto choices", autoModeChooser)
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID][edu.wpi.first.wpilibj.GenericHID] or one of its subclasses such
     * as [Joystick][edu.wpi.first.wpilibj.Joystick] or [XboxController][edu.wpi.first.wpilibj.XboxController],
     * and then passing it to a [JoystickButton][edu.wpi.first.wpilibj2.command.button.JoystickButton].
     */
    private fun configureButtonBindings() {
        bindXboxCommands(XboxController)
    }
}

lateinit var XboxController: CommandXboxController
