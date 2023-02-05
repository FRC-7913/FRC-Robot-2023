package frc7913.robot

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

object PortConstants {
    const val leftFrontMotor = 2
    const val leftRearMotor = 3

    const val rightFrontMotor = 0
    const val rightRearMotor = 1
}

// TODO: Put this declaration in a more relevant place
data class LimelightTransform(
    val translationX: Double,
    val translationY: Double,
    val translationZ: Double,
    val rotationX: Double,
    val rotationY: Double,
    val rotationZ: Double,
) {
    constructor() : this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    fun toArray() = DoubleArray(6) {
        when (it) {
            0 -> translationX
            1 -> translationY
            2 -> translationZ
            3 -> rotationX
            4 -> rotationY
            5 -> rotationZ
            else -> 0.0 // This line shouldn't run, but is necessary for an exhaustive "when" statement
        }
    }

    companion object {
        infix fun from(array: DoubleArray) =
            LimelightTransform(
                array[0],
                array[1],
                array[2],
                array[3],
                array[4],
                array[5]
            )
    }
}
