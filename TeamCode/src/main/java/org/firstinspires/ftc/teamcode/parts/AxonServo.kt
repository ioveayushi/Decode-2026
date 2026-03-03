package org.firstinspires.ftc.teamcode.parts

import androidx.core.math.MathUtils.clamp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import org.firstinspires.ftc.teamcode.PIDController
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

class AxonServo(val servo: CRServo, val analog: AnalogInput, val pid: PIDController = PIDController(1.0, 0.0, 0.0)): Updatable {
    var tolerance = 0.01

    // Holds the current position of the servo
    var position: Double
        get() {
            // Return position [0-1]
            return analog.voltage / 3.3
        }
        set(value) {
            // Set target position because this stores current
            targetPosition = value
        }

    // Target position of the servo
    var targetPosition: Double = 0.35
        set(value) {
            // Make sure the value is in range
            field = clamp(value, 0.0, 1.0)
        }

    // Set the power so it moves towards the target position
    override fun update() {
        val error = targetPosition - position

        if (abs(error) > tolerance) {
            val power = pid.calculate(targetPosition, position)
            servo.power = clamp(power, -1.0, 1.0)
        } else {
            servo.power = 0.0
            pid.reset()
        }
    }
}