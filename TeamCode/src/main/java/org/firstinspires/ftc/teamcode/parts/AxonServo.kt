package org.firstinspires.ftc.teamcode.parts

import androidx.core.math.MathUtils.clamp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

class AxonServo(val servo: CRServo, val analog: AnalogInput) {
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
    fun updatePosition() {
        if (abs(position - targetPosition) >= 0.05) {
            servo.power = sign(targetPosition - position) * (targetPosition - position).pow(2)
        } else {
            servo.power = 0.0
        }
    }
}