package org.firstinspires.ftc.teamcode.parts

import com.qualcomm.robotcore.hardware.CRServo

class Lift(val leftLift: CRServo, val rightLift: CRServo) : Updatable {
    private var power = 0.0

    fun setPower(p: Double) {
        power = p.coerceIn(-1.0, 1.0)
    }

    fun up() {
        power = 1.0
    }

    fun down() {
        power = -1.0
    }

    fun stop() {
        power = 0.0
    }

    override fun update() {
        leftLift.power = power
        rightLift.power = power
    }
}