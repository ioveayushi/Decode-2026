package org.firstinspires.ftc.teamcode.parts

import com.qualcomm.robotcore.hardware.CRServo

enum class IntakeMode { OFF, IN, OUT }

class Intake(val intakeMotor: CRServo) {
    fun set(mode: IntakeMode) {
        when(mode) {
            IntakeMode.OFF -> intakeMotor.power = 0.0
            IntakeMode.IN -> intakeMotor.power = 1.0
            IntakeMode.OUT -> intakeMotor.power = -1.0
        }
    }
}