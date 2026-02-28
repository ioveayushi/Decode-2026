package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.parts.IntakeMode
import org.firstinspires.ftc.teamcode.util.GamepadState

@TeleOp(name = "BasicTest")
class BasicTest : LinearOpMode() {
    val robot = Robot(this)

    val gamepadState1: GamepadState = GamepadState()
    val gamepadState2: GamepadState = GamepadState()

    override fun runOpMode() {
        waitForStart()

        while(opModeIsActive()) {
            robot.turret.update()

            robot.drive.drive(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble(), gamepad1.right_stick_x.toDouble())

            val intakeMode: IntakeMode = if (gamepad2.y) IntakeMode.OUT else if (gamepad2.b) IntakeMode.IN else IntakeMode.OFF
            robot.intake.set(intakeMode)

            if (gamepad2.left_bumper && !gamepadState2.left_bumper) {
                robot.spindexer.startSpdxMove(-1)
            }

            if (gamepad2.right_bumper && !gamepadState2.right_bumper) {
                robot.spindexer.startSpdxMove(1)
            }

            gamepadState1.updateGamepadState(gamepad1)
            gamepadState2.updateGamepadState(gamepad2)
        }
    }
}