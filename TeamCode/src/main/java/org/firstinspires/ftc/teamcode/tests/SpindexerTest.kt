package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Robot

@TeleOp(name = "SpindexerTest")
class SpindexerTest: LinearOpMode() {
    lateinit var robot: Robot

    override fun runOpMode() {
        robot = Robot(this)

//        robot.spindexer.home()

        waitForStart()

        while (opModeIsActive()) {
            robot.update()

            if (gamepad1.left_bumper && !robot.gamepadState1.left_bumper) {
                robot.spindexer.rotate(1)
            }

            robot.updateGamepadStates()

            robot.dashboardTelemetry.addData("servo power", robot.spindexer.servo.servo.power)
            robot.dashboardTelemetry.addData("servo target", robot.spindexer.servo.targetPosition)
            robot.dashboardTelemetry.addData("servo current", robot.spindexer.servo.position)
            robot.dashboardTelemetry.addData("servo analog", robot.spindexer.servo.analog.voltage)
            robot.dashboardTelemetry.update()

            telemetry.addData("servo power", robot.spindexer.servo.servo.power)
            telemetry.addData("servo target", robot.spindexer.servo.targetPosition)
            telemetry.addData("servo current", robot.spindexer.servo.position)
            telemetry.addData("servo analog", robot.spindexer.servo.analog.voltage)
            telemetry.update()
        }
    }
}