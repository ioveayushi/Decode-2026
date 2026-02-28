package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.parts.AxonServo
import org.firstinspires.ftc.teamcode.parts.Drive
import org.firstinspires.ftc.teamcode.parts.Intake
import org.firstinspires.ftc.teamcode.parts.Light
import org.firstinspires.ftc.teamcode.parts.Spindexer
import org.firstinspires.ftc.teamcode.parts.Turret

class Robot(opMode: OpMode) {
    var lf: DcMotor
    var lb: DcMotor
    var rf: DcMotor
    var rb: DcMotor
    var motors: Array<DcMotor>

    var flywheelMotor: DcMotorEx
    var intakeServo: CRServo
    var spindexerServo: AxonServo
    var spindexerColor: ColorSensor
    var spindexerDistance: DistanceSensor
    var spindexerMagnet: TouchSensor

    var turretMotor: DcMotor
    var limelight: Limelight3A

    var leftLiftServo: CRServo
    var rightLiftServo: CRServo

    var spindexerLight: Light

    var drive: Drive
    var intake: Intake
    var turret: Turret
    var spindexer: Spindexer

    init {
        val hardwareMap: HardwareMap = opMode.hardwareMap

        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)
        limelight.start()

        lf = hardwareMap.get(DcMotor::class.java, "lf")
        lb = hardwareMap.get(DcMotor::class.java, "lb")
        rf = hardwareMap.get(DcMotor::class.java, "rf")
        rb = hardwareMap.get(DcMotor::class.java, "rb")
        motors = arrayOf(lf, lb, rf, rb)

        for (motor in motors) {
            motor.power = 0.0
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        lf.direction = Direction.REVERSE
        lb.direction = Direction.REVERSE
        rf.direction = Direction.REVERSE
        rb.direction = Direction.FORWARD

        flywheelMotor = hardwareMap.get(DcMotorEx::class.java, "flywheel")
        flywheelMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flywheelMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        intakeServo = hardwareMap.get(CRServo::class.java, "intk")

        spindexerServo = AxonServo(hardwareMap.get(CRServo::class.java, "spdx"), hardwareMap.get(AnalogInput::class.java, "axen"))
        spindexerColor = hardwareMap.get(ColorSensor::class.java, "css")
        spindexerDistance = hardwareMap.get(DistanceSensor::class.java, "cs")
        spindexerMagnet = hardwareMap.get(TouchSensor::class.java, "mag")

        turretMotor = hardwareMap.get(DcMotor::class.java, "tt")
        turretMotor.direction = Direction.REVERSE
        turretMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turretMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        leftLiftServo = hardwareMap.get(CRServo::class.java, "leftLift")
        rightLiftServo = hardwareMap.get(CRServo::class.java, "rightLift")
        leftLiftServo.direction = Direction.FORWARD
        rightLiftServo.direction = Direction.FORWARD

        spindexerLight = Light(hardwareMap.get(Servo::class.java, "prism"))

        drive = Drive(lf, lb, rf, rb)
        intake = Intake(intakeServo)
        turret = Turret(limelight, turretMotor, flywheelMotor)
        spindexer = Spindexer(spindexerServo, spindexerDistance, spindexerColor, spindexerLight, spindexerMagnet)
    }
}