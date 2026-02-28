package org.firstinspires.ftc.teamcode

import android.graphics.Color
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.TouchSensor
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.tan
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name = "BasicOpMode1_Flywheel")
class BasicOpMode1_Flywheel : com.qualcomm.robotcore.eventloop.opmode.LinearOpMode() {

    private lateinit var flywheelLeft: DcMotorEx
    private lateinit var frontLeft: DcMotor
    private lateinit var frontRight: DcMotor
    private lateinit var backLeft: DcMotor
    private lateinit var backRight: DcMotor
    private lateinit var wheelServo: CRServo

    private lateinit var spindexer: CRServo
    private lateinit var spindexerAnalog: AnalogInput
    private lateinit var spdxColor: ColorSensor
    private lateinit var spdxDist: DistanceSensor
    private lateinit var mag: TouchSensor

    private lateinit var leftLift: CRServo
    private lateinit var rightLift: CRServo

    private lateinit var prism: ServoImplEx
    private lateinit var flyLight: ServoImplEx

    private val prismColorsUs = intArrayOf(1050, 1200, 1350, 1500, 1650, 1800, 1940)
    private val prismGreenIndex = 2
    private val prismPurpleIndex = 5
    private val prismRedIndex = 0
    private fun usToPos(us: Int): Double { return ((us - 500) / 2000.0).coerceIn(0.0, 1.0) }

    private var flywheelOn = true

    private var autoVelEnabled = true
    private var lastAutoVelToggle = false

    private var targetVelCmd = 2200.0
    private var targetVel = 2200.0

    private val velNudgeRate = 2200.0

    private var velOffset = 0.0
    private val velOffsetStep = 25.0

    private val minVel = 0.0
    private val maxVel = 4500.0

    private val velSlew = 7000.0

    private val maxVelEst = 4000.0
    private val kP_vel = 0.00030

    private val powerSlewPerSec = 3.0
    private var flyPower = 0.0

    private val LIMELIGHT_NAME = "limelight"
    private val PIPELINE_INDEX = 0

    private val camHeightCm = 36.02493
    private val tagHeightCm = 74.93
    private val camMountDeg = 20.0

    private var distFiltCm = 0.0
    private val distAlpha = 0.25
    private var lastDistSeenMs = 0L
    private val holdDistMs = 250L

    private var lastSeenTimeMs = 0L
    private val holdLastSeenMs = 250L
    private var lastTxDeg = 0.0
    private var lastTyDeg = 0.0

    private val TURRET_MOTOR_NAME = "tt"
    private val kP_tt = 0.02
    private val minPower = 0.012
    private val maxPower_tt = 1.0
    private val deadbandDeg = 1.0
    private val manualPower = 0.35

    private lateinit var limelight: Limelight3A
    private lateinit var turret: DcMotor

    private var autoAimEnabled = true
    private var lastToggleBtn = false

    private var turretHoming = false
    private var lastResetBtn = false
    private val turretHomePower = 0.55
    private val turretHomeTol = 15

    private var flCmd = 0.0
    private var frCmd = 0.0
    private var blCmd = 0.0
    private var brCmd = 0.0
    private val accelRate = 8.0
    private val brakeRate = 18.0

    private var spdxIndex = 0
    private var spdxMoving = false
    private var spdxMoveTimer = 0.0
    private var spdxBrakeTimer = 0.0
    private var spdxDir = 1
    private var spdxStage = 0

    private val spdxSpinPower = 0.18
    private val spdxTimeout = 0.85
    private val spdxBrakePower = 0.28
    private val spdxBrakeTime = 0.08

    private var lastRightBumper = false
    private var lastLeftBumper = false

    private var magStable = false
    private var magDebounceTimer = 0.0
    private val magDebounceS = 0.006

    private var totalTime = 0.0
    private var lastTime = 0L

    private var ballPresent = false
    private var hueFilt = 0f

    private fun clamp(v: Double, lo: Double, hi: Double): Double = max(lo, min(hi, v))

    private fun rampTowards(current: Double, target: Double, maxDelta: Double): Double {
        val delta = (target - current).coerceIn(-maxDelta, maxDelta)
        return current + delta
    }

    private fun rampWithBrake(current: Double, target: Double, dt: Double): Double {
        val rate = if (abs(target) < abs(current)) brakeRate else accelRate
        return rampTowards(current, target, rate * dt)
    }

    private fun lerp(a: Double, b: Double, t: Double) = a + (b - a) * t

    private fun interp1D(x: Double, xs: DoubleArray, ys: DoubleArray): Double {
        if (xs.isEmpty() || ys.isEmpty() || xs.size != ys.size) return ys.firstOrNull() ?: 0.0
        if (x <= xs.first()) return ys.first()
        if (x >= xs.last()) return ys.last()
        for (i in 0 until xs.size - 1) {
            val x0 = xs[i]
            val x1 = xs[i + 1]
            if (x in x0..x1) {
                val t = (x - x0) / (x1 - x0)
                return lerp(ys[i], ys[i + 1], t)
            }
        }
        return ys.last()
    }

    private fun slew(current: Double, target: Double, dt: Double, ratePerSec: Double): Double {
        val maxStep = ratePerSec * dt
        val delta = (target - current).coerceIn(-maxStep, maxStep)
        return current + delta
    }

    private fun startSpdxMove(dir: Int) {
        if (spdxMoving) return
        spdxDir = if (dir >= 0) 1 else -1
        spdxMoving = true
        spdxMoveTimer = 0.0
        spdxBrakeTimer = 0.0
        val startPressed = mag.isPressed
        spdxStage = if (startPressed) 0 else 1
    }

    override fun runOpMode() {

        limelight = hardwareMap.get(Limelight3A::class.java, LIMELIGHT_NAME)
        turret = hardwareMap.get(DcMotor::class.java, TURRET_MOTOR_NAME)
        turret.direction = Direction.REVERSE
        turret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turret.mode = DcMotor.RunMode.RUN_USING_ENCODER

        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(PIPELINE_INDEX)
        limelight.start()

        flywheelLeft = hardwareMap.get(DcMotorEx::class.java, "flywheel")
        flywheelLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flywheelLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        wheelServo = hardwareMap.get(CRServo::class.java, "intk")

        spindexer = hardwareMap.get(CRServo::class.java, "spdx")
        spindexerAnalog = hardwareMap.get(AnalogInput::class.java, "axen")
        spdxColor = hardwareMap.get(ColorSensor::class.java, "css")
        spdxDist = hardwareMap.get(DistanceSensor::class.java, "cs")
        mag = hardwareMap.get(TouchSensor::class.java, "mag")

        leftLift = hardwareMap.get(CRServo::class.java, "leftLift")
        rightLift = hardwareMap.get(CRServo::class.java, "rightLift")
        rightLift.direction = Direction.FORWARD
        leftLift.direction = Direction.FORWARD

        prism = hardwareMap.get(ServoImplEx::class.java, "prism")
        prism.pwmRange = PwmControl.PwmRange(500.0, 2500.0)

        frontLeft = hardwareMap.get(DcMotor::class.java, "lf")
        frontRight = hardwareMap.get(DcMotor::class.java, "rf")
        backLeft = hardwareMap.get(DcMotor::class.java, "lb")
        backRight = hardwareMap.get(DcMotor::class.java, "rb")

        frontLeft.direction = Direction.REVERSE
        backLeft.direction = Direction.REVERSE
        frontRight.direction = Direction.REVERSE
        backRight.direction = Direction.FORWARD

        frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        prism.position = usToPos(prismColorsUs[prismRedIndex])

        waitForStart()

        lastTime = System.nanoTime()
        totalTime = 0.0
        magStable = mag.isPressed
        lastSeenTimeMs = System.currentTimeMillis()
        lastDistSeenMs = System.currentTimeMillis()

        while (opModeIsActive()) {

            val now = System.nanoTime()
            val dt = (now - lastTime) / 1e9
            lastTime = now
            if (dt <= 0.0 || dt >= 0.2) continue
            totalTime += dt

            val magRaw = mag.isPressed
            if (magRaw != magStable) {
                magDebounceTimer += dt
                if (magDebounceTimer >= magDebounceS) {
                    magStable = magRaw
                    magDebounceTimer = 0.0
                }
            } else {
                magDebounceTimer = 0.0
            }
            val magPressed = magStable

            val y = -gamepad1.left_stick_y.toDouble()
            val x = gamepad1.left_stick_x.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()

            val denominator = maxOf(abs(y) + abs(x) + abs(rx), 1.0)

            val flTarget = (y + x + rx) / denominator
            val blTarget = (y - x + rx) / denominator
            val frTarget = (y - x - rx) / denominator
            val brTarget = (y + x - rx) / denominator

            val drivespeed = 1.0

            val flOut = flTarget * drivespeed
            val blOut = blTarget * drivespeed
            val frOut = frTarget * drivespeed
            val brOut = brTarget * drivespeed

            flCmd = rampWithBrake(flCmd, flOut, dt)
            blCmd = rampWithBrake(blCmd, blOut, dt)
            frCmd = rampWithBrake(frCmd, frOut, dt)
            brCmd = rampWithBrake(brCmd, brOut, dt)

            frontLeft.power = flCmd
            backLeft.power = blCmd
            frontRight.power = frCmd
            backRight.power = brCmd

            flywheelOn = !gamepad1.a

            val result: LLResult? = limelight.getLatestResult()

            var hasTargetNow = false
            var txDeg = 0.0
            var tyDeg = 0.0

            if (result != null) {
                hasTargetNow = result.isValid
                txDeg = result.tx
                tyDeg = result.ty
                if (hasTargetNow) {
                    lastTxDeg = txDeg
                    lastTyDeg = tyDeg
                    lastSeenTimeMs = System.currentTimeMillis()
                }
            }

            val nowMs = System.currentTimeMillis()
            val recentlySeen = (nowMs - lastSeenTimeMs) <= holdLastSeenMs

            val toggleBtn = gamepad1.x
            if (toggleBtn && !lastToggleBtn) autoAimEnabled = !autoAimEnabled
            lastToggleBtn = toggleBtn

            val resetBtn = gamepad1.y
            if (resetBtn && !lastResetBtn) {
                turretHoming = true
                turret.mode = DcMotor.RunMode.RUN_TO_POSITION
                turret.targetPosition = 0
                turret.power = turretHomePower
            }
            lastResetBtn = resetBtn

            val turretCmdPower = if (turretHoming) {
                val err = turret.targetPosition - turret.currentPosition
                if (abs(err) <= turretHomeTol) {
                    turretHoming = false
                    turret.power = 0.0
                    turret.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    0.0
                } else {
                    turretHomePower
                }
            } else if (autoAimEnabled) {
                val txToUse = when {
                    hasTargetNow -> txDeg
                    recentlySeen -> lastTxDeg
                    else -> 0.0
                }

                if (abs(txToUse) <= deadbandDeg) {
                    0.0
                } else {
                    var pwr = kP_tt * txToUse
                    pwr = if (pwr > 0) pwr + minPower else pwr - minPower
                    clamp(pwr, -maxPower_tt, maxPower_tt)
                }
            } else {
                val pwr =
                    if (gamepad1.dpad_right) manualPower
                    else if (gamepad1.dpad_left) -manualPower
                    else 0.0
                clamp(pwr, -manualPower, manualPower)
            }

            if (!turretHoming) turret.power = turretCmdPower

            if (hasTargetNow) {
                val angleRad = Math.toRadians(camMountDeg + tyDeg)
                val tanVal = tan(angleRad)
                if (tanVal > 0.01) {
                    val d = (tagHeightCm - camHeightCm) / tanVal
                    if (d.isFinite() && d > 0.0) {
                        distFiltCm = (1.0 - distAlpha) * distFiltCm + distAlpha * d
                        lastDistSeenMs = nowMs
                    }
                }
            }
            val distFresh = (nowMs - lastDistSeenMs) <= holdDistMs

            val autoBtn = gamepad2.x
            if (autoBtn && !lastAutoVelToggle) autoVelEnabled = !autoVelEnabled
            lastAutoVelToggle = autoBtn

            // Trim (gamepad2 dpad left/right)
            if (gamepad2.dpad_right) velOffset += velOffsetStep
            if (gamepad2.dpad_left) velOffset -= velOffsetStep
            velOffset = velOffset.coerceIn(-800.0, 800.0)

            // Manual nudge (continuous): gamepad2 dpad up/down
            val upNow = gamepad2.dpad_up
            val dnNow = gamepad2.dpad_down
            val manualDelta = (if (upNow) 1.0 else 0.0) - (if (dnNow) 1.0 else 0.0)
            if (manualDelta != 0.0) {
                targetVelCmd += manualDelta * velNudgeRate * dt
                targetVelCmd = targetVelCmd.coerceIn(minVel, maxVel)
            }

            val distPts = doubleArrayOf(80.0, 120.0, 160.0, 180.0, 200.0, 320.0)
            val velPts  = doubleArrayOf(1667.0, 1843.6459, 1879.0, 2007.901, 2080.0, 2390.0)

            if (autoVelEnabled && distFresh) {
                val v = interp1D(distFiltCm, distPts, velPts) + velOffset
                targetVelCmd = v.coerceIn(minVel, maxVel)
            }

            targetVel = slew(targetVel, targetVelCmd, dt, velSlew)
            val measuredVel = abs(flywheelLeft.velocity)

            val ff = (targetVel / maxVelEst).coerceIn(0.0, 1.0)

            val err = targetVel - measuredVel
            val corr = kP_vel * err

            var out = (ff + corr).coerceIn(0.0, 1.0)

            if (!flywheelOn) out = 0.0

            flyPower = rampTowards(flyPower, out, powerSlewPerSec * dt)
            flywheelLeft.power = -flyPower

            var wheelServoPower = 0.0
            if (gamepad2.y) wheelServoPower = -1.0
            if (gamepad2.b) wheelServoPower = 1.0
            wheelServo.power = wheelServoPower

            rightLift.power = 0.0
            leftLift.power = 0.0

            val distCmBall = spdxDist.getDistance(DistanceUnit.CM)
            val distValid = distCmBall.isFinite() && distCmBall > 0.0
            val aa = spdxColor.alpha()

            val rr = spdxColor.red()
            val gg = spdxColor.green()
            val bb = spdxColor.blue()

            val hsv = FloatArray(3)
            Color.RGBToHSV(rr, gg, bb, hsv)
            val hue = hsv[0]
            val sat = hsv[1]
            val value = hsv[2]

            val inDist = 3.4
            val outDist = 4.3
            val inAlpha = 28
            val outAlpha = 18

            val closeNow = distValid && distCmBall < inDist
            val farNow = !distValid || distCmBall > outDist

            ballPresent = if (!ballPresent) {
                closeNow && aa > inAlpha
            } else {
                !(farNow || aa < outAlpha)
            }

            val hueValid = ballPresent && aa >= inAlpha && sat >= 0.12f && value >= 0.12f
            if (hueValid) hueFilt = 0.8f * hueFilt + 0.2f * hue else hueFilt = hue

            val ballColor = if (!hueValid) {
                "unknown"
            } else if (hueFilt >= 190f) {
                "purple"
            } else {
                "green"
            }

            prism.position = usToPos(
                prismColorsUs[
                    if (!ballPresent) prismRedIndex
                    else if (ballColor == "green") prismGreenIndex
                    else prismPurpleIndex
                ]
            )

            val rightBumper = gamepad2.right_bumper
            val leftBumperBtn = gamepad2.left_bumper

            if (rightBumper && !lastRightBumper) startSpdxMove(1)
            if (leftBumperBtn && !lastLeftBumper) startSpdxMove(-1)

            lastRightBumper = rightBumper
            lastLeftBumper = leftBumperBtn

            if (spdxMoving) {
                if (spdxStage != 2) spdxMoveTimer += dt

                if (spdxStage == 0) {
                    spindexer.power = spdxSpinPower * spdxDir
                    if (!magPressed || spdxMoveTimer >= 0.35) {
                        spdxStage = 1
                        spdxMoveTimer = 0.0
                    }
                } else if (spdxStage == 1) {
                    spindexer.power = spdxSpinPower * spdxDir
                    if (magPressed || spdxMoveTimer >= spdxTimeout) {
                        spdxStage = 2
                        spdxBrakeTimer = 0.0
                    }
                } else {
                    spdxBrakeTimer += dt
                    spindexer.power = -spdxBrakePower * spdxDir
                    if (spdxBrakeTimer >= spdxBrakeTime) {
                        spindexer.power = 0.0
                        spdxMoving = false
                        spdxIndex = (spdxIndex + spdxDir + 3) % 3
                    }
                }
            } else {
                spindexer.power = 0.0
            }

            telemetry.addData("autoVel", autoVelEnabled)
            telemetry.addData("gp2_up", gamepad2.dpad_up)
            telemetry.addData("gp2_dn", gamepad2.dpad_down)
            telemetry.addData("velOffset", velOffset)

            telemetry.addData("distFiltCm", distFiltCm)
            telemetry.addData("distFresh", distFresh)

            telemetry.addData("targetVelCmd", targetVelCmd)
            telemetry.addData("targetVel", targetVel)
            telemetry.addData("measuredVel", measuredVel)

            telemetry.addData("ff", ff)
            telemetry.addData("err", err)
            telemetry.addData("flyPower", flyPower)

            telemetry.addData("LL valid", hasTargetNow)
            telemetry.addData("tx", txDeg)
            telemetry.addData("ty", tyDeg)

            telemetry.addData("turretPower", turretCmdPower)
            telemetry.addData("turretPos", turret.currentPosition)
            telemetry.addData("turretHoming", turretHoming)

            telemetry.addData("ballPresent", ballPresent)
            telemetry.addData("ballColor", ballColor)
            telemetry.update()
        }
    }
}