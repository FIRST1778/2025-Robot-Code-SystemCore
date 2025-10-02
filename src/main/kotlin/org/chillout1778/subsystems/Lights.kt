package org.chillout1778.subsystems

import com.ctre.phoenix6.configs.LEDConfigs
import com.ctre.phoenix6.controls.*
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.AnimationDirectionValue
import com.ctre.phoenix6.signals.LarsonBounceValue
import com.ctre.phoenix6.signals.RGBWColor
import com.ctre.phoenix6.signals.StripTypeValue
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Robot
import kotlin.math.abs

object Candle: SubsystemBase() {
    val CANDLE_CAN_BUS = "can_s3"

    private var leds: CANdle = CANdle(Constants.CanIds.CANDLE, CANDLE_CAN_BUS)

    var lightsTimer = Timer()

    init {
        leds.configurator.apply(LEDConfigs().withStripType(StripTypeValue.RGB))

        lightsTimer.reset()
        lightsTimer.start()
    }

    override fun periodic() {
        val status = getDisabledStatus()
        if (Robot.isEnabled) {
            if (RobotController.isBrownedOut()) {
                writeLedColor(Ranges.Strip, LedColors.PoopBrown)
            } else if (!status.CANHealthy) {
                writeLedColor(Ranges.Strip, LedColors.BadRed)
            } else if (Swerve.isAligned) {
//                nuclearRats(3.0,
//                    if (Superstructure.inputs.wantedScoringLevel == Superstructure.ScoringLevel.TROUGH)
//                        LedColors.LightBlue
//                    else
//                        LedColors.DarkBlue)
                blinkLed(Ranges.Strip, Constants.LEDs.ALLIGNMENT_BLINK_SPEED, LedColors.GoodGreen)
            } else if (Arm.hasObject) {
                writeLedColor(Ranges.Strip, LedColors.PureWhite)
            } else if (Intake.hasCoral) {
                writeLedColor(Ranges.Strip, LedColors.LightBlue)
            } else {
                writeLedColor(Ranges.Strip, LedColors.DarkBlue)
            }
            if (Arm.isArmStuck) {
                blinkLed(Ranges.Cross, Constants.LEDs.ARM_STUCK_BLINK_SPEED, LedColors.WarningYellow)
            }
        } else {
            writeDisabledStatus(status)
            disabledAnimations()
        }
    }

    data class RobotStatus(
        val CANHealthy : Boolean,
        val armCorrectOrientation: Boolean,
        val batteryVoltage: Double,
        val camerasConnected: Boolean,
        val brakeMode: Boolean,
        val intakeHasCoral: Boolean,
    )

    enum class LedColors(val color: RGBWColor) {
        GoodGreen(RGBWColor(0,255,0,0)),
        BadRed(RGBWColor(255, 0, 0, 0)),
        WarningYellow(RGBWColor(255, 255, 0, 0)),
        PureWhite(RGBWColor(255, 255, 255, 0)),
        LightBlue(RGBWColor(0, 175, 255, 0)),
        DarkBlue(RGBWColor(0, 0, 255, 0)),
        PoopBrown(RGBWColor(160,82,45, 0)),
        SpaceBlack(RGBWColor(0,0,0,0)),
    }

    enum class Ranges(val start: Int, val end: Int) {
        Whole(0, Constants.LEDs.LAST_ID),
        Strip(8, Constants.LEDs.LAST_ID),
        Right(8, 39),
        Cross(40, 59),
        Left(60, Constants.LEDs.LAST_ID),
        Candle(0,7),
    }

    fun getDisabledStatus(): RobotStatus {
//        try {
//            volts = Robot.pdh.voltage
//        } catch (_: Exception) {}
        return RobotStatus(
            CANHealthy = Arm.armPivotMotor.isConnected && Arm.rollerMotor.isConnected,
            armCorrectOrientation = abs(Arm.closeClampedPosition()) == 0.5,
            batteryVoltage = 12.7, //temporary disabled
            camerasConnected = Vision.allConnected(),
            brakeMode = !Robot.wasCoastModeEnabled,
            intakeHasCoral = Intake.hasCoral,
        )
    }


    fun writeDisabledStatus(state: RobotStatus) {
        writeLedColor(Ranges.Cross, LedColors.DarkBlue)

        writeLedRange(47, 48, DisabledStatusIndicator(state.camerasConnected))

        writeLedRange(45, 46, DisabledStatusIndicator(state.CANHealthy))

        writeLedRange(43, 44, DisabledStatusIndicator(state.armCorrectOrientation))

        writeLedRange(41, 42, DisabledStatusIndicator(state.brakeMode))

        writeLedRange(39, 40, DisabledStatusIndicator(!state.intakeHasCoral))
    }

    fun disabledAnimations() {
        if (Robot.wasEnabledLED) {
            if (lightsTimer.get() <= 5) {
                leds.setControl(RainbowAnimation(Ranges.Strip.start, Ranges.Strip.end).withFrameRate(2.0))
            } else {
                Robot.wasEnabledLED = false
                lightsTimer.restart()
            }
        } else {
            if (lightsTimer.get() <= 2) {
                leds.setControl(
                    ColorFlowAnimation(Ranges.Left.start, Ranges.Left.end)
                        .withColor(LedColors.LightBlue.color)
                        .withFrameRate(0.5)
                        .withDirection(AnimationDirectionValue.Forward))

                leds.setControl(
                    ColorFlowAnimation(Ranges.Right.start, Ranges.Right.end)
                        .withColor(LedColors.LightBlue.color)
                        .withFrameRate(0.5)
                        .withDirection(AnimationDirectionValue.Backward))

            } else if (lightsTimer.get() % 10.0 <= 1.0) {
                leds.setControl(
                    LarsonAnimation(Ranges.Left.start, Ranges.Left.end)
                        .withColor(LedColors.LightBlue.color)
                        .withFrameRate(1.0)
                        .withBounceMode(LarsonBounceValue.Front))

                leds.setControl(
                    LarsonAnimation(Ranges.Right.start, Ranges.Right.end)
                        .withColor(LedColors.LightBlue.color)
                        .withFrameRate(1.0)
                        .withBounceMode(LarsonBounceValue.Front))

            } else {
                FireAnimation(Ranges.Left.start, Ranges.Left.end)
                    .withFrameRate(1.0)
                    .withDirection(AnimationDirectionValue.Forward)

                FireAnimation(Ranges.Right.start, Ranges.Right.end)
                    .withFrameRate(1.0)
                    .withDirection(AnimationDirectionValue.Backward)
            }
        }
    }

    fun DisabledStatusIndicator(b: Boolean) = if (b) LedColors.DarkBlue else LedColors.BadRed


    fun writeLedColor(range: Ranges, c: LedColors) {
        leds.setControl(SolidColor(range.start, range.end).withColor(c.color))
    }

    fun writeLedRange(start: Int, end: Int, c: LedColors) {
        leds.setControl(SolidColor(start, end).withColor(c.color))
    }

    fun blinkLed(range: Ranges, freq: Double, c: LedColors) {
        leds.setControl(StrobeAnimation(range.start, range.end).withColor(c.color).withFrameRate(freq))
    }

    fun slotClear() {
        for (i in 0..7) {
            leds.setControl(EmptyAnimation(i))
        }
    }

    object nuclearRats {
        //In honor of OG nuclearRats
        fun feed() {}
        fun pet() {}
    }
}