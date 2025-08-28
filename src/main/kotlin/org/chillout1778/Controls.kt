package org.chillout1778

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.chillout1778.subsystems.Arm
import org.chillout1778.subsystems.Elevator
import org.chillout1778.subsystems.Superstructure
import kotlin.math.abs

object Controls {
    private val driverController = CommandGenericHID(0)
    val operatorController = CommandXboxController(1)

    enum class AlignMode { None, ReefAlign, TroughAlign, AlgaeAlign, BargeAlign }

    data class DriveInputs(
        val forward: Double,
        val left: Double,
        val rotation: Double,
        val deadzone: Double,
        val alignMode: AlignMode,
    ){
        val nonZero: Boolean get() = abs(forward) > deadzone || abs(left) > deadzone || abs(rotation) > deadzone
        val redFlipped: DriveInputs get() = this.copy(forward = -forward, left = -left)
    }

    val emptyInputs = DriveInputs(0.0, 0.0, 0.0, 0.0, AlignMode.None)

    val driverInputs get() = DriveInputs(
        forward = driverController.getRawAxis(1),
        left = -driverController.getRawAxis(0),
        rotation = -driverController.getRawAxis(4),
        deadzone = 0.05,
        alignMode =
            if (wantBargeAutoAlign) AlignMode.BargeAlign
            else if (wantCoralAutoAlign && superstructureInputs.wantedScoringLevel != Superstructure.ScoringLevel.TROUGH) AlignMode.ReefAlign
            else if(wantCoralAutoAlign && superstructureInputs.wantedScoringLevel == Superstructure.ScoringLevel.TROUGH) AlignMode.TroughAlign
            else if(wantAlgaeAutoAlign && superstructureInputs.wantGetAlgae && !Arm.hasObject) AlignMode.AlgaeAlign
            else AlignMode.None
    )
    val operatorInputs: DriveInputs get(){
        return DriveInputs(
            forward = -operatorController.hid.leftY,
            left = -operatorController.hid.leftX,
            rotation = -operatorController.hid.rightX,
            deadzone = 0.1,
            alignMode =
                if (wantBargeAutoAlign) AlignMode.BargeAlign
                else if (wantCoralAutoAlign && superstructureInputs.wantedScoringLevel != Superstructure.ScoringLevel.TROUGH) AlignMode.ReefAlign
                else if(wantCoralAutoAlign && superstructureInputs.wantedScoringLevel == Superstructure.ScoringLevel.TROUGH) AlignMode.TroughAlign
                else if(wantAlgaeAutoAlign && superstructureInputs.wantGetAlgae && !Arm.hasObject) AlignMode.AlgaeAlign
                else AlignMode.None
        )
    }

    val wantCoralAutoAlign get() = superstructureInputs.wantExtend &&
            Superstructure.state != Superstructure.State.AlgaeRest &&
            Superstructure.state != Superstructure.State.PreProcessor &&
            Superstructure.state != Superstructure.State.ScoreProcessor
    val wantAlgaeAutoAlign get() = superstructureInputs.wantGetAlgae && Arm.atSetpoint && Elevator.atSetpoint
    val wantBargeAutoAlign get() = false

    val wantOffsetArmPositive get() = operatorController.getRawAxis(0) > 0.9 && operatorController.hid.leftStickButton
    val wantOffsetArmNegative get() = operatorController.getRawAxis(0) < -0.9 && operatorController.hid.leftStickButton

    var lastScoringLevel = Superstructure.ScoringLevel.TROUGH
    val superstructureInputs: Superstructure.SuperstructureInputs get() {
        val level = when (operatorController.hid.pov) {
            1 -> Superstructure.ScoringLevel.L4
            8 -> Superstructure.ScoringLevel.L3
            4 -> Superstructure.ScoringLevel.L2
            2 -> Superstructure.ScoringLevel.TROUGH
            else -> lastScoringLevel
        }
        lastScoringLevel = level
        return Superstructure.SuperstructureInputs(
            wantExtend = operatorController.getRawAxis(2) > .5,
            wantScore = (driverController.getRawAxis(4) > .5 || operatorController.hid.aButton),
                    /*&& (Swerve.withinScoringTolerance || level == Superstructure.ScoringLevel.TROUGH)*/ //TODO: Check drone input, operator is backup
            wantGroundIntake = operatorController.getRawAxis(3) > .5,
            wantArmSourceIntake = operatorController.hid.bButton,
            wantedScoringLevel = level,
            wantGetAlgae = operatorController.hid.rightBumperButton,
            wantDescoreAlgae = false,
            wantResetSuperstructure = operatorController.hid.startButton,
            wantSourceIntake = false,
            wantScoreProcessor = operatorController.hid.leftBumperButton,
            wantAlgaeGroundIntake = operatorController.hid.xButton
//            wantPopsiclePickup = false//operatorController.hid.l1Button
        )
    }
}
