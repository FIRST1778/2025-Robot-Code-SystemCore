package org.chillout1778

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.geometry.struct.Pose2dStruct
import edu.wpi.first.math.geometry.struct.Pose3dStruct
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import org.chillout1778.subsystems.Arm
import org.chillout1778.subsystems.Elevator
import org.chillout1778.subsystems.Intake
import org.chillout1778.subsystems.Swerve
import kotlin.system.measureTimeMillis


object LoggingManager {
    private val robotPosePublisher = NetworkTableInstance.getDefault().getTable("Logging").getStructTopic("0 - Pose", Pose2dStruct()).publish()
    private val baseStagePublisher = NetworkTableInstance.getDefault().getTable("Logging").getStructTopic("1 - BaseStage", Pose3dStruct()).publish()
    private val firstStagePublisher= NetworkTableInstance.getDefault().getTable("Logging").getStructTopic("2 - FirstStage", Pose3dStruct()).publish()
    private val carriagePublisher = NetworkTableInstance.getDefault().getTable("Logging").getStructTopic("3 - Carriage", Pose3dStruct()).publish()
    private val armPublisher = NetworkTableInstance.getDefault().getTable("Logging").getStructTopic("4 - Arm", Pose3dStruct()).publish()
    private val intakePublisher = NetworkTableInstance.getDefault().getTable("Logging").getStructTopic("5 - Intake", Pose3dStruct()).publish()
    private val swervePublisher = NetworkTableInstance.getDefault().getTable("Logging").getStructArrayTopic("Swerve", SwerveModuleState.struct).publish()

    val baseStagePose = Pose3d(Translation3d(), Rotation3d(Math.toRadians(90.0), Math.toRadians(0.0), Math.toRadians(90.0)))
    var firstStagePose = Pose3d(Translation3d(), Rotation3d(Math.toRadians(90.0), Math.toRadians(0.0), Math.toRadians(90.0)))
    var carriagePose = Pose3d(Translation3d(), Rotation3d(Math.toRadians(90.0), Math.toRadians(0.0), Math.toRadians(90.0)))
    val baseArmTranslation = Translation3d(Units.inchesToMeters(3.5), 0.0, Units.inchesToMeters(8.75))
    val intakePose = Pose3d(Translation3d(Units.inchesToMeters(13.0), 0.0, Units.inchesToMeters(6.75)), Rotation3d(Math.toRadians(90.0), Math.toRadians(0.0), Math.toRadians(90.0)))

    init{
        baseStagePublisher.set(baseStagePose) // only needs to happen once, it doesn't move
        DriverStation.startDataLog(DataLogManager.getLog())
    }

    val times: MutableList<Long> = mutableListOf()
    fun update(){
        times.clear()
        val elevatorHeight = Elevator.height
        val armAngle = MathUtil.angleModulus(Arm.position + Math.PI)
        times.add(measureTimeMillis {
            robotPosePublisher.set(Swerve.estimatedPose)
        })
        times.add(measureTimeMillis {
            firstStagePublisher.set(
                firstStagePose +
                        Transform3d(
                            0.0,
                            (elevatorHeight - Units.inchesToMeters(26.0)).coerceAtLeast(0.0),
                            0.0,
                            Rotation3d()
                        )
            )
        })
        times.add(measureTimeMillis {
            carriagePublisher.set(carriagePose + Transform3d(0.0, elevatorHeight, 0.0, Rotation3d()))
        })
        times.add(measureTimeMillis {
            armPublisher.set(
                Pose3d(
                    Translation3d(
                        baseArmTranslation.x,
                        baseArmTranslation.y,
                        elevatorHeight + baseArmTranslation.z
                    ),
                    Rotation3d(Math.PI/2, armAngle, Math.PI/2)
                )
            )
        })
        times.add(measureTimeMillis {
            intakePublisher.set(intakePose.rotateBy(Rotation3d(Intake.angle, 0.0, 0.0)))
        })
        times.add(measureTimeMillis {
            swervePublisher.set(Swerve.moduleStates)
        })

    }
}