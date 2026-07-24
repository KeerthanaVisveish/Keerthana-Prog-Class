package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmHW;
import org.firstinspires.ftc.teamcode.subsystems.Lift.LiftHW;
import org.firstinspires.ftc.teamcode.subsystems.collector.CollectorChallengeAnswerKey;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretAnswerKey;

public class CommandsList {

    public static Command collectorIntake(CollectorChallengeAnswerKey collector) {
        return new InstantCommand(() -> collector.setIntakeState(CollectorChallengeAnswerKey.IntakeState.INTAKE));
    }

    public static Command collectorOff(CollectorChallengeAnswerKey collector) {
        return new InstantCommand(() -> collector.setIntakeState(CollectorChallengeAnswerKey.IntakeState.OFF));
    }

    public static Command turretToCenter(TurretAnswerKey turret) {
        return new InstantCommand(() -> {
            turret.setTurretState(TurretAnswerKey.TurretState.POINT_AT_ANGLE);
            turret.setTargetAngle(0);
        });
    }

    public static Command liftToTargetPosition(LiftHW lift, double position) {
        return new InstantCommand(() -> lift.setTargetPosition(position));
    }

    public static Command armToTargetAngle(ArmHW arm, double angleRad) {
        return new InstantCommand(() -> arm.setTargetAngle(angleRad));
    }

    public static Command armTo90Degrees(ArmHW arm) {
        return new InstantCommand(() -> arm.setTargetAngle(Math.toRadians(90)));
    }
}