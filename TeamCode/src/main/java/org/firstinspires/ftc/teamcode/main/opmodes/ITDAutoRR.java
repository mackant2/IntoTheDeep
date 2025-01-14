package org.firstinspires.ftc.teamcode.main.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.components.Arm;
import org.firstinspires.ftc.teamcode.main.components.Intake;
import org.firstinspires.ftc.teamcode.main.utils.ParsedHardwareMap;
import org.firstinspires.ftc.teamcode.main.utils.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "[NOT DONE] Auto With RR", group = "official")
public class ITDAutoRR extends LinearOpMode {
    public static Boolean cycleSamples = false;

    MecanumDrive drive;

    final double width = 12.3;
    final double length = 15.75;
    final double specimenSampleY = -12;
    final Vector2d[] specimenSamplePositions = {new Vector2d(45, specimenSampleY), new Vector2d(55, specimenSampleY), new Vector2d(61, specimenSampleY)};
    final Vector2d specimenHangPos = new Vector2d(8, -34);
    final Vector2d specimenPickupPos = new Vector2d(36, -56);

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(width / 2, -72 + length / 2, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);
        ParsedHardwareMap parsedHardwareMap = new ParsedHardwareMap(hardwareMap);
        parsedHardwareMap.flipDown.setPosition(Intake.FlipdownPosition.in);

        Robot robot = new Robot(this, parsedHardwareMap, false);

        robot.arm.RotateFourBar(0.5);
        robot.arm.GoToHeight(Arm.Height.DOWN);
        parsedHardwareMap.extender.setTargetPosition(30);
        parsedHardwareMap.wrist.setPosition(.2);
        parsedHardwareMap.claw.setPosition(Arm.ClawPosition.Closed);

        waitForStart();

        if (isStopRequested()) return;

        TrajectoryActionBuilder specimenHang = drive.actionBuilder(initialPose)
                .lineToY(-38.5);

        TrajectoryActionBuilder oneEighty = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(180));

        TrajectoryActionBuilder specimenBackup = specimenHang.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(0)
                .lineToXLinearHeading(34, 0)
                .setTangent(Math.PI / 2)
                .lineToY(-10)
                .setTangent(0)
                .lineToX(40)
                .setTangent(Math.PI / 2)
                .lineToY(-54)
                .lineToY(-10)
                .setTangent(0)
                .lineToX(50)
                .setTangent(Math.PI / 2)
                .lineToY(-60);


        Actions.runBlocking(new SequentialAction(
                //robot.arm.GoToSpecimenPosition(),
                specimenHang.build(),
                //robot.arm.HangSpecimen(),
                Sleep(500),
                //robot.arm.ReleaseSpecimen(),
                Sleep(500),
                //robot.arm.Reset(),
                specimenBackup.build()
        ));
        sleep(100000);
    }

    public Action Sleep(int milliseconds) {
        return new Sleep(milliseconds);
    }

    public class Sleep implements Action {
        int milliseconds;

        public Sleep(int milliseconds) {
            this.milliseconds = milliseconds;
        }

        @Override
        public boolean run (@NonNull TelemetryPacket telemetryPacket) {
            sleep(milliseconds);
            return false;
        }
    }
}