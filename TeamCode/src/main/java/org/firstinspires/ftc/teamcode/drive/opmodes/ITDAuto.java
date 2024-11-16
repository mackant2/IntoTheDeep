package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Logger;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "\uD83E\uDD16 ITDAuto", group = "*DRIVERS USE THIS*")
public class ITDAuto extends LinearOpMode {
    Intake intake;
    Logger logger = new Logger();
    Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        logger.Initialize(telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        arm = new Arm(drive, telemetry, gamepad1);

        intake = new Intake(drive, this, logger);

        Pose2d startPose = new Pose2d(0,0);
        drive.setPoseEstimate(startPose);

        double armLiftTime = 4;
        double intakeTime = 1.5;
        double outtakeTime = 0.8;
        int threeSamplesY = 26;

        Pose2d dropPose = new Pose2d(-16, 0, Math.toRadians((0)));
        Pose2d FirstSpot = new Pose2d(-10, 26, Math.toRadians((-90)));


        Trajectory toFirstSpot = drive.trajectoryBuilder(dropPose)
                .lineToSplineHeading(new Pose2d(-10, threeSamplesY, Math.toRadians(-90 )))
                .build();

        Trajectory toSecondSpot = drive.trajectoryBuilder(dropPose)
                .lineToSplineHeading(new Pose2d(-20, threeSamplesY, Math.toRadians(-90 )))
                .build();

        Trajectory toDropSpot = drive.trajectoryBuilder(FirstSpot)
                .lineToSplineHeading(new Pose2d(-16, 0, Math.toRadians(0)))
                .build();

        TrajectorySequence almostEverything = drive.trajectorySequenceBuilder(startPose)
                 .addTemporalMarker(() -> {
                     arm.GoToHeight(Arm.Height.UPPER_BUCKET);
                   logger.Log("raising arm with sample 1");
                 })
                .waitSeconds(armLiftTime)
                .addTemporalMarker(() -> logger.Log("to drop one"))
                .lineToSplineHeading(dropPose)
                .addTemporalMarker(() -> logger.Log("outtake one start"))
                .addTemporalMarker(4, intake.Outtake)
                .waitSeconds(outtakeTime)
                .addTemporalMarker(() -> logger.Log("to pickup one"))
                .addTrajectory(toFirstSpot)
                .addTemporalMarker(() -> logger.Log("arm down one start"))
                .waitSeconds(armLiftTime)
                .addTemporalMarker(() -> logger.Log("intake one start"))
                .addTemporalMarker(intake.Intake)
                .waitSeconds(intakeTime)
                .addTemporalMarker(() -> {
                    arm.GoToHeight(Arm.Height.UPPER_BUCKET);
                    logger.Log("raising arm with sample 2");
                    logger.close();
                })
                .waitSeconds(armLiftTime)
                .addTrajectory(toDropSpot)
                .addTemporalMarker(4, intake.Outtake)
                .waitSeconds(outtakeTime)
                .addTrajectory(toSecondSpot)
                .waitSeconds(armLiftTime)
                .addTemporalMarker(intake.Intake)
                .waitSeconds(intakeTime)
                .addTemporalMarker(() -> {
                  arm.GoToHeight(Arm.Height.UPPER_BUCKET);
                  logger.Log("raising arm with sample 3");
                  logger.close();
                })
                .waitSeconds(armLiftTime)
               // .lineToSplineHeading(dropPose)
                .addTemporalMarker(intake.Outtake)
                .waitSeconds(outtakeTime)
                .lineToSplineHeading(new Pose2d(0, 0))
                .build();
        telemetry.addData("almost everything",1);
        telemetry.update();
        sleep(5000);
        waitForStart();

        if (isStopRequested()) return;

        //update components
        while (opModeIsActive()) {
          //update components
          intake.Update();
          arm.Update();
        }

        drive.followTrajectorySequence(almostEverything);
    }
}