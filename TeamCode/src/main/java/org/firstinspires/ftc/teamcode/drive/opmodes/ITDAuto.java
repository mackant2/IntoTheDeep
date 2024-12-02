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

@Autonomous(name = "[OFFICIAL] Auto", group = "official")
public class ITDAuto extends LinearOpMode {
    Intake intake;
    Logger logger = new Logger();
    Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        logger.Initialize(telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        arm = new Arm(drive, telemetry, gamepad1, () -> {});
        intake = new Intake(drive, this, logger, () -> {});

        Pose2d startPose = new Pose2d(0,0);
        drive.setPoseEstimate(startPose);

        double armLiftTime = 4;
        double intakeTime = 1.5;
        double outtakeTime = 0.8;
        int threeSamplesY = 26;

        Pose2d dropPose = new Pose2d(16, 0, Math.toRadians((0)));
        Pose2d FirstSpot = new Pose2d(10, 26, Math.toRadians((-90)));


        Trajectory toFirstSpot = drive.trajectoryBuilder(dropPose)
                .lineToSplineHeading(new Pose2d(10, threeSamplesY, Math.toRadians(-90 )))
                .build();

        Trajectory toSecondSpot = drive.trajectoryBuilder(dropPose)
                .lineToSplineHeading(new Pose2d(20, threeSamplesY, Math.toRadians(-90 )))
                .build();

        Trajectory toDropSpot = drive.trajectoryBuilder(FirstSpot)
                .lineToSplineHeading(new Pose2d(16, 0, Math.toRadians(0)))
                .build();

        TrajectorySequence almostEverything = drive.trajectorySequenceBuilder(startPose)
                 .addTemporalMarker(() -> {
                     arm.GoToHeight(Arm.Height.UPPER_BUCKET);
                 })
                .lineToSplineHeading(dropPose)
                /*pgt[=f.addTrajectory(toFirstSpot)
                .addTrajectory(toDropSpot)
                .addTrajectory(toSecondSpot)
                .lineToSplineHeading(dropPose)
                .lineToSplineHeading(new Pose2d(0, 0))*/
                .build();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Beforeawdawd", 1);
        telemetry.update();
        drive.followTrajectorySequenceAsync(almostEverything);
        telemetry.addData("aiowjdaiowdjaiowjd", 1);
        telemetry.update();

        //update components
        while (opModeIsActive()) {
            //update components
            intake.Update();
            arm.Update();
        }
    }
}