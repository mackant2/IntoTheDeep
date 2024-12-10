package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Logger;

@Autonomous(name = "[OFFICIAL] Auto", group = "official")
public class ITDAuto extends LinearOpMode {
    Intake intake;
    Logger logger = new Logger();
    static Boolean wallAuto = false;
    static Boolean Side;
    DcMotorEx frontLeft, backLeft, backRight, frontRight;

    public void moveStraight (double power, int time) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(power);
        sleep(time);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        frontLeft = drive.frontLeft;
        backLeft = drive.backLeft;
        backRight = drive.backRight;
        frontRight = drive.frontRight;

        drive.leftFourBar.setPosition(.55);
        drive.rightFourBar.setPosition(.55);
        drive.liftLeft.setTargetPosition(0);
        drive.extendo.setTargetPosition(500);
        sleep(500);
        drive.extendo.setTargetPosition(Intake.ExtenderPosition.IN);
        drive.flipdown.setPosition(0);
        drive.wrist.setPosition(Arm.WristPosition.Specimen + 1);

        waitForStart();

        drive.claw.setPosition(Arm.ClawPosition.Closed);

        if(Side == wallAuto) {
            drive.liftLeft.setTargetPosition(Arm.Height.LOWER_BUCKET);

            drive.leftFourBar.setPosition(Arm.FourBarPosition.Specimen);
            drive.rightFourBar.setPosition(Arm.FourBarPosition.Specimen);

            sleep(2000);

            drive.wrist.setPosition(Arm.WristPosition.SampleDrop);

            moveStraight(.4, 900);

            sleep(1000);

            drive.claw.setPosition(Arm.ClawPosition.Open);

            sleep(10000);

        }
        else {
            //move to sub
            drive.liftLeft.setTargetPosition(Arm.Height.UPPER_BAR - 775);

            drive.leftFourBar.setPosition(Arm.FourBarPosition.Specimen);
            drive.rightFourBar.setPosition(Arm.FourBarPosition.Specimen);

            sleep(3000);

            moveStraight(.4, 2500);

            //put specimen on rung
            drive.wrist.setPosition(.5);
            sleep(500);

            //release specimen
            drive.claw.setPosition(Arm.ClawPosition.Open);
            sleep(500);

            drive.wrist.setPosition(.85);
            moveStraight(-.25, 450);

            drive.leftFourBar.setPosition(Arm.FourBarPosition.Specimen - .2);
            drive.rightFourBar.setPosition(Arm.FourBarPosition.Specimen - .2);
            drive.wrist.setPosition(.85);
            sleep(500);

            //move to box
            moveStraight(-.4, 300);
            drive.leftFourBar.setPosition(Arm.FourBarPosition.Transfer);
            drive.rightFourBar.setPosition(Arm.FourBarPosition.Transfer);

            drive.frontLeft.setPower(.43);
            drive.backLeft.setPower(-.4);
            drive.backRight.setPower(.4);
            drive.frontRight.setPower(-.43);

            drive.liftLeft.setTargetPosition(Arm.Height.DOWN);
            drive.wrist.setPosition(.44);
            sleep(1800);

            moveStraight(.5,1000);

            drive.frontLeft.setPower(.43);
            drive.backLeft.setPower(-.4);
            drive.backRight.setPower(.4);
            drive.frontRight.setPower(-.43);
            sleep(700);


            moveStraight(-.3, 4000);
        }
        /*logger.Initialize(telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        arm = new Arm(drive, this, () -> {});
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
                /*.build();

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
        }*/
    }
}