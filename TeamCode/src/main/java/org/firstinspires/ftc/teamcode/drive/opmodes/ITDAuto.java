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


import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "[OFFICIAL] Auto", group = "official")
public class ITDAuto extends LinearOpMode {
    Intake intake;
    Logger logger = new Logger();
    static Boolean wallAuto = false;
    static Boolean specimenAuto = true;
    static Boolean Side;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.claw.setPosition(Arm.ClawPosition.Closed);
        drive.leftFourBar.setPosition(.55);
        drive.liftLeft.setTargetPosition(0);
        drive.extendo.setTargetPosition(30);
        drive.flipdown.setPosition(0);

        waitForStart();


        if(Side == wallAuto) {

            drive.liftLeft.setTargetPosition(Arm.Height.LOWER_BUCKET);

            sleep(2000);

            drive.leftFourBar.setPosition(Arm.FourBarPosition.Specimen);
            drive.wrist.setPosition(Arm.WristPosition.SampleDrop);

            drive.frontLeft.setPower(.4);
            drive.backLeft.setPower(.4);
            drive.backRight.setPower(.4);
            drive.frontRight.setPower(.4);

            sleep(900);

            drive.frontLeft.setPower(0);
            drive.backLeft.setPower(0);
            drive.backRight.setPower(0);
            drive.frontRight.setPower(0);

            sleep(1000);

            drive.claw.setPosition(Arm.ClawPosition.Open);

            sleep(10000);

}

if(Side == specimenAuto){

drive.liftLeft.setTargetPosition(Arm.Height.UPPER_BAR - 782);
drive.wrist.setPosition(Arm.WristPosition.Specimen + .3);
drive.leftFourBar.setPosition(Arm.FourBarPosition.Specimen);
drive.rightFourBar.setPosition(Arm.FourBarPosition.Specimen);

sleep(1000);

    drive.frontLeft.setPower(.4);
    drive.backLeft.setPower(.4);
    drive.backRight.setPower(.4);
    drive.frontRight.setPower(.4);
    sleep(2800);

    drive.wrist.setPosition(.4);
    sleep(500);

    drive.claw.setPosition(Arm.ClawPosition.Open);
    sleep(500);

    drive.frontLeft.setPower(-.25);
    drive.backLeft.setPower(-.25);
    drive.backRight.setPower(-.25);
    drive.frontRight.setPower(-.25);
    sleep(400);

    drive.leftFourBar.setPosition(Arm.FourBarPosition.Specimen - .2);
    drive.rightFourBar.setPosition(Arm.FourBarPosition.Specimen - .2);
    drive.wrist.setPosition(.85);
    drive.frontLeft.setPower(0);
    drive.backLeft.setPower(0);
    drive.backRight.setPower(0);
    drive.frontRight.setPower(0);
    sleep(500);

    drive.frontLeft.setPower(-.4);
    drive.backLeft.setPower(-.4);
    drive.backRight.setPower(-.4);
    drive.frontRight.setPower(-.4);

    sleep(800);

    drive.frontLeft.setPower(.43);
    drive.backLeft.setPower(-.4);
    drive.backRight.setPower(.4);
    drive.frontRight.setPower(-.43);
    drive.liftLeft.setTargetPosition(Arm.Height.DOWN);
    drive.wrist.setPosition(.44);
    sleep(1000);

    drive.leftFourBar.setPosition(Arm.FourBarPosition.Transfer);
    drive.rightFourBar.setPosition(Arm.FourBarPosition.Transfer);

    sleep(1500);

    drive.frontLeft.setPower(-.3);
    drive.backLeft.setPower(-.3);
    drive.backRight.setPower(-.3);
    drive.frontRight.setPower(-.3);

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