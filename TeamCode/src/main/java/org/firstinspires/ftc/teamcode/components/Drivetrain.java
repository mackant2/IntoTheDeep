package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class Drivetrain {
  Gamepad driverController;
  Set<Double> velocities = new HashSet<>();
  double rbVelocity;
  double rfVelocity;
  double lbVelocity;
  double lfVelocity;
  DcMotorEx rightBack, rightFront, leftBack, leftFront;
  double rotationFactor = 0.8;
  final int SPEED = 850;

  public Drivetrain(SampleMecanumDrive drive, Gamepad driverController) {
    this.driverController = driverController;
    rightBack = drive.backRight;
    rightFront = drive.frontRight;
    leftBack = drive.backLeft;
    leftFront = drive.frontLeft;
  }

  public void Update() {
    double x = -driverController.left_stick_x;
    double y = driverController.left_stick_y;
    double r = driverController.right_stick_x * rotationFactor;
    rbVelocity = -x - y - r;
    rfVelocity = x - y - r;
    lbVelocity = x - y + r;
    lfVelocity = -x - y + r ;
    velocities.add(rbVelocity); //right back
    velocities.add(rfVelocity); //right front
    velocities.add(lbVelocity); //left back
    velocities.add(lfVelocity); //left front
    double absMax = Math.abs(Collections.max(velocities));
    if (absMax > 1){
      rbVelocity /= absMax;
      rfVelocity /= absMax;
      lbVelocity /= absMax;
      lfVelocity /= absMax;
    }
    /*rightBack.setVelocity(rbVelocity * SPEED);
    rightFront.setVelocity(rfVelocity * SPEED);
    leftBack.setVelocity(lbVelocity * SPEED);
    leftFront.setVelocity(lfVelocity * SPEED);
    velocities.clear();*/
    rightBack.setPower(rbVelocity);
    rightFront.setPower(rfVelocity);
    leftBack.setPower(lbVelocity);
    leftFront.setPower(lfVelocity);
  }
}
