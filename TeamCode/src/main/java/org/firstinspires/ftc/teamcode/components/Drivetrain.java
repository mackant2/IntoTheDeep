package org.firstinspires.ftc.teamcode.components;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.custom.Robot;

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
  BNO055IMU imu;
  Orientation angles;
  Acceleration gravity;
  Robot robot;
  float degree_Zero = 0;


  void initGyro(){
    imu = robot.drive.imu;
  }

  public void resetOrientation() {
      degree_Zero = angles.firstAngle;
  }

  public Drivetrain(Robot robot) {

    this.robot = robot;
    this.driverController = robot.opMode.gamepad1;
    rightBack = robot.drive.backRight;
    rightFront = robot.drive.frontRight;
    leftBack = robot.drive.backLeft;
    leftFront = robot.drive.frontLeft;

    initGyro();
  }

  public void Update() {
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double  theta = angles.firstAngle - degree_Zero;
    if (theta > 180) {
      theta -= 360;
    }
    else if (theta < -180){
      theta += 360;
    }

    robot.opMode.telemetry.addData("angle", theta);

    double x1 = -driverController.left_stick_x;
    double y1 = driverController.left_stick_y;
    double r = driverController.right_stick_x * rotationFactor;
    double  x = x1 * (Math.cos(Math.toRadians(theta))) + y1 * (Math.sin(Math.toRadians(theta)));
    double y = x1 * ( - Math.sin(Math.toRadians(theta))) + y1 * (Math.cos(Math.toRadians(theta)));
    rbVelocity = -x - y - r;
    rfVelocity = x - y - r;
    lbVelocity = x - y + r;
    lfVelocity = -x - y + r;
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
