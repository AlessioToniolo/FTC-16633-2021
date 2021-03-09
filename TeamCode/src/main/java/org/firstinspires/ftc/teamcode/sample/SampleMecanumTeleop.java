package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class SampleMecanumTeleop extends LinearOpMode {

    // Runtime
    ElapsedTime runtime = new ElapsedTime();

    // Shooter Toggle Fields
    boolean prevValueShooter = false;
    boolean toggleShooter = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // IMU Fields
        BNO055IMU imu;
        Orientation angles;
        Acceleration gravity;
        BNO055IMU.Parameters imuParameters;

        // Drive motors
        DcMotor leftFront = hardwareMap.dcMotor.get("leftfront");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightfront");
        DcMotor leftRear = hardwareMap.dcMotor.get("leftrear");
        DcMotor rightRear = hardwareMap.dcMotor.get("rightrear");

        // Servos
        Servo wobbleServo = hardwareMap.servo.get("wobbleservo");
        Servo shooterServo = hardwareMap.servo.get("shooterservo");

        // Intake motors
        DcMotor frontRoller = hardwareMap.dcMotor.get("frontroller");
        DcMotor bottomRoller = hardwareMap.dcMotor.get("bottomroller");

        // Wobble motor
        DcMotor wobbleArm = hardwareMap.dcMotor.get("wobblearm");
        double armMotorPower = 0;

        // Shooter motor
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU Initialization
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        // Opmode
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {

            // IMU Data
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double imuAngle = angles.firstAngle;

            // Compute field centric vector
            double theta;
            if (Math.abs(gamepad1.left_stick_x) < 0.05)
            {
                theta = Math.atan(gamepad1.left_stick_y/0.05);
            }
            else {
                if (gamepad1.left_stick_x < 0) {
                    theta = Math.atan((-1*gamepad1.left_stick_y)/(gamepad1.left_stick_x));
                    //theta = Math.atan((gamepad1.left_stick_y)/(gamepad1.left_stick_x));
                }
                else {
                    theta = Math.atan(gamepad1.left_stick_y/gamepad1.left_stick_x);
                }
            }
            double mag = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
            theta = theta - Math.PI/2;
            if (gamepad1.left_stick_x > 0) {
                theta = theta * -1;
            }
            double newTheta = theta - ((imuAngle/360.0) * 2 * Math.PI);

            // Compute power for wheels
            double leftFrontPower = mag * Math.cos(newTheta + (Math.PI/4)) - gamepad1.right_stick_x;
            double rightFrontPower = mag * Math.sin(newTheta + (Math.PI/4)) + gamepad1.right_stick_x;
            double leftRearPower = mag * Math.sin(newTheta + (Math.PI/4)) - gamepad1.right_stick_x;
            double rightRearPower = mag * Math.cos(newTheta + (Math.PI/4)) + gamepad1.right_stick_x;

            // Set motor powers
            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            // Intake
            if (gamepad1.dpad_down) {
                frontRoller.setPower(1.0);
                bottomRoller.setPower(1.0);
            }
            if (gamepad1.dpad_right) {
                frontRoller.setPower(0);
                bottomRoller.setPower(0);
            }

            // Wobble Servo
            // wobble servo opens
            if (gamepad1.left_bumper) {
                wobbleServo.setPosition(0.5);
            }
            // wobble servo closes
            if (gamepad1.right_bumper) {
                wobbleServo.setPosition(0.85);
            }

            // Wobble Arm
            armMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (armMotorPower > 0.4) {
                armMotorPower = 0.4;
            } else if (armMotorPower < -0.4) {
                armMotorPower = -0.4;
            }
            wobbleArm.setPower(armMotorPower);

            // Shooter Servo
            if (gamepad1.dpad_left) {
                shooterServo.setPosition(0.2);
                delay(0.6);
                shooterServo.setPosition(0.05);
            }

            // Shooter Toggle
            if (gamepad1.dpad_up && gamepad1.dpad_up != prevValueShooter) {
                if (!toggleShooter) {
                    shooter.setPower(-1.0);
                } else {
                    shooter.setPower(0);
                }
                toggleShooter = !toggleShooter;
            }
            prevValueShooter = gamepad1.dpad_up;

            telemetry.addData("RPM: ", shooter.getVelocity());
            telemetry.update();
        }
    }

    // Sample Delay Code
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            //telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        }
    }
}
