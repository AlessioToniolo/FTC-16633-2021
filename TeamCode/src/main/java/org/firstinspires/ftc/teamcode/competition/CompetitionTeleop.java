package org.firstinspires.ftc.teamcode.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utility.MaxStaticVelocity;
import org.firstinspires.ftc.teamcode.utility.PoseStorage;

@TeleOp
public class CompetitionTeleop extends LinearOpMode {

    // Runtime
    ElapsedTime runtime = new ElapsedTime();

    // PIDF
    public static PIDCoefficients PID = new PIDCoefficients(14,0,10);

    // Shooter Toggle Fields
    boolean prevValueShooter = false;
    boolean toggleShooter = false;

    double hardwareShooterAngle = 15;
    Pose2d poseEstimate;
    Pose2d updatePose;
    double encoderVelo;

    DcMotorEx shooter;

    @Override
    public void runOpMode() throws InterruptedException {


        // IMU Fields
        BNO055IMU imu;
        Orientation angles;
        Acceleration gravity;
        BNO055IMU.Parameters imuParameters;

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
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        frontRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        // Opmode
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {

            drive.update();

            poseEstimate = drive.getPoseEstimate();

            // TODO: figure out adjustment
            double poseRight = -poseEstimate.getHeading() + 180;

            // Our field centric driving
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(poseRight);

            drive.setWeightedDrivePower(
                    // TODO: add speed control
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

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

            // Trig angle adjustment
            if (gamepad1.a) {
                drive.turnAsync(trigAngle());
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

            // TODO: figure out reset
            updatePose = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0.0);

            if (gamepad1.x) {
                drive.setPoseEstimate(updatePose);
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

            while (gamepad1.a) {
                frontRoller.setPower(-1.0);
                bottomRoller.setPower(-1.0);
            }

            telemetry.addData("RPM: ", shooter.getVelocity());
            telemetry.update();
        }
    }

    public double trigAngle() {
        // A is along the Y axis (RoadRunner Coordinate System)
        // Y value can return same angle when positive with conditional at end
        double diffA = 72 - Math.abs(poseEstimate.getY());
        // B is along the X axis (RoadRunner Coordinate System)
        double diffB;

        // We can assume that if the field is remote for RIGHT
        // The length of the field will be 96", with the max Y value:
        // 24
        // And the minimum Y value will be
        // - 72

        // TODO: test and make changes for aligning to the right goal and
        // aliging with a bit of offset because of the robot

        // Accounting for negative X values
        if (poseEstimate.getX() < 0) {
            diffB = 72 + (72 - Math.abs(poseEstimate.getX()));
        } else {
            // For positive X values
            diffB = 72 - poseEstimate.getX();
        }

        // Find angle with atan2 of A over B
        double adjustment = Math.toDegrees(Math.atan2(diffA, diffB));

        // For making sure turning is the right way
        if (poseEstimate.getY() > 0) {
            adjustment = -(adjustment);
        } else {
            adjustment = Math.abs(adjustment);
        }

        // For RoadRunner criteria
        return Math.toRadians(adjustment);
    }

    // Sample Delay Code
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            //telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        }
    }

    // PIDF shooting function
    public void setBasicVelocity(double power) {
        encoderVelo = -((power * (100)) * 28);
        shooter.setVelocityPIDFCoefficients(PID.p, PID.i, PID.d, MaxStaticVelocity.maxStaticPIDFVelocity);
        shooter.setVelocity(encoderVelo);
    }
}
