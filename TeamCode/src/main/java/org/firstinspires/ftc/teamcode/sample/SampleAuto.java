  package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.MaxStaticVelocity;

@Disabled
public class SampleAuto extends LinearOpMode {

    // Runtime
    ElapsedTime runtime = new ElapsedTime();

    // PIDF
    DcMotorEx shooter;

    // Wobble
    DcMotor wobbleArm;

    @Override
    public void runOpMode() throws InterruptedException {
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
        wobbleArm = hardwareMap.dcMotor.get("wobblearm");
        double armMotorPower = 0;

        // Shooter motor
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(10, 13.9, 0, MaxStaticVelocity.maxStaticPIDFVelocity);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void wobbleArmDegSet(double speed, double deg, double timeoutS) {
        int target;
        deg = deg * 2;
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (Math.abs(speed) > 1.0) {
            speed = 1.0;
        }

        //TODO: finish!
    }
}
