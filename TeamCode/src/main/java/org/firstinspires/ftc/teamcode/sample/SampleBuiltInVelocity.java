package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.MaxStaticVelocity;
import org.firstinspires.ftc.teamcode.utility.external.TuningController;

@TeleOp
@Disabled
public class SampleBuiltInVelocity extends LinearOpMode {
    DcMotorEx flywheelMotor;
    Servo kicker;

    ElapsedTime runtime = new ElapsedTime();

    double speed = 0.95;
    double currentSpeed = 0.0;
    double encoderVelo = 0.0;

    // TODO: plug in your PID coefficients
    public static PIDCoefficients PID = new PIDCoefficients(14,0,10);

    @Override
    public void runOpMode() throws InterruptedException {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.servo.get("shooterservo");

        flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flywheelMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive() & !isStopRequested()) {
            currentSpeed = flywheelMotor.getVelocity();

            if (gamepad1.dpad_up) {
                // Sample value: 0.95
                setBasicVelocity(speed);
            }

            if (gamepad1.dpad_left) {
                kicker.setPosition(0.2);
                delay(0.6);
                kicker.setPosition(0.05);
            }

            // TODO: make sure this power is equal to desired power to verify your PIDF values
            telemetry.addData("Current Velocity Comparison Test", " |");
            telemetry.addData("Current Velocity:", currentSpeed);
            telemetry.addData("Desired Velocity:", encoderVelo);
            telemetry.update();
        }
    }

    public void setBasicVelocity(double power) {
        encoderVelo = -((power * (TuningController.MOTOR_MAX_RPM / 60)) * TuningController.MOTOR_TICKS_PER_REV);
        flywheelMotor.setVelocityPIDFCoefficients(PID.p, PID.i, PID.d, MaxStaticVelocity.maxStaticPIDFVelocity);
        flywheelMotor.setVelocity(encoderVelo);
    }

    public void terminateVelocity() {
        flywheelMotor.setVelocity(0);
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
