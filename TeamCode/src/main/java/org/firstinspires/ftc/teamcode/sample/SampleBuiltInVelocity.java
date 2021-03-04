package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.utility.MaxStaticVelocity;
import org.firstinspires.ftc.teamcode.utility.TuningController;

@TeleOp
public class SampleBuiltInVelocity extends LinearOpMode {
    DcMotorEx flywheelMotor;
    double speed = 0.5;
    double currentSpeed = 0.0;
    double encoderVelo = 0.0;

    // TODO: plug in your PID coefficients
    public static PIDCoefficients PID = new PIDCoefficients(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelmotor");

        flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flywheelMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        waitForStart();

        if (opModeIsActive() & !isStopRequested()) {
            // Sample value: 0.5
            setBasicVelocity(speed);
        }

        while (opModeIsActive() & !isStopRequested()) {
            currentSpeed = flywheelMotor.getVelocity();

            // TODO: make sure this power is equal to desired power to verify your PIDF values
            telemetry.addData("Current Velocity Comparison Test", " |");
            telemetry.addData("Current Velocity:", currentSpeed);
            telemetry.addData("Desired Velocity:", encoderVelo);
        }
    }

    public void setBasicVelocity(double power) {
        encoderVelo = (power * (TuningController.MOTOR_MAX_RPM / 60)) * TuningController.MOTOR_TICKS_PER_REV;
        flywheelMotor.setVelocityPIDFCoefficients(PID.p, PID.i, PID.d, MaxStaticVelocity.maxStaticVelocity);
        flywheelMotor.setVelocity(encoderVelo);
    }

    public void terminateVelocity() {
        flywheelMotor.setVelocity(0);
    }

}
