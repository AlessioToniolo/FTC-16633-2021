package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utility.MaxStaticVelocity;

@TeleOp
public class SampleMaxVelocityTest extends LinearOpMode {

    // TODO: reconfigure for base robot class
    // TODO: match up motor names for temp absence of robot class
    DcMotorEx flywheelMotor;
    double currentVelocity = 0.0;
    double maxVelocity = 0.0;
    
    @Override
    public void runOpMode() throws InterruptedException {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelmotor");

        telemetry.addData("Important: ", "Make sure to have a full battery!");

        waitForStart();

        while (opModeIsActive()) {
            currentVelocity = flywheelMotor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();

            MaxStaticVelocity.maxStaticVelocity = maxVelocity;
        }

    }
}
