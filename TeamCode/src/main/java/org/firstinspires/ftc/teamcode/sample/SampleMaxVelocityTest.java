package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utility.MaxStaticVelocity;

@TeleOp
@Disabled
public class SampleMaxVelocityTest extends LinearOpMode {

    // TODO: reconfigure for base robot class
    // TODO: match up motor names for temp absence of robot class
    DcMotorEx flywheelMotor;
    double currentVelocity = 0.0;
    double maxVelocity = 0.0;
    
    @Override
    public void runOpMode() throws InterruptedException {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Important: ", "Make sure to have a full battery!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            flywheelMotor.setPower(1.0);
            currentVelocity = flywheelMotor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.addData("static variable velocity", MaxStaticVelocity.maxStaticVelocity);
            telemetry.update();

            MaxStaticVelocity.maxStaticVelocity = maxVelocity;
        }

    }
}
