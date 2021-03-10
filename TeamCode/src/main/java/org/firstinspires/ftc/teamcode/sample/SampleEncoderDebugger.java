package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SampleEncoderDebugger extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "rightrear");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Encoder counts: ", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
