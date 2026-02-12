package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HuskyLens Basic Test")
public class HuskyLensTest extends LinearOpMode {
    private HuskyLens husky;

    @Override
    public void runOpMode() {
        husky = hardwareMap.get(HuskyLens.class, "huskyLens");
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addLine("HuskyLens initialized - point at AprilTag");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = husky.blocks();
            telemetry.addData("Detected Blocks", blocks.length);

            if (blocks.length > 0) {
                HuskyLens.Block first = blocks[0];
                telemetry.addData("ID", first.id);
                telemetry.addData("x/y center", "%.0f, %.0f", first.x, first.y);
                telemetry.addData("Width/Height", "%d x %d", first.width, first.height);
            } else {
                telemetry.addLine("No tags detected");
            }

            telemetry.update();
            sleep(100);
        }
    }
}