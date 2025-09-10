package org.firstinspires.ftc.teamcode.Utility;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp
public class Vision extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException{

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .setCameraResolution(new Size(640, 480))
                .build();


        waitForStart();
        while (!isStopRequested() && opModeIsActive()){

            if (tagProcessor.getDetections().size() > 0 ){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("x: ", tag.ftcPose.x);
                telemetry.addData("y: ", tag.ftcPose.y);
                telemetry.addData("z: ", tag.ftcPose.x);
                telemetry.addData("roll: ", tag.ftcPose.roll);
                telemetry.addData("pitch: ", tag.ftcPose.pitch);
                telemetry.addData("yaw: ", tag.ftcPose.yaw);
                telemetry.addData("id: ", tag.id);


                telemetry.update();

            }



        }


    }
}
