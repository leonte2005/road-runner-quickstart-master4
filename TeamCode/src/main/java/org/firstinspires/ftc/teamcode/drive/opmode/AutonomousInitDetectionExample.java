/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.T265Localizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;






/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Autonomous
public class AutonomousInitDetectionExample extends LinearOpMode
{
    private Servo griper;
    private DcMotor carousel;
    private DcMotor ridbrat;
    int dist_sus;

    /**
     * inchidem griperul
     */
    private void inchide_griper() {
        griper.setPosition(1);
    }


    /**
     * initializare carousel
     */
    private void initializare_carousel() {
        carousel.setDirection(DcMotorSimple.Direction.REVERSE);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /**
     * actionare carousel
     */
    private void carousel2() {
        carousel.setTargetPosition(2800);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) carousel).setVelocity(1600);
    }




    /**
     * ridicam bratul1
     */
    private void ridica_brat() {
        ridbrat.setDirection(DcMotorSimple.Direction.FORWARD);
        ridbrat.setTargetPosition(80);
        ridbrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) ridbrat).setVelocity(800);
        while (dist_sus <= 50) {
            dist_sus = ridbrat.getCurrentPosition();
            telemetry.addData("Pozitie sus", dist_sus);
            telemetry.update();
        }
    }


    /**
     * ridicam bratul2
     */
    private void ridica_brat2() {
        ridbrat.setDirection(DcMotorSimple.Direction.FORWARD);
        ridbrat.setTargetPosition(150);
        ridbrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) ridbrat).setVelocity(800);
        while (dist_sus <= 150) {
            dist_sus = ridbrat.getCurrentPosition();
            telemetry.addData("Pozitie sus", dist_sus);
            telemetry.update();
        }
    }




    /**
     * ridicam bratul3
     */
    private void ridica_brat3() {
        ridbrat.setDirection(DcMotorSimple.Direction.FORWARD);
        ridbrat.setTargetPosition(210);
        ridbrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) ridbrat).setVelocity(800);
        while (dist_sus <= 210) {
            dist_sus = ridbrat.getCurrentPosition();
        }
    }







    /**
     * coboram bratul
     */
    private void coboara_brat() {
        ridbrat.setDirection(DcMotorSimple.Direction.REVERSE);
        ridbrat.setTargetPosition(dist_sus / dist_sus);
        ridbrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) ridbrat).setVelocity(200);
        ridbrat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (ridbrat.isBusy()) {
            dist_sus = ridbrat.getCurrentPosition();
        }
        ridbrat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    /**
     * Deschide griper
     */
    private void deschide_griper() {
        griper.setPosition(0.5);
    }


    OpenCvWebcam webcam;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        griper = hardwareMap.get(Servo.class, "griper");
        griper.setPosition(0.5);
        dist_sus = 0;
        ridbrat = hardwareMap.get(DcMotor.class, "ridbrat");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        initializare_carousel();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        //telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        //telemetry.update();




        /**
         * Traiectorii
         */
        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .splineTo(new Vector2d(16, -27), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, Math.toRadians(184), 7.28),
                        SampleMecanumDrive.getAccelerationConstraint(52.48180821614297))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .splineTo(new Vector2d(1, 20), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(25, Math.toRadians(184), 7.28),
                        SampleMecanumDrive.getAccelerationConstraint(52.48180821614297))

                //.splineTo(new Vector2d(-58,-50), Math.toRadians(180))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-3, -86), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(184), 7.28),
                        SampleMecanumDrive.getAccelerationConstraint(52.48180821614297))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(-3, -92), Math.toRadians(-90), SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(184), 7.28),
                        SampleMecanumDrive.getAccelerationConstraint(52.48180821614297))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(),true)
                .splineTo(new Vector2d(-3, -86), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(25, Math.toRadians(184), 7.28),
                        SampleMecanumDrive.getAccelerationConstraint(52.48180821614297))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), true)
                .splineTo(new Vector2d(-3, -50), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(25, Math.toRadians(184), 7.28),
                        SampleMecanumDrive.getAccelerationConstraint(52.48180821614297))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToLinearHeading(new Pose2d(12, -28, Math.toRadians(0)))

                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToLinearHeading(new Pose2d(-7, -45, Math.toRadians(-90)))

                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .forward(40)

                .build();


        switch (snapshotAnalysis)
        {
            case LEFT:
            {
                /* Your autonomous code */

                inchide_griper();
                sleep(500);
                ridica_brat();
                sleep(200);
                drive.followTrajectory(traj1);
                sleep(50);
                deschide_griper();
                sleep(50);
                coboara_brat();
                drive.followTrajectory(traj2);
                sleep(50);
                carousel2();
                sleep(2000);
                drive.followTrajectory(traj3);
                drive.followTrajectory(traj4);
                inchide_griper();
                drive.followTrajectory(traj5);
                ridica_brat3();
                drive.followTrajectory(traj6);
                drive.followTrajectory(traj7);
                deschide_griper();
                sleep(50);
                drive.followTrajectory(traj8);
                drive.followTrajectory(traj9);

                //deschide_griper();
                //sleep(400);
                //coboara_brat();
                //drive.followTrajectory(traj3);
                //drive.followTrajectory(traj4);




                break;
            }

            case RIGHT:
            {
                /* Your autonomous code */

                inchide_griper();
                sleep(500);
                ridica_brat3();
                sleep(200);
                drive.followTrajectory(traj1);
                sleep(50);
                deschide_griper();
                sleep(50);
                coboara_brat();
                drive.followTrajectory(traj2);
                sleep(50);
                carousel2();
                sleep(2000);
                drive.followTrajectory(traj3);
                drive.followTrajectory(traj4);
                inchide_griper();
                drive.followTrajectory(traj5);
                ridica_brat3();
                drive.followTrajectory(traj6);
                drive.followTrajectory(traj7);
                deschide_griper();
                sleep(50);
                drive.followTrajectory(traj8);
                drive.followTrajectory(traj9);
                break;
            }

            case CENTER:
            {
                /* Your autonomous code*/
                /* Your autonomous code */

                inchide_griper();
                sleep(500);
                ridica_brat2();
                sleep(200);
                drive.followTrajectory(traj1);
                sleep(50);
                deschide_griper();
                sleep(50);
                coboara_brat();
                drive.followTrajectory(traj2);
                sleep(50);
                carousel2();
                sleep(2000);
                drive.followTrajectory(traj3);
                drive.followTrajectory(traj4);
                inchide_griper();
                drive.followTrajectory(traj5);
                ridica_brat3();
                drive.followTrajectory(traj6);
                drive.followTrajectory(traj7);
                deschide_griper();
                sleep(50);
                drive.followTrajectory(traj8);
                drive.followTrajectory(traj9);
                break;
            }
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive())
        {
            drive.update();
            drive.updatePoseEstimate();
            // Don't burn CPU cycles busy-looping in this sample
         //   sleep(50);
        }

        ((T265Localizer) drive.getLocalizer()).cleanup();
    }
}