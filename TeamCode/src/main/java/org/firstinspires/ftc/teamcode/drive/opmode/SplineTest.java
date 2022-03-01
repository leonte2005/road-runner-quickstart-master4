package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.T265Localizer;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    private static T265Camera slamera = null;
    private Servo griper;
    private DcMotor ridbrat;
    int dist_sus;


    /**
     * inchidem griperul
     */
    private void inchide_griper() {
        griper.setPosition(1);
    }

    /**
     * ridicam bratul
     */
    private void ridica_brat() {
        ridbrat.setDirection(DcMotorSimple.Direction.FORWARD);
        ridbrat.setTargetPosition(230);
        ridbrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) ridbrat).setVelocity(800);
        while (dist_sus <= 230) {
            dist_sus = ridbrat.getCurrentPosition();
            telemetry.addData("Pozitie sus", dist_sus);
            telemetry.update();
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


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        griper = hardwareMap.get(Servo.class, "griper");
        griper.setPosition(0.5);
        dist_sus = 0;
        ridbrat = hardwareMap.get(DcMotor.class, "ridbrat");


        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .splineTo(new Vector2d(20, -20), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(431), 7.28),
                        SampleMecanumDrive.getAccelerationConstraint(52.48180821614297))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(40)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(40)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), true)
                .splineTo(new Vector2d(2,0), Math.toRadians(180))
                .build();



        inchide_griper();
        sleep(500);
        ridica_brat();
        sleep(200);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        deschide_griper();
        sleep(400);
        coboara_brat();
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);



        ((T265Localizer) drive.getLocalizer()).cleanup();
    }

}
