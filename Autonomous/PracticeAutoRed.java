package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime
;


import org.firstinspires.ftc.teamcode.Util.PracticeHardware;


@Autonomous(name="Blue Side Ver 2.4", group="Pushbot")
public class PracticeAutoRed extends LinearOpMode {
    ColorSensor colorSensor;


    PracticeHardware robot = new PracticeHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.25;
    static final double Servo = 0.8;
    static final double Servobback = -.8;
    double arm = -.2;

    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        colorSensor = hardwareMap.colorSensor.get("colorsensor");
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 8)) {
            // robot.leftDrive.setPower(-.15);
            // robot.rightDrive.setPower(-.3);
            // robot.rightbackDrive.setPower(.15);
            // robot.leftbackDrive.setPower(.3);
            robot.slideup2.setPower(.1);

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            robot.colorarm.setPosition(Servo);
            //   robot.leftDrive.setPower(FORWARD_SPEED);
            //  robot.rightDrive.setPower(-FORWARD_SPEED);
            // robot.rightbackDrive.setPower(-FORWARD_SPEED);
            // robot.leftbackDrive.setPower(FORWARD_SPEED);

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            //  robot.colorarm.setPosition(0);
            robot.colorarmx.setPower(.1);

            //  robot.left1Claw.setPosition(-robot.MID_SERVO1);
            //  robot.right1Claw.setPosition(robot.MID_SERVO1);
            //  robot.left2Claw.setPosition(robot.MID_SERVO1);
            //  robot.right2Claw.setPosition(-robot.MID_SERVO1);


            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 5)) {
            if (colorSensor.red() >= 5) {
                robot.leftDrive.setPower(-.3);
                robot.rightDrive.setPower(-.3);
                robot.rightbackDrive.setPower(.3);
                robot.leftbackDrive.setPower(.3);
            }


                telemetry.addData("Clear", colorSensor.alpha());
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());

                telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5)) {
                if (colorSensor.red() < 1) {
                    robot.leftDrive.setPower(.3);
                    robot.rightDrive.setPower(.3);
                    robot.rightbackDrive.setPower(-.3);
                    robot.leftbackDrive.setPower(-.3);
                }


                telemetry.addData("Clear", colorSensor.alpha());
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());

                telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .3)) {
                robot.colorarmx.setPower(-1*.2);

                //  robot.left1Claw.setPosition(-robot.MID_SERVO1);
                //  robot.right1Claw.setPosition(robot.MID_SERVO1);
                //  robot.left2Claw.setPosition(robot.MID_SERVO1);
                //  robot.right2Claw.setPosition(-robot.MID_SERVO1);


                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2)) {
                robot.colorarm.setPosition(-1*.5);
                //   robot.leftDrive.setPower(FORWARD_SPEED);
                //  robot.rightDrive.setPower(-FORWARD_SPEED);
                // robot.rightbackDrive.setPower(-FORWARD_SPEED);
                // robot.leftbackDrive.setPower(FORWARD_SPEED);

                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            runtime.reset();


            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }






