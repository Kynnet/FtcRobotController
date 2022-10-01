package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor slidesMotor = hardwareMap.dcMotor.get("slidesMotor");


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            class pidTuner {
                double p = 0;
                double i = 0;
                double d = 0;
                private double previousError = 0;
                private double previousTime = 0;
                public double pid(double desiredPos, double currentPos, double slidesP, double slidesI, double slidesD, double maxI) {
                    double previousTime = 0;


                    double currentTime = getRuntime();
                    double currentError = desiredPos - currentPos;
                    double previousError = currentError;







                    p = slidesP * currentError;
                    i += slidesI * (currentError * (currentTime - previousTime));

                    if (i > maxI) {
                        i = maxI;
                    } else if (i < -maxI) {
                        i = -maxI;
                    }

                    d = slidesD * (currentError - previousError) / (currentTime - previousTime);



                    previousTime = currentTime;
                    return p + i + d;
                }
            }
            pidTuner power = new pidTuner();
            double encoderPos;
            encoderPos = slidesMotor.getCurrentPosition()/384.5 *2;
            slidesMotor.setPower(power.pid(50,encoderPos,0.01,0.005,0.003,0.01));



        }




    }
}

