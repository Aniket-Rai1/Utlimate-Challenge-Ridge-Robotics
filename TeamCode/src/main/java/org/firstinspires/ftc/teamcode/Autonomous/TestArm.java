package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="Test Arm", group = "basic")
public class TestArm extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.teleop);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){

            //rob.encoderMovement(0.5, 1, 10, 1000, Goal.movements.clawIn, rob.claw);
            rob.grabber.setPosition(.5);
        }


    }
}
