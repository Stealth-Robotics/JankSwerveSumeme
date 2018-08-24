package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SwerveModule {

    private DcMotor driveMotor;
    private DcMotor swerveMotor;

    private double zeroDegreePosition;

    final int MAX_POTENTIOMETER_VOLTAGE = 3;
    final int VOLTAGE_TO_ANGLE_FACTOR = 360 / MAX_POTENTIOMETER_VOLTAGE;
    final int ANGLE_TO_VOLTAGE_FACTOR = MAX_POTENTIOMETER_VOLTAGE / 360;

    final int TICKS_PER_REV = 1124;
    final int TICKS_TO_DEGREE_FACTOR = TICKS_PER_REV / 360;

    final int VOLTAGE_TO_TICKS_FACTOR = TICKS_PER_REV / MAX_POTENTIOMETER_VOLTAGE;

    public SwerveModule(DcMotor driveMotor, DcMotor serveMotor, AnalogInput potentiometer)
    {
        this.driveMotor = driveMotor;
        this.driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.swerveMotor = serveMotor;
        this.swerveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        zeroDegreePosition = -potentiometer.getVoltage() * VOLTAGE_TO_TICKS_FACTOR;

    }

    public void rotateByDegree(int degree)
    {
        swerveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int currentTicks = swerveMotor.getCurrentPosition();
        int targetTicks = currentTicks + degree * TICKS_TO_DEGREE_FACTOR;
        swerveMotor.setTargetPosition(targetTicks);
    }

    public void rotateToDegree(double degree)
    {

    }
}
