package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SwerveModule {

    private DcMotor driveMotor;
    private DcMotor swerveMotor;

    private double swervePower = 0.5;

    private int zeroDegreePosition;

    final double MAX_POTENTIOMETER_VOLTAGE = 3.3;

    final int TICKS_PER_REV = 1124;
    final int TICKS_TO_DEGREE_FACTOR = 360 / TICKS_PER_REV;
    final int DEGREE_TO_TICKS_FACTOR = TICKS_PER_REV / 360;

    final double VOLTAGE_TO_TICKS_FACTOR = TICKS_PER_REV / MAX_POTENTIOMETER_VOLTAGE;

    public SwerveModule(DcMotor driveMotor, DcMotor swerveMotor, AnalogInput potentiometer)
    {
        this.driveMotor = driveMotor;
        this.driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.swerveMotor = swerveMotor;
        this.swerveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        zeroDegreePosition = (int)(-potentiometer.getVoltage() * VOLTAGE_TO_TICKS_FACTOR);
    }

    public void rotateByDegree(int angle)
    {
        swerveMotor.setPower(swervePower);
        int currentTicks = swerveMotor.getCurrentPosition();
        int targetTicks = currentTicks + angle * DEGREE_TO_TICKS_FACTOR;
        swerveMotor.setTargetPosition(targetTicks);
    }

    public void rotateToDegree(int angle)
    {
        int angleDiff = (angle - getCurrentAngle()) % 360;
        if (angleDiff > 180)
        {
            angleDiff -= 360;
        }
        else if (angleDiff < -180)
        {
            angleDiff += 360;
        }

        rotateByDegree(angleDiff);
    }

    public void setSwervePower(double power)
    {
        swervePower = power;
    }

    public int getCurrentAngle()
    {
        return (swerveMotor.getCurrentPosition() - zeroDegreePosition) * TICKS_TO_DEGREE_FACTOR;
    }

    public void setDriveMode(DcMotor.RunMode mode)
    {
        driveMotor.setMode(mode);
    }

    public void setDrivePower(double power)
    {
        driveMotor.setPower(power);
    }

    public int getDrivePosition()
    {
        return driveMotor.getCurrentPosition();
    }

    public void setDriveTarget(int target)
    {
        driveMotor.setTargetPosition(target);
    }
}
