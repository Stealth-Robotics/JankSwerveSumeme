package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class TriSwerveDrive implements Runnable {

    private SwerveModule[] modules;
    BNO055IMU imu;

    double speed;
    double xSpeed;
    double ySpeed;
    double rotSpeed;

    DcMotor.RunMode mode;

    public TriSwerveDrive(
            DcMotor drive1, DcMotor swerve1, AnalogInput potentiometer1,
            DcMotor drive2, DcMotor swerve2, AnalogInput potentiometer2,
            DcMotor drive3, DcMotor swerve3, AnalogInput potentiometer3,
            BNO055IMU imu
            )
    {
        modules = new SwerveModule[3];
        modules[0] = new SwerveModule(drive1, swerve1, potentiometer1, 0, 18, 0);
        modules[1] = new SwerveModule(drive2, swerve2, potentiometer2, 120, 18, 0);
        modules[2] = new SwerveModule(drive3, swerve3, potentiometer3, -120, 18, 0);

        this.imu = imu;

        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    }

    @Override
    public void run()
    {
        //if the goal is to call this every loop anyway, rather than implementing Runnable,
        //i would just make a drive function. we need 3 inputs: left x, left y, and right x
        //ideally the deadzone handling would be done beforehand. i have made this function for my comments
        //feel free to remove it if you don't want to do it that way

        if (rotSpeed == 0)
        {
            //double check this crab math. I've fixed it up some in TestSwerveModule
            double angle;
            if (xSpeed != 0)
            {
                angle = Math.atan(ySpeed / xSpeed);
            }
            else
            {
                angle = (ySpeed >= 0) ? 90 : -90;
            }

            for (SwerveModule module : modules)
            {
                module.rotateToDegree(angle);
                module.setDrivePower(speed);
            }
        }
        else if (speed == 0)
        {
            for (SwerveModule module : modules)
            {
                module.rotateToDegree(module.angleFromCenter - 90);
                module.setDrivePower(rotSpeed);
            }
        }
        else
        {

        }
    }

    public void update(double x, double y, double turnX){
        //assume joystick deadzone is already dealt with
        //let's define some numbers.

        //the swerve pivot setpoints
        double[] alphas = new double[3];
        //this is the base drive power for crab mode, given by joystick. calculate here
        //reference TestSwerveModule as needed
        double pow = 0;
        //this is the angle of primary joystick. calculate here (make sure to convert to degrees)
        //reference TestSwerveModule as needed
        double gamma = 0;
        //this is the orientation of the robot. make sure it is constrained to the range 0-360
        double phi = getHeading();
        //these are for ocelot twists.
        //this describes the offset of each swerve pivot from the vector of motion
        double[] gammas = new double[] {-phi - gamma, 120 - phi - gamma, 240 - phi - gamma};
        //this is the turn angle of the fictional reference CL wheel.
        //this wheel lies along the vector of motion the same distance from the center as the other wheels
        //we constrain its turn from -45 to 45 mostly because it makes the math nice
        //it keeps the turn centerpoint outside of the robot
        double delta_cl = turnX * 45;
        //these are the angular correction needed to perform the ocelot twist
        double[] deltas = new double[3];
        //this is the distance to centerpoint around the snake turn
        //we don't actually perform that motion but we pretend we are
        double r_cp = 0;
        //these are the distances from the individual wheels to the centerpoint
        double[] rs = new double[3];
        //these are the individual drive module powers
        double[] pows = new double[3];
        if(x == 0 && y == 0){
            //rotate in place
            for(int i = 0; i < 3; i++){
                if(turnX > 0){
                    alphas[i] = 90;
                }
                else if(turnX < 0){
                    alphas[i] = 270;
                }
            }
            //specifically do nothing if both joysticks idle
        }
        else{
            //all other math lives here!
            //it's getting a little late for me so i'll fill in some more stuff tomorrow morning
        }
    }

    public void setRot(double rotSpeed)
    {
        if (rotSpeed + this.speed > 1)
        {
            double speedFactor = 1 / (rotSpeed + this.speed);
            setSpeed(this.speed * speedFactor);
            rotSpeed *= speedFactor;
        }

        this.rotSpeed = rotSpeed;
    }

    public void setSpeed(double speed)
    {
        if (speed + this.rotSpeed > 1)
        {
            double speedFactor = 1 / (speed + this.rotSpeed);
            setRot(this.rotSpeed * speedFactor);
            speed *= speedFactor;
        }

        this.speed = speed;
    }

    public void setDirection(double angle)
    {
        xSpeed = Math.cos(angle) * speed;
        ySpeed = Math.sin(angle) * speed;
    }

    public void setVelocity(double angle, double speed)
    {
        setSpeed(speed);
        setDirection(angle);
    }

    public void setMode(DcMotor.RunMode mode)
    {
        for (SwerveModule module : modules)
        {
            module.setDriveMode(mode);
        }

        this.mode = mode;
    }

    public double[] findNextVector(SwerveModule module)
    {
        double angle = getHeading() + module.angleFromCenter;
        double currX = Math.cos(angle) * module.distFromCenter;
        double currY = Math.sin(angle) * module.distFromCenter;

        double nextX = module.distFromCenter * Math.cos(rotSpeed / module.distFromCenter + angle) + xSpeed;
        double nextY = module.distFromCenter * Math.sin(rotSpeed / module.distFromCenter + angle) + ySpeed;
        double[] vector = new double[2];

        double diffX = nextX - currX;
        double diffY = nextY - currY;
        vector[0] = Math.sqrt(diffX * diffX + diffY * diffY);
        vector[1] = (diffX != 0) ? Math.atan(diffY / diffX) : ((diffY > 0) ? 90 : -90);
        return vector;
    }

    private double getHeading()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
