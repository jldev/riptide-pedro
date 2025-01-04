package org.firstinspires.ftc.teamcode.riptide;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RiptideConstants {



    public static double SLIDE_SPEED = 1.0;
    public static int SLIDE_MANUAL_SPEED = 25;
    public static double PIVOT_SPEED = 1.0;
    public static int PIVOT_MANUAL_SPEED = 15;
    public static double SLIDE_LOCK_POWER = 0.2;

    public static double SLIDES_PID_TOLERANCE = 10;
    public static double SLIDES_PID_POS_COEFFICIENT = .25;

    public static double HORIZONTAL_PID_P = 0.009;
    public static double HORIZONTAL_PID_I = 0.001;
    public static double HORIZONTAL_PID_D = 0.00;
    public static double HORIZONTAL_PID_F = 0.0;

    public static double VERTICAL_PID_P = 0.0075;
    public static double VERTICAL_PID_I = 0.0;
    public static double VERTICAL_PID_D = 0.00;
    public static double VERTICAL_PID_F = 0.0;

    public static double PIVOT_PID_P = 0.0175;
    public static double PIVOT_PID_I = 0.0;
    public static double PIVOT_PID_D = 0.001;
    public static double PIVOT_PID_F = 0.0;

    public  static double LIFT_POS_0 = 0.975f;
    public  static double LIFT_POS_1 = 0.9f;

    public static double LIFT_POS_2 = 0.5f;

    //    vertical positions

    public static int VERTICAL_SLIDE_HOME = 0;
    public static int VERTICAL_SLIDE_WALL = 275;
    public static int VERTICAL_SLIDE_HANG = 0;
    public static int VERTICAL_SLIDE_BASKET = 3853;
    public static int VERTICAL_PRELOAD_BASKET = 1900;
    public static int VERTICAL_ENDGAME = 500;

    // horizontal positions
    public static int HORIZONTAL_SLIDE_HOME = 0;
    public static int HORIZONTAL_SLIDE_OBS = 0;

    //    pivot positions

    public static int PIVOT_HOME = 0;
    public static int PIVOT_HANG = 1020;
    public static int PIVOT_BASKET = 1050;
    public static int PIVOT_SUB = 0;
    public static int PIVOT_WALL = 100;
    public static int PIVOT_PRELOAD_BASKET = 400;

    //   claw positions

    public static double GRIPPER_CLOSED_VALUE = 0.8f;
    public static double GRIPPER_OPEN_VALUE = 0.275f;
    public static double CLAW_YAW_INIT = 1f;
    public static double CLAW_PITCH_INIT = 0.5f;

    public static double YAW_HOME = 1;
    public static double PITCH_HOME = 0;
    public static double YAW_HANG = 0.0;
    public static double PITCH_HANG = 0.55;
    public static double YAW_SUB = 0.0;
    public static double PITCH_SUB = 0.55;
    
}
