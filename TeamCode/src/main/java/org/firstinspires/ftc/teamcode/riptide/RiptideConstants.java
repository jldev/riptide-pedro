package org.firstinspires.ftc.teamcode.riptide;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RiptideConstants {



    public static double SLIDE_SPEED = 1.0;
    public static int VERT_SLIDE_MANUAL_SPEED = 25;
    public static int HORZ_SLIDE_MANUAL_SPEED = 10;
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
    public static int VERTICAL_SLIDE_WALL = 150;
    public static int VERTICAL_SLIDE_HANG = 400;
    public static int VERTICAL_SLIDE_BASKET = 400; //2800;
    public static int VERTICAL_PRELOAD_BASKET = 400; //1900;
    public static int VERTICAL_ENDGAME = 500;

    // horizontal positions
    public static int HORIZONTAL_SLIDE_HOME = 0;
    public static int HORIZONTAL_SLIDE_OBS = 0;
    public static int HORIZONTAL_SLIDE_SUB = 0;
    public static int HORIZONTAL_SLIDE_WALL = 0;


    //   horizontal claw positions

    public static double GRIPPER_CLOSED_VALUE_VERTICAL = 0.2f;
    public static double GRIPPER_CLOSED_VALUE_HORIZONTAL = 0.05;
    public static double GRIPPER_OPEN_VALUE = .50f;

    public static double LIGHT_ON = 1f;
    public static double LIGHT_OFF = 0f;

    // INIT
    public static double CLAW_PIVOT_INIT = 0f;
    public static double CLAW_SHOULDER_INIT = 0f;

    public static double CLAW_ELBOW_INIT = 0f;

    public static double CLAW_WRIST_INIT = 0f;

    // HOME
    public static double HORZ_HOME_PIVOT = .7f;
    public static double HORZ_HOME_SHOULDER = .5f;
    public static double HORZ_HOME_ELBOW = 1f;

    public static double HORZ_HOME_WRIST = 0f;

    // DEPLOYED

    public static double HORZ_DEPLOYED_PIVOT = 0.55f;
    public static double HORZ_DEPLOYED_SHOULDER = .5f;
    public static double HORZ_DEPLOYED_SHOULDER_DOWN = .68f;
    // .275 - .825
    public static double HORZ_DEPLOYED_ELBOW = 1f;
    // 0 - .6
    public static double HORZ_DEPLOYED_WRIST = 0f;

    // WALL

    public static double HORZ_WALL_PIVOT = 0.1f;
    public static double HORZ_WALL_SHOULDER = .75f;
    // .275 - .825
    public static double HORZ_WALL_ELBOW = 0.35f;
    // 0 - .6
    public static double HORZ_WALL_WRIST = 0f;

    // HANDSHAKE
    public static double HORZ_HANDSHAKE_PIVOT = 0f;
    public static double HORZ_HANDSHAKE_SHOULDER = .55f;
    public static double HORZ_HANDSHAKE_ELBOW = .4f;

    public static double HORZ_HANDSHAKE_WRIST = 0f;





    //      VERTICAL

    // home

    public static double VERT_HOME_SHOULDER = 0.16f;
    public static double VERT_HOME_ROTATION = 0f;
    public static double VERT_HOME_ELBOW = 1f;

    // hang

    public static double VERT_HANG_SHOULDER = 0.85f;
    public static double VERT_HANG_ROTATION = 0.55f;
    public static double VERT_HANG_ELBOW = 0.85f;

    // wall

    public static double VERT_WALL_SHOULDER = 0.16f;
    public static double VERT_WALL_ROTATION = 0.0f;
    public static double VERT_WALL_ELBOW = 0.8f;

    // HANDSHAKE

    public static double VERT_BASKET_SHOULDER = 0.7f;
    public static double VERT_BASKET_ROTATION = 0.55f;
    public static double VERT_BASKET_ELBOW = 0.6f;

    
}
