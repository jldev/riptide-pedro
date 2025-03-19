package org.firstinspires.ftc.teamcode.riptide;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RiptideConstants {


    public static double IN_PER_TICK = .00104;


    // SLIDE CONTROL

    public static int VERT_SLIDE_MANUAL_SPEED = 25;
    public static int HORZ_SLIDE_MANUAL_SPEED = 50;

    public static double SLIDES_PID_TOLERANCE = 10;
    public static double SLIDES_PID_POS_COEFFICIENT = .25;


    //   HORIZONTAL PID
    public static double HORIZONTAL_PID_P = 0.025;
    public static double HORIZONTAL_PID_I = 0.001;
    public static double HORIZONTAL_PID_D = 0.00;
    public static double HORIZONTAL_PID_F = 0.0;


    //   VERTICAL PID
    public static double VERTICAL_PID_P = 0.01;
    public static double VERTICAL_PID_I = 0.0;
    public static double VERTICAL_PID_D = 0.00;
    public static double VERTICAL_PID_F = 0.0;



    //   VERTICAL SLIDE POSITION
    public static int VERTICAL_SLIDE_HOME = 0;
    public static int VERTICAL_SLIDE_WALL = 0;
    public static int VERTICAL_SLIDE_HANG = 400;
    public static int VERTICAL_SLIDE_BASKET = 2725;

    public static final int HEIGHT_LIMIT_WHEN_HORIZONTAL_DEPLOYED = VERTICAL_SLIDE_HANG;



    //   HORIZONTAL SLIDE POSITIONS
    public static int HORIZONTAL_SLIDE_HOME = 0;
    public static int HORIZONTAL_SLIDE_HANDSHAKE = 30;

    public static int HORIZONTAL_SLIDE_MAX = 1425;
    public static final int LENGTH_LIMIT_WHEN_VERTICAL_DEPLOYED = 100;


    //   CLAW GRIP VALUES
    public static double GRIPPER_CLOSED_VALUE_VERTICAL = 0.2f;
    public static double GRIPPER_OPEN_VALUE_VERTICAL = .4f;
    public static double GRIPPER_OPEN_VALUE_HORIZONTAL = .3f;
    public static double GRIPPER_CLOSED_VALUE_HORIZONTAL = 0.05;
    public static double GRIPPER_CLOSED_HANDSHAKE_VALUE_HORIZONTAL = 0.15;



                          //    HORIZONTAL CLAW PRESETS
    // HOME
    public static double HORZ_HOME_SHOULDER = .75f;
    public static double HORZ_HOME_ELBOW = .05f;

    public static double HORZ_HOME_WRIST = .48f;

    // DEPLOYED

    public static double HORZ_DEPLOYED_SHOULDER = .25f;
    public static double HORZ_DEPLOYED_SHOULDER_DOWN = .075f;
    // .275 - .825
    public static double HORZ_DEPLOYED_ELBOW = .3f;
    // 0 - .6
    public static double HORZ_DEPLOYED_WRIST = .48f;


    // HANDSHAKE
    public static double HORZ_HANDSHAKE_SHOULDER = .25f;
    public static double HORZ_HANDSHAKE_ELBOW = .875f;

    public static double HORZ_HANDSHAKE_WRIST = .48f;



                       //       VERTICAL CLAW PRESETS

    // HOME

    public static double VERT_HOME_SHOULDER = 0.15f;
    public static double VERT_HOME_ROTATION = 0f;
    public static double VERT_HOME_ELBOW = 1;

    // HANDSHAKE
    public static double VERT_HANDSHAKE_SHOULDER = 0.03;
    public static double VERT_HANDSHAKE_ELBOW = .985;

    // HANG
    public static double VERT_HANG_SHOULDER = 0.83f;
    public static double VERT_HANG_ROTATION = 0.55f;
    public static double VERT_HANG_ELBOW = 0.8f;

    // WALL
    public static double VERT_WALL_SHOULDER = 0.175f;
    public static double VERT_WALL_ROTATION = 0.0f;
    public static double VERT_WALL_ELBOW = 0.85f;

    // BASKET
    public static double VERT_BASKET_SHOULDER = 0.7f;
    public static double VERT_BASKET_ROTATION = 0.55f;
    public static double VERT_BASKET_ELBOW = 0.6f;

    
}
