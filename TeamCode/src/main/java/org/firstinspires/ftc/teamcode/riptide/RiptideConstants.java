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

    public static int HANDSHAKE_WAIT_TIME = 350;


    //   HORIZONTAL PID
    public static double HORIZONTAL_PID_P = 0.0125;
    public static double HORIZONTAL_PID_I = 0.001;
    public static double HORIZONTAL_PID_D = 0.00;
    public static double HORIZONTAL_PID_F = 0.0;


    //   VERTICAL PID
    public static double VERTICAL_PID_P = 0.0125;
    public static double VERTICAL_PID_I = 0.0;
    public static double VERTICAL_PID_D = 0.00;
    public static double VERTICAL_PID_F = 0.0;



    //   VERTICAL SLIDE POSITION
    public static int VERTICAL_SLIDE_HOME = 0;
    public static int VERTICAL_SLIDE_WALL = 0;
    public static int VERTICAL_SLIDE_HANG = 425;
    public static int VERTICAL_SLIDE_BASKET = 1875;

    public static final int HEIGHT_LIMIT_WHEN_HORIZONTAL_DEPLOYED = VERTICAL_SLIDE_HANG;



    //   HORIZONTAL SLIDE POSITIONS
    public static int HORIZONTAL_SLIDE_HOME = 0;
    public static int HORIZONTAL_SLIDE_HANDSHAKE = 675;

    public static int HORIZONTAL_SLIDE_MAX = 1400;
    public static final int LENGTH_LIMIT_WHEN_VERTICAL_DEPLOYED = 100;


    //   CLAW GRIP VALUES
    public static double GRIPPER_CLOSED_VALUE_VERTICAL = 0.2f;
    public static double GRIPPER_OPEN_VALUE_VERTICAL = .5f;
    public static double GRIPPER_OPEN_VALUE_HORIZONTAL = .26f;
    public static double GRIPPER_CLOSED_VALUE_HORIZONTAL = 0.125;
    public static double GRIPPER_CLOSED_HANDSHAKE_VALUE_HORIZONTAL = 0.15;



                          //    HORIZONTAL CLAW PRESETS
    // HOME
    public static double HORZ_HOME_SHOULDER = .75f;
    public static double HORZ_HOME_ELBOW = .325f;

    public static double HORZ_HOME_WRIST = .5f;

    // DEPLOYED

    public static double HORZ_DEPLOYED_SHOULDER = .4f;
    public static double HORZ_DEPLOYED_SHOULDER_DOWN = .25f;
    // .275 - .825
    public static double HORZ_DEPLOYED_ELBOW = .55f;
    // 0 - .6
    public static double HORZ_DEPLOYED_WRIST = .5f;


    // HANDSHAKE
    public static double HORZ_HANDSHAKE_SHOULDER = .59f;
    public static double HORZ_HANDSHAKE_ELBOW = .95f;

    public static double HORZ_HANDSHAKE_WRIST = .5f;

    // HOCKEY
    public  static double HOCKEY_UP = .85;
    public  static double HOCKEY_DOWN = .15;



                       //       VERTICAL CLAW PRESETS

    // HOME

    public static double VERT_HOME_SHOULDER = .9;
    public static double VERT_HOME_ROTATION = .85;
    public static double VERT_HOME_ELBOW = 1;

    // HANDSHAKE
    public static double VERT_HANDSHAKE_SHOULDER = .95;
    public static double VERT_HANDSHAKE_ELBOW = .925;
    public static double VERT_HANDSHAKE_ROTATION = .85;

    // HANG
    public static double VERT_HANG_SHOULDER = .175;
    public static double VERT_HANG_ROTATION = 0.3f;
    public static double VERT_HANG_ELBOW = 0.2f;

    // WALL
    public static double VERT_WALL_SHOULDER = 0.98;
    public static double VERT_WALL_ROTATION = 0.85f;
    public static double VERT_WALL_ELBOW = 1;

    // BASKET
    public static double VERT_BASKET_SHOULDER = 0.4f;
    public static double VERT_BASKET_ROTATION = 0.375f;
    public static double VERT_BASKET_ELBOW = 0.55f;

    // INIT
    public static double VERT_INIT_SHOULDER = 0.8;
    public static double VERT_INIT_ELBOW = 1;


    public static double SPEED_SERVO_SLOW=.75;
    public static double SPEED_SERVO_FAST =1;

}
