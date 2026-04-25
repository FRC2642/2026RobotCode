package frc.robot;

public class Constants {

    //DEVICE IDS
        public static int MAIN_CONTROLLER_PORT = 0;
        public static int AUX_CONTROLLER_PORT = 1;
        public static int BUTTON_BOARD_PORT = 2;

        public static int FLYWHEEL_MOTOR_1 = 17;
        public static int FLYWHEEL_MOTOR_2 = 20;
        public static int FLYWHEEL_MOTOR_4 = 23;

        public static int SHOOTER_ROLLER_MOTOR_1 = 19;
        public static int SHOOTER_ROLLER_MOTOR_2 = 18;

        public static int INTERMEDIATE_MOTOR = 15;

        public static int INTAKE_TILT_MOTOR = 14;
        public static int INTAKE_TILT_ENCODER = 9;
        public static int INTAKE_SPIN_MOTOR = 16;
    //CURRENT LIMITS
        public static double FLYWHEEL_CURRENT_LIMIT = 25;
        public static double SHOOTER_ROLLER_CURRENT_LIMIT = 25;
        public static double INTERMEDIATE_CURRENT_LIMIT = 20;
        public static double INTAKE_SPIN_CURRENT_LIMIT = 20;

    //SPEEDS
        public static double INTAKE_SPIN_SPEED = 0.70;
        public static double MANUAL_TILT_SPEED = 0.3;
        public static double REVERSE_SHOOTER_SPEED = -0.3;
        public static double INTERMEDIATE_SPEED = 0.3;

        public static double SHOOTER_ROLLER_1_SPEED = 0.4;
        public static double SHOOTER_ROLLER_2_SPEED = 0.6;
        public static double SHOOTER_FLYWHEEL_SPEED = 50;
        public static double START_FLYWHEEL_SPEED = 50;
        public static double PASSING_FLYWHEEL_SPEED = 100;
}
