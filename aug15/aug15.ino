#include <HardwareSerial.h>

// MDDS30A Motor Driver Pin Definitions for ESP32
// Motor 1 (Front Right)
#define MOTOR1_PWM_PIN    2   // PWM signal to MDDS PWM1
#define MOTOR1_DIR_PIN    4   // Direction signal to MDDS DIR1

// Motor 2 (Front Left) 
#define MOTOR2_PWM_PIN    5   // PWM signal to MDDS PWM2
#define MOTOR2_DIR_PIN    18  // Direction signal to MDDS DIR2

// Motor 3 (Rear Left)
#define MOTOR3_PWM_PIN    19  // PWM signal to MDDS PWM3
#define MOTOR3_DIR_PIN    21  // Direction signal to MDDS DIR3

// Motor 4 (Rear Right)
#define MOTOR4_PWM_PIN    22  // PWM signal to MDDS PWM4
#define MOTOR4_DIR_PIN    23  // Direction signal to MDDS DIR4

// ESP32 PWM Configuration for MDDS compatibility
#define PWM_FREQUENCY     1000  // 1kHz - optimal for MDDS motor drivers
#define PWM_RESOLUTION    8     // 8-bit resolution (0-255)
#define PWM_MAX_VALUE     255   // Maximum PWM value for 8-bit

// Movement Commands
typedef enum {
    STOP = 0,
    FORWARD = 1,
    REVERSE = 2,
    LEFT = 3,
    RIGHT = 4,
    ROTATE_CW = 5,
    ROTATE_CCW = 6
} movement_command_t;

// Motor structure
typedef struct {
    int pwm_pin;
    int dir_pin;
} motor_t;

// Initialize motors array
motor_t motors[4] = {
    {MOTOR1_PWM_PIN, MOTOR1_DIR_PIN},  // Front Right
    {MOTOR2_PWM_PIN, MOTOR2_DIR_PIN},  // Front Left
    {MOTOR3_PWM_PIN, MOTOR3_DIR_PIN},  // Rear Left
    {MOTOR4_PWM_PIN, MOTOR4_DIR_PIN}   // Rear Right
};

// Function prototypes
void init_motors(void);
void set_motor_speed(int motor_index, int speed, int direction);
void execute_movement(movement_command_t command, int speed);
void stop_all_motors(void);

// Initialize PWM and GPIO for all motors - ESP32 specific for MDDS
void init_motors(void) {
    Serial.println("Initializing ESP32 for MDDS Motor Driver...");
    
    // Configure each motor with ESP32 LEDC PWM
    for (int i = 0; i < 4; i++) {
        // Configure PWM channel - ESP32 specific function
        if (!ledcAttach(motors[i].pwm_pin, PWM_FREQUENCY, PWM_RESOLUTION)) {
            Serial.print("Failed to attach PWM to pin ");
            Serial.println(motors[i].pwm_pin);
            return;
        }
        
        // Initialize PWM to 0 (motors stopped)
        ledcWrite(motors[i].pwm_pin, 0);
        
        // Configure direction pin as output
        pinMode(motors[i].dir_pin, OUTPUT);
        digitalWrite(motors[i].dir_pin, LOW);
        
        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.print(" - PWM Pin: ");
        Serial.print(motors[i].pwm_pin);
        Serial.print(", DIR Pin: ");
        Serial.println(motors[i].dir_pin);
    }
    
    Serial.println("ESP32 Motors initialized successfully for MDDS!");
    Serial.print("PWM Frequency: ");
    Serial.print(PWM_FREQUENCY);
    Serial.println(" Hz (MDDS compatible)");
    Serial.print("PWM Resolution: ");
    Serial.print(PWM_RESOLUTION);
    Serial.println(" bits");
    Serial.print("PWM Range: 0-");
    Serial.println(PWM_MAX_VALUE);
}

// Set individual motor speed and direction
void set_motor_speed(int motor_index, int speed, int direction) {
    if (motor_index < 0 || motor_index > 3) return;
    
    // Clamp speed to valid range
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    
    // Convert speed percentage to PWM value
    int pwm_value = (speed * PWM_MAX_VALUE) / 100;
    
    // Set direction (0 = forward, 1 = reverse for motor)
    digitalWrite(motors[motor_index].dir_pin, direction);
    
    // Set PWM value (new ESP32 Arduino Core 3.x syntax)
    ledcWrite(motors[motor_index].pwm_pin, pwm_value);
}

// Execute movement based on command using switch-case
void execute_movement(movement_command_t command, int speed) {
    Serial.print("Executing movement command: ");
    Serial.print(command);
    Serial.print(" at speed: ");
    Serial.println(speed);
    
    switch(command) {
        case FORWARD:
            // All motors forward for X-drive forward movement
            set_motor_speed(0, speed, 0);  // Front Right - Forward
            set_motor_speed(1, speed, 1);  // Front Left - Reverse (opposite mounting)
            set_motor_speed(2, speed, 1);  // Rear Left - Reverse (opposite mounting)
            set_motor_speed(3, speed, 0);  // Rear Right - Forward
            Serial.println("Moving FORWARD");
            break;
            
        case REVERSE:
            // All motors reverse for X-drive backward movement
            set_motor_speed(0, speed, 1);  // Front Right - Reverse
            set_motor_speed(1, speed, 0);  // Front Left - Forward
            set_motor_speed(2, speed, 0);  // Rear Left - Forward
            set_motor_speed(3, speed, 1);  // Rear Right - Reverse
            Serial.println("Moving REVERSE");
            break;
            
        case LEFT:
            // Strafe left - diagonal motors work together
            set_motor_speed(0, speed, 1);  // Front Right - Reverse
            set_motor_speed(1, speed, 1);  // Front Left - Reverse
            set_motor_speed(2, speed, 0);  // Rear Left - Forward
            set_motor_speed(3, speed, 0);  // Rear Right - Forward
            Serial.println("Moving LEFT");
            break;
            
        case RIGHT:
            // Strafe right - diagonal motors work together
            set_motor_speed(0, speed, 0);  // Front Right - Forward
            set_motor_speed(1, speed, 0);  // Front Left - Forward
            set_motor_speed(2, speed, 1);  // Rear Left - Reverse
            set_motor_speed(3, speed, 1);  // Rear Right - Reverse
            Serial.println("Moving RIGHT");
            break;
            
        case ROTATE_CW:
            // Clockwise rotation - all motors same direction
            set_motor_speed(0, speed, 0);  // Front Right - Forward
            set_motor_speed(1, speed, 1);  // Front Left - Reverse
            set_motor_speed(2, speed, 1);  // Rear Left - Reverse
            set_motor_speed(3, speed, 1);  // Rear Right - Reverse
            Serial.println("Rotating CLOCKWISE");
            break;
            
        case ROTATE_CCW:
            // Counter-clockwise rotation
            set_motor_speed(0, speed, 1);  // Front Right - Reverse
            set_motor_speed(1, speed, 0);  // Front Left - Forward
            set_motor_speed(2, speed, 0);  // Rear Left - Forward
            set_motor_speed(3, speed, 0);  // Rear Right - Forward
            Serial.println("Rotating COUNTER-CLOCKWISE");
            break;
            
        case STOP:
        default:
            stop_all_motors();
            Serial.println("STOPPED");
            break;
    }
}

// Stop all motors
void stop_all_motors(void) {
    for (int i = 0; i < 4; i++) {
        set_motor_speed(i, 0, 0);
    }
}

// Setup function - runs once on ESP32
void setup() {
    Serial.begin(115200);
    
    Serial.println("ESP32 Holonomic X-Drive Robot Starting");
    Serial.println("Compatible with MDDS Motor Drivers");
    
    // Initialize motors for ESP32-MDDS communication
    init_motors();
    
    Serial.println("✓ ESP32 Robot ready for MDDS control!");
    Serial.println("Starting demo sequence...");
}

// Main loop function
void loop() {
    // Demo movement sequence
    int demo_speed = 70;  // 70% speed
    
    // Forward for 2 seconds
    execute_movement(FORWARD, demo_speed);
    delay(2000);
    
    // Stop for 1 second
    execute_movement(STOP, 0);
    delay(1000);
    
    // Left strafe for 2 seconds
    execute_movement(LEFT, demo_speed);
    delay(2000);
    
    // Stop for 1 second
    execute_movement(STOP, 0);
    delay(1000);
    
    // Reverse for 2 seconds
    execute_movement(REVERSE, demo_speed);
    delay(2000);
    
    // Stop for 1 second
    execute_movement(STOP, 0);
    delay(1000);
    
    // Right strafe for 2 seconds
    execute_movement(RIGHT, demo_speed);
    delay(2000);
    
    // Stop for 1 second
    execute_movement(STOP, 0);
    delay(1000);
    
    // Rotate clockwise for 2 seconds
    execute_movement(ROTATE_CW, demo_speed);
    delay(2000);
    
    // Stop for 1 second
    execute_movement(STOP, 0);
    delay(1000);
    
    Serial.println("Demo sequence completed, restarting...");
}

/* 
 * WIRING GUIDE: ESP32 to MDDS Motor Driver Connection
 * ==================================================
 * 
 * ESP32 GPIO -> MDDS30A Pins
 * -------------------------
 * Motor 1 (Front Right):
 *   GPIO 2  -> MDDS PWM1 (PWM signal)
 *   GPIO 4  -> MDDS DIR1 (Direction control)
 * 
 * Motor 2 (Front Left):
 *   GPIO 5  -> MDDS PWM2 (PWM signal)
 *   GPIO 18 -> MDDS DIR2 (Direction control)
 * 
 * Motor 3 (Rear Left):
 *   GPIO 19 -> MDDS PWM3 (PWM signal)
 *   GPIO 21 -> MDDS DIR3 (Direction control)
 * 
 * Motor 4 (Rear Right):
 *   GPIO 22 -> MDDS PWM4 (PWM signal)
 *   GPIO 23 -> MDDS DIR4 (Direction control)
 * 
 * Power & Ground:
 *   ESP32 GND -> MDDS GND (Common ground)
 *   ESP32 3V3 -> MDDS VCC (Logic power - if needed)
 *   Battery + -> MDDS VIN (Motor power 12V/24V)
 *   Battery - -> MDDS GND
 * 
 * IMPORTANT NOTES:
 * - ESP32 outputs 3.3V logic levels (MDDS compatible)
 * - MDDS PWM frequency: 1kHz (optimal for motor control)
 * - Use external power supply for motors (not USB power)
 * - Connect ESP32 and MDDS grounds together
 * 
 * USAGE:
 * - Call execute_movement() with desired command and speed (0-100%)
 * - Use the movement_command_t enum values
 * - ESP32 will generate proper PWM/DIR signals for MDDS
 * - Motors automatically coordinate for holonomic X-drive movement
 */
