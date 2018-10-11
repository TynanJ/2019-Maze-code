#include <Arduino.h>
#include "Motor.h"
#include "Config.h"
#include "IMU.h"
#include "Common.h"
#include <Wire.h>
// #include "Thermal.h"
#include "ColorSensor.h"
#include "MotorController.h"
#include "LRF.h"
#include "LightSensor.h"
#include "LED.h"
// #include "PID.h"


// Thermal MLX90614;

Motor motor1;
Motor motor2;
Motor motor3;
Motor motor4;

MotorController robot;

LRF Sensor1;
LRF Sensor2;
LRF Sensor3;
LRF Sensor4;

LRF distances[4] = {Sensor1, Sensor2, Sensor3, Sensor4};

IMU mpu9250;
// PID pid(1, 1, 1, 255);

ColorSensor Color1;
ColorSensor Color2;
ColorSensor Color3;
ColorSensor Color4;

ColorSensor colors[4] = {Color1, Color2, Color3, Color4};

LightSensor light_N(LIGHT_OUT_N);
LightSensor light_E(LIGHT_OUT_E);
LightSensor light_S(LIGHT_OUT_S);
LightSensor light_W(LIGHT_OUT_W);

LightSensor lights[4] = {light_N, light_E, light_S, light_W};

LED diode(LED_PIN);

// Calibrated values
uint16_t LIGHT_THRESHOLD[4] = {LIGHT_FRONT, LIGHT_RIGHT, LIGHT_BACK, LIGHT_LEFT};
uint16_t COLOR_THRESHOLD[4] = {COLOR_FRONT, COLOR_RIGHT, COLOR_BACK, COLOR_LEFT};


// Variable Initialisation
int8_t dir_of_travel = 0;
int8_t prev_dir_of_travel = 100;
int32_t maze[MAZE_Y][MAZE_X];
int16_t visited_nodes = 0;
Vector2D permutations[4] = {{0,1}, {1,0}, {0,-1}, {-1,0}};


// Current sensor readings
uint16_t LRF_data[4];
uint16_t light_data[4];
IntVector3D colorsensor_data[4];
double current_heading;
int current_x = int(MAZE_X/2 + 0.5);
int current_y = int(MAZE_Y/2 + 0.5);


// Previous sensor readings
uint16_t prev_LRF_data[4];
uint16_t prev_light_data[4];
IntVector3D prev_colorsensor_data[4];
double prev_current_heading;


void LRFinit() {
    // Shutdown pins of LRF ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
    pinMode(VL53L0X_XSHUT_pin1, OUTPUT);
    pinMode(VL53L0X_XSHUT_pin2, OUTPUT);
    pinMode(VL53L0X_XSHUT_pin3, OUTPUT);
    // delay(80);

    //Change address of sensor and power up next one
    for(int i=3; i>0; i--) {
        distances[i].setAddress(VL53L0x_DEFAULT_ADDRESS+i);
        pinMode(VL53L0X_XSHUT_pin1+(i-1), INPUT);
        delay(80);
    }

    for(int i=0; i<4; i++) {
        distances[i].init();
    }

    for(int i=0; i<4; i++) {
        distances[i].setTimeout(500);
        // distances[i].startContinuous();
        distances[i].setMeasurementTimingBudget(200000);
    }
}

uint8_t findPath(int x, int y, uint8_t prev_dir) {
    // Is on black?

    uint8_t is_black = 0;
    for(int i=0; i<4; i++) {
        if(light_data[i] > LIGHT_THRESHOLD[i]) {
            is_black++;
        } 
    }

    if(is_black >= 2) {
        // All sensors say that we are on black
        maze[y][x] = 9999;
        return prev_dir;
    }
    

    // Not on black. Label current node
    if(maze[y][x] == -1) {
        // Node was undiscovered
        maze[y][x] = visited_nodes;
    } else {
        // Node was discovered previously
        visited_nodes = maze[y][x];
    }
    

    // Connect adjacent nodes
    int32_t adjacent_nodes[4];
    for(int i=0; i<4; i++) {
        if(LRF_data[i] > WALL_THRESHOLD) {
            adjacent_nodes[i] = maze[int(current_y + permutations[i].y)][int(current_x + permutations[i].x)];
        } else {
            adjacent_nodes[i] = -2;
        }
    }

    // Find a good path
    uint8_t dir_to_travel = 0;
    while(true) {
        if(adjacent_nodes[dir_to_travel%4] == -1) {
            // Unvisited node found
            return dir_to_travel%4;
        } 

        if(dir_to_travel >= 4) {     
            // Failed to find an unvisited node. Find the n-1 node
            if(visited_nodes == 0) {
                // Maze has been completed
                return 255;
            } else if(adjacent_nodes[dir_to_travel%4] == visited_nodes-1) {
                return dir_to_travel%4;
            }
        } 
        dir_to_travel++;
    }
}

void delayWithIMU(int time_to_wait){
    unsigned int start_time = millis();
    while(millis() - start_time < time_to_wait) {mpu9250.update();}
}




void setup()
{   
    Wire.begin();
    Serial.begin(9600);

    // Initialise maze
    for(int i = 0; i < MAZE_X; i++) {
        for(int j = 0; j < MAZE_Y; j++) {
            maze[j][i] = -1;
        }
    }

    // Initialise Motors
    motor1.init(mA_en, mA_in1, mA_in2);
    motor2.init(mB_en, mB_in1, mB_in2);
    motor3.init(mC_en, mC_in1, mC_in2);
    motor4.init(mD_en, mD_in1, mD_in2);

    robot.init(&motor1, &motor2, &motor3, &motor4);


    // Initialise Color distances
    Color1.init(TCS230_Sensor1_S0, TCS230_Sensor1_S1, TCS230_Sensor1_S2, TCS230_Sensor1_S3, TCS230_Sensor1_OUT);
    Color2.init(TCS230_Sensor2_S0, TCS230_Sensor2_S1, TCS230_Sensor2_S2, TCS230_Sensor2_S3, TCS230_Sensor2_OUT);
    Color3.init(TCS230_Sensor3_S0, TCS230_Sensor3_S1, TCS230_Sensor3_S2, TCS230_Sensor3_S3, TCS230_Sensor3_OUT);
    Color4.init(TCS230_Sensor4_S0, TCS230_Sensor4_S1, TCS230_Sensor4_S2, TCS230_Sensor4_S3, TCS230_Sensor4_OUT);

    ColorSensor temp_colors[4] = {Color1, Color2, Color3, Color4};
    
    for(int i=0; i<4; i++) {
        colors[i] = temp_colors[i];
        // colors[i].calibrate();
    }

    // Initialise LRFs
    LRFinit();
    

    // Initialise IMU
    mpu9250.init();
    mpu9250.calibrate();
}


void loop() {
    // Read sensors (Except IMU)
    for(int i=0; i<4; i++) {
        LRF_data[i] = distances[i].readRangeSingleMillimeters();
        colorsensor_data[i] = colors[i].readColor();
        light_data[i] = lights[i].readLight();
    }
    
    // Determine state of victims
    // for(int i=0; i<4; i++) {
    //     if(colorsensor_data[i].x < COLOR_THRESHOLD[i]) {
    //         diode.setPower(255);
    //         delayWithIMU(3000);
    //         diode.setPower(0);
    //     }
    // }

    // Find a path
    dir_of_travel = findPath(current_x, current_y, prev_dir_of_travel);

    // Travel on path
    if(dir_of_travel != 255) {
        robot.moveAll(BASELINE_SPEED, BASELINE_SPEED, dir_of_travel*90);
        delayWithIMU(TIME_TO_TRAVEL + (EXTRA_SIDEWAYS_TIME * (dir_of_travel%2 != 0)));
    }

    // Stop the robot
    robot.stop();
    delayWithIMU(300);
    // Angular Correction
    mpu9250.update();

    uint16_t angle = mpu9250.heading;
    robot.turn(50, 50, angle > 180);
    while(3 < angle && angle < 357) {
        mpu9250.update();
        angle = mpu9250.heading;
    }
    robot.stop();

    // Change robot position
    current_x += permutations[dir_of_travel].x;
    current_y += permutations[dir_of_travel].y;
    visited_nodes++;

    // Current values are now previous values  
    prev_current_heading = current_heading;

    for(int i=0; i<4; i++) {
        prev_LRF_data[i] = LRF_data[i];
        prev_light_data[i] = light_data[i];
        prev_colorsensor_data[i] = colorsensor_data[i];
    }

    prev_dir_of_travel = (dir_of_travel+2)%4;
    
    // Final delay to let robot settle
    delayWithIMU(1000);
}



 /* MAZE ALGORITHM
    1. Read all distances
    
    2. Otherwise, check for a victim with colors
    3. IF there is a victim, acknowledge with short flash
    4. Check for victims with heat
    5. IF there is a victim, acknowledge with long flash
    6. Otherwise, DETERMINE A PATH TO TRAVEL 
        1. IF on a black tile, retreat in previous direction
        2. otherwise, check for directions NOT the one previously travelled
        3. IF there are no free directions other than the ones previously travelled, select the previous tile
    7. Travel on the path picked
    8. Set current values to previous values
 */
