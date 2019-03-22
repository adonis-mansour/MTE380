#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

// IMU Libs
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// TOF Libs
#include <SparkFun_RFD77402_Arduino_Library.h>

/////////////////////////////////////////////
///////////// SENSOR DEFS  /////////////
/////////////////////////////////////////////

//PIN CONSTANTS
// FLAME SENSOR DEFS
#define FLAME1 50 
#define FLAME2 48 
#define FLAME3 51
#define FLAME4 39 
#define FAN 52

//ULTRASONIC DEFS
#define trigPin 8  //Trigger                                                               
#define echoPin0 9  //Front_Left
#define echoPin1 13 //Front_Right
#define echoPin2 5  //Right_Front
#define echoPin3 6  //Right_Back
#define echoPin4 2  //Left_Front
#define echoPin5 3  //Left_Back
#define echoPin6 7  //Rear_R                                     
//#define echoPin7 4  //Rear_L

//TOF SENSOR DATA TRANSFER
#define TOF_REQUEST 23
#define TOF_SENT 22

// HALL EFFECT DEFS
#define HE0 A0
#define HE1 A1
#define HE2 A2
#define HE3 A3
#define HE4 A4
#define HE5 A5
#define HE6 A6
#define HE7 A7
#define HE8 A13

//COLOUR SENSOR RECEIVER DEFS
#define Colour_REQUEST 25
#define Colour_SENT 24

//EXTRA
#define Colour_RED_HOUSE 49
#define Colour_YELLOW_HOUSE 50
#define Colour_OTHER 51

//IR SENSOR 
#define sensor0 A9  //F                              
#define sensor1 A10  //R
#define sensor2 A11  //L
#define sensor3 A12  //B

// Motor Definitions
// Right Motors
#define in1 32
#define in2 30
#define in3 28
#define in4 26
#define en1 12 //RL
#define en2 11 //FL

// Left Motors
#define in5 33
#define in6 31
#define in7 29
#define in8 27
#define en3 10 // FR
#define en4 4  // RR

// PID Consts
#define Kp 3
#define Ki 0.5
#define Kd 0.001

////////////////////////////////////////////

/////////////////////////////////////////////
///////////// SENSOR CONSTS  ///////////////
/////////////////////////////////////////////

//ULTRASONIC CONSTANTS
long US_duration, US_distance; 
int US_counter = 0, US = 0;
boolean sensors_detected[8] = {0};
boolean object_location[4] = {0};

const int US_NumOfValues = 5;
int US0_values[US_NumOfValues]={0};
int US1_values[US_NumOfValues]={0};
int US2_values[US_NumOfValues]={0};
int US3_values[US_NumOfValues]={0};
int US4_values[US_NumOfValues]={0};
int US5_values[US_NumOfValues]={0};
int US6_values[US_NumOfValues]={0};
int US7_values[US_NumOfValues]={0};

//  IR CONSTANTS
int IR_counter = 0;
float volts = 0;
int IR_distance = 0;
int terrain[4]; //0 = flat, 1 = sand, 2 = gravel, 3 = pit

const int NumOfValues = 5;
int IR0_values[NumOfValues]={0};
int IR1_values[NumOfValues]={0};
int IR2_values[NumOfValues]={0};
int IR3_values[NumOfValues]={0};
int IR, IR0, IR1, IR2, IR3;

// IMU Vars
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float target_angle_x = 0;

// TOF Sensor
RFD77402 tof_distance;
////////////////////////////////////////////

/////////////////////////////////////////////
///////////// PATH FINDING STUFF /////////////
/////////////////////////////////////////////
//STRUCTS
typedef struct coordinates{
    int x;
    int y;
}coordinates;

typedef struct tile{
    coordinates location;
    struct tile* leftSquare;
    struct tile* rightSquare;
    struct tile* behindSquare;
    struct tile* frontSquare;
    struct tile* parent;
    int type; //0 for regular (flat), 1 for sand, 2 for water
    //int hCost,fCost,gCost;
    bool isRevealed;
    bool isObstacle;
}tile;

typedef struct node{
    struct node* parent;
    struct node* child;
    struct tile* s;
}node;

typedef struct list{
    struct node* headNode;
}list;

//GLOBAL VARIABLES  - PATHFINDING
const int DIMENSION = 6;
const int STARTINGX = 3;
const int STARTINGY = 0;
int orientation = 0;
int currentx;
int currenty;
bool fireFound = false;
bool foodFound = false;
bool survivorFound = false;
bool groupFound = false;
tile* food;
tile* survivor;
tile* group;

//MOCK OBJECTIVE LOCATIONS/INFORMATION
tile* mockFire;
tile* mockFood;
tile* mockSurvivor;
tile* mockGroup;
tile* mockSand1,*mockSand2,*mockSand3,*mockSand4;
tile* mockWater1,*mockWater2,*mockWater3,*mockWater4;


//INITIALIZE ALL SQUARES IN A BOARD
void createBoard(tile b[][DIMENSION], int dim){
    for(int i = 0; i < dim; i++){
        for (int j = 0; j < dim; j++){
            b[i][j].location.x = i;
            b[i][j].location.y = j;
            b[i][j].isRevealed = false;
            b[i][j].isObstacle = false;
            if (i != 0) b[i][j].leftSquare = &b[i-1][j];
            else  b[i][j].leftSquare = NULL;
            if (i != DIMENSION -1) b[i][j].rightSquare = &b[i+1][j];
            else  b[i][j].rightSquare = NULL;
            if (j != 0) b[i][j].behindSquare = &b[i][j-1];
            else  b[i][j].behindSquare = NULL;
            if (j != DIMENSION - 1) b[i][j].frontSquare = &b[i][j+1];
            else  b[i][j].frontSquare = NULL;
            b[i][j].parent = NULL;
        }
    }
    return;
}

//FIND SHORTEST DISTANCE BETWEEN TWO NODES REGARDLESS OF OBSTACLES
int findDistance(coordinates c1, coordinates c2){
    int xDistance = abs(c1.x-c2.x);
    int yDistance = abs(c1.y-c2.y);

    if (xDistance > 0 && yDistance > 0){
        xDistance--;
        yDistance--;
        return (10*(xDistance+yDistance) + 14);
    }

    else return 10*(xDistance + yDistance);
}

//SET F, G & H COSTS OF A SQUARE
void setCosts(tile* current, tile* previous, tile* end, int gCost[][DIMENSION], int hCost[][DIMENSION], int fCost[][DIMENSION], bool override){

    if (current == NULL) return;

    coordinates c = current->location;
    coordinates cP = previous->location;
    hCost[c.x][c.y] = findDistance(c, end->location);

    if (current->isRevealed && current->type != 2 && !current->isObstacle && !override){
        int g = gCost[cP.x][cP.y] + findDistance(c,cP);
        if (g < gCost[c.x][c.y]){
            current->parent = previous;
            gCost[c.x][c.y] = g;
        }
    }

    else if (current->isRevealed && current->type != 2 && current->isObstacle && current == end){
        int g = gCost[cP.x][cP.y] + findDistance(c,cP);
        if (g < gCost[c.x][c.y]){
            current->parent = previous;
            gCost[c.x][c.y] = g;
        }
    }

    fCost[c.x][c.y] = gCost[c.x][c.y] + hCost[c.x][c.y];
    return;
}

//IMPLEMENTATION OF A* ALGORITHM
list findPath(tile* starting, tile* end, tile b[][DIMENSION]){
    list path;
    if(starting == end) {
        path.headNode = NULL;
        return path;
    }
    tile* current = starting;
    int gCost[DIMENSION][DIMENSION];
    int hCost[DIMENSION][DIMENSION];
    int fCost[DIMENSION][DIMENSION];
    //bool nextToWater[DIMENSION][DIMENSION];
    bool isTraversed[DIMENSION][DIMENSION];

    for(int i = 0; i < DIMENSION; i++){
        for(int j = 0; j < DIMENSION; j++){
            gCost[i][j] = 1000;
            hCost[i][j] = 1000;
            fCost[i][j] = 1000;
            isTraversed[i][j] = false;
        }
    }

    coordinates c = starting->location;
    gCost[c.x][c.y] = 0;
    isTraversed[c.x][c.y] = true;

    //WHILE LOOP STARTS HERE
    while (current != end) {
        //CHECKING SURROUNDING COSTS
        setCosts(current->behindSquare, current, end, gCost, hCost, fCost, false);
        setCosts(current->frontSquare, current, end, gCost, hCost, fCost, false);
        setCosts(current->leftSquare, current, end, gCost, hCost, fCost, false);
        setCosts(current->rightSquare, current, end, gCost, hCost, fCost, false);

//        if (current->behindSquare != NULL) {
//            setCosts(current->behindSquare->leftSquare, current, end, gCost, hCost, fCost, false);
//            setCosts(current->behindSquare->rightSquare, current, end, gCost, hCost, fCost, false);
//        }
//
//        if (current->frontSquare != NULL) {
//            setCosts(current->frontSquare->leftSquare, current, end, gCost, hCost, fCost, false);
//            setCosts(current->frontSquare->rightSquare, current, end, gCost, hCost, fCost, false);
//        }

        //DIAGONAL CHECK
        if (current->frontSquare != NULL ){
            if (current->frontSquare->leftSquare != NULL) {
                bool case1 = current->leftSquare->type == 2 | current->frontSquare->type == 2 | current->leftSquare->isObstacle | current->frontSquare->isObstacle;
                setCosts(current->frontSquare->leftSquare, current, end, gCost, hCost, fCost, case1);
                //fCost[current->location.x-1][current->location.y+1] = 1000;
                //gCost[current->location.x-1][current->location.y+1] = 1000;
            }
            if (current->frontSquare->rightSquare != NULL) {
                bool case2 = current->rightSquare->type == 2 | current->frontSquare->type == 2 | current->rightSquare->isObstacle | current->frontSquare->isObstacle;
                setCosts(current->frontSquare->rightSquare, current, end, gCost, hCost, fCost, case2);
                //fCost[current->location.x+1][current->location.y+1] = 1000;
                //gCost[current->location.x+1][current->location.y+1] = 1000;
            }
        }

        if (current->behindSquare != NULL){
            if (current->behindSquare->leftSquare != NULL) {
                bool case1 = current->leftSquare->type == 2 | current->behindSquare->type == 2 | current->leftSquare->isObstacle | current->behindSquare->isObstacle;
                setCosts(current->behindSquare->leftSquare, current, end, gCost, hCost, fCost, case1);
                //fCost[current->location.x-1][current->location.y-1] = 1000;
                //gCost[current->location.x-1][current->location.y-1] = 1000;
            }
            if (current->behindSquare->rightSquare != NULL) {
                bool case2 = current->rightSquare->type == 2 | current->behindSquare->type == 2 | current->rightSquare->isObstacle | current->behindSquare->isObstacle;
                setCosts(current->behindSquare->rightSquare, current, end, gCost, hCost, fCost, case2);
                //fCost[current->location.x+1][current->location.y-1] = 1000;
                //gCost[current->location.x+1][current->location.y-1] = 1000;
            }
        }

        //FINDING LOWEST F COST AND SETTING IT TO CURRENT NODE
        int lowestCost = 1000;
        int x = 0;
        int y = 0;
        for (int i = 0; i < DIMENSION; i++) {
            for (int j = 0; j < DIMENSION; j++) {
                if (fCost[i][j] < lowestCost && isTraversed[i][j] == false) {
                    x = i;
                    y = j;
                    lowestCost = fCost[i][j];
                } else if (fCost[i][j] == lowestCost && isTraversed[i][j] == false) {
                    if (hCost[i][j] < hCost[x][y]) {
                        x = i;
                        y = j;
                        lowestCost = fCost[i][j];
                    }
                }
            }
        }
        current = &b[x][y];
        isTraversed[x][y] = true;
        //printf("Lowest F Cost: %d (%d,%d)\n", lowestCost, x, y);
    }

    //CREATING THE LIST OF NODES
    path.headNode = (node*) malloc(sizeof(node));
    path.headNode->s = starting;
    path.headNode->parent = NULL;
//    printf("Headnode starting coordinates %d,%d\n", path.headNode->s->location.x,path.headNode->s->location.y);
    Serial.print("Headnode starting coordinates: ");
    Serial.print(path.headNode->s->location.x);
    Serial.print(" ,");
    Serial.println(path.headNode->s->location.y);
    node* currentNode = (node*) malloc(sizeof(node));
    currentNode->s = end;
    currentNode->child = NULL;
    while(currentNode->s->parent != starting){
        currentNode->s->parent = currentNode->s->parent;
        node* tempNode = (node*) malloc(sizeof(node));
        tempNode->s = currentNode->s->parent;
        tempNode->child = currentNode;
        currentNode = tempNode;
        //printf("Coordinates %d,%d\n",currentNode->s->location.x,currentNode->s->location.y);
    }
    currentNode->parent = path.headNode;
    path.headNode->child = currentNode;
    //printf("headnode next %d,%d\n",path.headNode->child->s->location.x,path.headNode->child->s->location.y);
    return path;
}

void predictCosts(tile* current, tile* previous, tile* end, int gCost[][DIMENSION], int hCost[][DIMENSION], int fCost[][DIMENSION]){

    if (current == NULL) return;

    coordinates c = current->location;
    coordinates cP = previous->location;
    hCost[c.x][c.y] = findDistance(c, end->location);

    if (current->type != 2 && !current->isObstacle){
        int g = gCost[cP.x][cP.y] + findDistance(c,cP);
        if (g < gCost[c.x][c.y]){
            current->parent = previous;
            gCost[c.x][c.y] = g;
        }
    }

    else if (current->type != 2 && current->isObstacle && current == end){
        int g = gCost[cP.x][cP.y] + findDistance(c,cP);
        if (g < gCost[c.x][c.y]){
            current->parent = previous;
            gCost[c.x][c.y] = g;
        }
    }
    fCost[c.x][c.y] = gCost[c.x][c.y] + hCost[c.x][c.y];
    return;
}

int findActualDistance(tile b[][DIMENSION], tile* starting, tile* end){
    if(starting == end) return 0;
    tile* current = starting;
    int gCost[DIMENSION][DIMENSION];
    int hCost[DIMENSION][DIMENSION];
    int fCost[DIMENSION][DIMENSION];
    //bool nextToWater[DIMENSION][DIMENSION];
    bool isTraversed[DIMENSION][DIMENSION];

    for(int i = 0; i < DIMENSION; i++){
        for(int j = 0; j < DIMENSION; j++){
            gCost[i][j] = 1000;
            hCost[i][j] = 1000;
            fCost[i][j] = 1000;
            isTraversed[i][j] = false;
        }
    }

    coordinates c = starting->location;
    gCost[c.x][c.y] = 0;
    isTraversed[c.x][c.y] = true;

    //WHILE LOOP STARTS HERE
    while (current != end) {
        //CHECKING SURROUNDING COSTS
        predictCosts(current->behindSquare, current, end, gCost, hCost, fCost);
        predictCosts(current->frontSquare, current, end, gCost, hCost, fCost);
        predictCosts(current->leftSquare, current, end, gCost, hCost, fCost);
        predictCosts(current->rightSquare, current, end, gCost, hCost, fCost);

        //FINDING LOWEST F COST AND SETTING IT TO CURRENT NODE
        int lowestCost = 1000;
        int x = 0;
        int y = 0;
        for (int i = 0; i < DIMENSION; i++) {
            for (int j = 0; j < DIMENSION; j++) {
                if (fCost[i][j] < lowestCost && isTraversed[i][j] == false) {
                    x = i;
                    y = j;
                    lowestCost = fCost[i][j];
                } else if (fCost[i][j] == lowestCost && isTraversed[i][j] == false) {
                    if (hCost[i][j] < hCost[x][y]) {
                        x = i;
                        y = j;
                        lowestCost = fCost[i][j];
                    }
                }
            }
        }
        current = &b[x][y];
        isTraversed[x][y] = true;
    }

    //FINDING THE COST
    tile* currentNode = end;
    int score = 10;
    while(currentNode->parent != starting){
        score += 10;
        currentNode = currentNode->parent;
    }
    return score;
}

//MOVE FORWARD
void moveForward(){
    Serial.print("Moving forward");
    if(orientation == 0) currenty++;
    else if(orientation == 1){
        currenty++;
        currentx++;
    }
    else if (orientation == 2) currentx++;
    else if (orientation == 3){
        currentx++;
        currenty--;
    }
    else if (orientation == 4) currenty--;
    else if (orientation == 5){
        currenty--;
        currentx--;
    }
    else if (orientation == 6) currentx--;
    else if (orientation == 7){
        currentx--;
        currenty++;
    }
}

//MOVE BACKWARDS
void moveBackward(){
    Serial.print("Moving backward");
}

//ROTATE RIGHT 45 DEGREES
void rr45(){
    orientation = (orientation+1)%8;
    Serial.print("Rotating right 45 degrees ");
}

//ROTATE LEFT 45 DEGREES
void rl45(){
    orientation = (orientation+7)%8;
    Serial.print("Rotating left 45 degrees ");
}

//ROTATE RIGHT 90 DEGREES
void rr90(){
    orientation = (orientation+2)%8;
    Serial.print("Rotating right 90 degrees ");
}

//ROTATE LEFT 90 DEGREES
void rl90(){
    orientation = (orientation+6)%8;
    Serial.print("Rotating left 90 degrees ");
}

//ROTATE RIGHT 135 DEGREES
void rr135(){
    orientation = (orientation+3)%8;
    Serial.print("Rotating right 135 degrees ");
}

//ROTATE LEFT 135 DEGREES
void rl135(){
    orientation = (orientation+5)%8;
    Serial.print("Rotating left 135 degrees ");
}

void r180(){
    orientation = (orientation+4)%8;
    Serial.print("Rotating 180 degrees ");
}

void probe(){
    Serial.println("Inspecting closer...\n");
    return;
}

void unprobe(){
    return;
}

//RETURNS DIRECTION OF NEXT SQUARE
int findDirection(coordinates c1, coordinates c2){
    int dx = c2.x - c1.x;
    int dy = c2.y - c1.y;

    if (dx == 1){
        if (dy == 1) return 1;
        else if (dy == 0) return 2;
        else if (dy == -1) return 3;
    }

    else if (dx == 0){
        if (dy == 1) return 0;
        else if (dy == -1) return 4;
    }

    else if (dx == -1){
        if (dy == 1) return 7;
        else if (dy == 0) return 6;
        else if (dy == -1) return 5;
    }
    else if (dx == 0 & dy == 0) return 8;
}

//TURN LIST VALUES INTO MOVEMENT COMMANDS
void decodeList(list path){
    node* temp = path.headNode;
    if(temp == NULL) return;
//    printf("Starting at: (%d,%d)\n", temp->s->location.x,temp->s->location.y);
    Serial.print("Starting at: ");
    Serial.print(temp->s->location.x);
    Serial.print(" ,");
    Serial.println(temp->s->location.y);
    while(temp->child!= NULL && !temp->child->s->isObstacle){
        int d = findDirection(temp->s->location,temp->child->s->location);
        //printf("Direction: %d ", d);
        //printf("Orientation: %d ", orientation);
//        printf("Next Point:(%d,%d) ", temp->child->s->location.x,temp->child->s->location.y);
        Serial.print("Next Point: ");
        Serial.print(temp->child->s->location.x);
        Serial.print(" ,");
        Serial.println(temp->child->s->location.y);        

        if((d - orientation + 8)%8 == 1){
            rr45();
            moveForward();
        }

        else if((d - orientation + 8)%8 == 2){
            rr90();
            moveForward();
        }

        else if((d - orientation + 8)%8 == 3){
            rr135();
            moveForward();
        }

        else if((d - orientation + 8)%8 == 4){
            r180();
            moveForward();
        }

        else if((d - orientation + 8)%8 == 5){
            rl135();
            moveForward();
        }

        else if((d - orientation + 8)%8 == 6){
            rl90();
            moveForward();
        }

        else if((d - orientation + 8)%8 == 7){
            rl45();
            moveForward();
        }

        else if((d - orientation + 8)%8 == 0){
            moveForward();
        }
//        printf(" to point (%d,%d)\n", currentx, currenty);
        Serial.print(" to point : ");
        Serial.print(currentx);
        Serial.print(" ,");
        Serial.println(currenty);  
        
        node* temp2 = temp->child;
        free(temp);
        temp = temp2;
    }
    if(temp != NULL) free(temp);
    //free(path.headNode);
    //printf("Destination Reached\n");
    return;
}

//REVEAL A SQUARE AND ITS TERRAIN TYPE
void reveal(tile* b, int terrain){
    b->isRevealed = true;
    b->type = terrain;
}

// TODO: Fix this (signal is already used)
void signal(){
    Serial.println("ITEM FOUND\n");
}

bool foodDetected(tile* current){
    if(current == mockFood) {
        Serial.println("Found food\n");
        food = current;
        foodFound = true;
        return true;
    }
    else return false;
}

tile* relativePointer(tile* current, int relativeDir){
    if (relativeDir == 0){
        if (orientation == 0) return current->frontSquare;
        if (orientation == 2) return current->rightSquare;
        if (orientation == 4) return current->behindSquare;
        if (orientation == 6) return current->leftSquare;
    }
    else if (relativeDir == 2){
        if (orientation == 0) return current->rightSquare;
        if (orientation == 2) return current->behindSquare;
        if (orientation == 4) return current->leftSquare;
        if (orientation == 6) return current->frontSquare;
    }
    else if (relativeDir == 4){ //behind robot
        if (orientation == 0) return current->behindSquare;
        if (orientation == 2) return current->leftSquare;
        if (orientation == 4) return current->frontSquare;
        if (orientation == 6) return current->rightSquare;
    }
    else if (relativeDir == 6){ //left  of robot
        if (orientation == 0) return current->leftSquare;
        if (orientation == 2) return current->frontSquare;
        if (orientation == 4) return current->rightSquare;
        if (orientation == 6) return current->behindSquare;
    }
}

bool fireDetected(tile* current, tile board[][DIMENSION]){
    if (mockFire == relativePointer(current, 0)) {
        fireFound = true;
//        printf("Fire detected at (%d,%d)\n",relativePointer(current, 0)->location.x, relativePointer(current, 0)->location.y);
        Serial.print("Fire detected at : ");
        Serial.print(relativePointer(current, 0)->location.x);
        Serial.print(" ,");
        Serial.println(relativePointer(current, 0)->location.y);
        return true;
    }
    else return false;
}

void groupDetected(tile* current, tile board[][DIMENSION]) {

    if (mockGroup == relativePointer(current, 0)) {
        groupFound = true;
//        printf("Group detected at (%d,%d)\n", relativePointer(current, 0)->location.x,
//               relativePointer(current, 0)->location.y);
        Serial.print("Group detected at : ");
        Serial.print(relativePointer(current, 0)->location.x);
        Serial.print(" ,");
        Serial.println(relativePointer(current, 0)->location.y);
        group = relativePointer(current, 0);
        return;
    }
    return;
}

void survivorDetected(tile* current, tile board[][DIMENSION]) {

    if (mockSurvivor == relativePointer(current, 0)) {
        survivorFound = true;
//        printf("Survivor detected at (%d,%d)\n", relativePointer(current, 0)->location.x,
//               relativePointer(current, 0)->location.y);
        Serial.print("Survivor detected at : ");
        Serial.print(relativePointer(current, 0)->location.x);
        Serial.print(" ,");
        Serial.println(relativePointer(current, 0)->location.y);
        survivor = relativePointer(current, 0);
        return;
    }
    return;
}

void getObstacles(bool obstacles[], tile* current){
    for (int i = 0; i < 4; i++){
        if (!obstacles[i]){
            obstacles[i] = (mockFire != NULL && relativePointer(current, 2*i) == mockFire);
            obstacles[i] = obstacles[i] || (mockSurvivor != NULL && relativePointer(current, 2*i) == mockSurvivor);
            obstacles[i] = obstacles[i] || (mockGroup != NULL && relativePointer(current, 2*i) == mockGroup);
        }
    }
    return;
}

void getTerrain(int *terrain, tile* current){
    for (int i = 0; i < 4; i++){
        if (mockWater1 != NULL && relativePointer(current, 2*i) == mockWater1) terrain[i] = 2;
        else if (mockWater2 != NULL && relativePointer(current, 2*i) == mockWater2) terrain[i] = 2;
        else if (mockWater3 != NULL && relativePointer(current, 2*i) == mockWater3) terrain[i] = 2;
        else if (mockWater4 != NULL && relativePointer(current, 2*i) == mockWater4) terrain[i] = 2;
        else if (mockSand1 != NULL && relativePointer(current, 2*i) == mockSand1) terrain[i] = 1;
        else if (mockSand2 != NULL && relativePointer(current, 2*i) == mockSand2) terrain[i] = 1;
        else if (mockSand3 != NULL && relativePointer(current, 2*i) == mockSand3) terrain[i] = 1;
        else if (mockSand4 != NULL && relativePointer(current, 2*i) == mockSand4) terrain[i] = 1;
        else terrain[i] = 0;
    }
    return;
}

void scanArea(tile* current, tile board[][DIMENSION]){
    //Obtaining Terrain Types
    int terrain[4] = {4,4,4,4}; // 0 - front sensor | 1 - right | 2 - behind | 3 - left
    getTerrain(terrain, current);
    for (int i = 0; i < 4; i++) {
        if (relativePointer(current, 2 * i) != NULL) reveal(relativePointer(current, 2 * i), terrain[i]);
    }

    //Obtaining Object Locations
    bool obstacles[4] = {0,0,0,0}; // 0 - front sensor | 1 - right | 2 - behind | 3 - left
    getObstacles(obstacles, current);

    //Investigating obstacles
    if(obstacles[0] && relativePointer(current, 0) != NULL && !relativePointer(current, 0)->isObstacle) {
        relativePointer(current, 0)->isObstacle = true;
        probe();
        fireDetected(current, board);
        groupDetected(current, board);
        survivorDetected(current, board);
        
    }

    if(obstacles[1] && relativePointer(current, 2) != NULL && !relativePointer(current, 2)->isObstacle){
        relativePointer(current, 2)->isObstacle = true;
        rr90();
        probe();
        fireDetected(current, board);
        groupDetected(current, board);
        survivorDetected(current, board);
        unprobe();
    }

    if(obstacles[2] && relativePointer(current, 4) != NULL && !relativePointer(current, 4)->isObstacle){
        relativePointer(current, 4)->isObstacle = true;
        r180();
        probe();
        fireDetected(current, board);
        groupDetected(current, board);
        survivorDetected(current, board);
        unprobe();
    }

    if(obstacles[3] && relativePointer(current, 6) != NULL && !relativePointer(current, 6)->isObstacle){
        relativePointer(current, 6)->isObstacle = true;
        rl90();
        probe();
        fireDetected(current, board);
        groupDetected(current, board);
        survivorDetected(current, board);
    }

    //DETECTING FOR FOOD
    if(foodDetected(current)){
        foodFound = true;
        food = current;
        //printf("\nFood Found: %d\n", foodFound);
    }

}

void moveTowards(tile* start, tile* end, bool isVisited[][DIMENSION], tile board[][DIMENSION]){

    tile* current = start;
    int distance[4] = {1000, 1000, 1000, 1000};
    bool visited = false;
    //printf("Type: %d Obstacle: %d\n", current->frontSquare->type != 2, current->frontSquare->isObstacle);
    if (current->frontSquare != NULL && current->frontSquare->type != 2 && !current->frontSquare->isObstacle){
        visited = isVisited[current->frontSquare->location.x][current->frontSquare->location.y];
        distance[0] = (1 + visited + !(current->frontSquare->type == 1 && !visited) + (current->frontSquare == relativePointer(current, 4)))*findDistance(current->frontSquare->location, end->location);
        //printf("front dist: %d Distance to goal:%d Is visited?: %d \n", distance[0],findDistance(current->frontSquare->location, end->location),visited);
    }
    if (current->rightSquare != NULL && current->rightSquare->type != 2 && !current->rightSquare->isObstacle){
        visited = isVisited[current->rightSquare->location.x][current->rightSquare->location.y];
        distance[1] = (1 + visited + !(current->rightSquare->type == 1 && !visited)+ (current->rightSquare == relativePointer(current, 4)))*findDistance(current->rightSquare->location, end->location);
        //printf("right dist: %d Distance to goal:%d Is visited?: %d \n", distance[1],findDistance(current->rightSquare->location, end->location),visited);
    }

    if (current->behindSquare != NULL && current->behindSquare->type != 2 && !current->behindSquare->isObstacle){
        visited = isVisited[current->behindSquare->location.x][current->behindSquare->location.y];
        distance[2] = (1 + visited + !(current->behindSquare->type == 1 && !visited)+ (current->behindSquare == relativePointer(current, 4)))*findDistance(current->behindSquare->location, end->location);
        //printf("behind dist: %d Distance to goal:%d Is visited?: %d \n", distance[2],findDistance(current->behindSquare->location, end->location),visited);
    }

    if (current->leftSquare != NULL && current->leftSquare->type != 2 && !current->leftSquare->isObstacle){
        visited = isVisited[current->leftSquare->location.x][current->leftSquare->location.y];
        distance[3] = (1 + visited + !(current->leftSquare->type == 1 && !visited)+ (current->leftSquare == relativePointer(current, 4)))*findDistance(current->leftSquare->location, end->location);
        //printf("left dist: %d Distance to goal:%d Is visited?: %d\n", distance[3],findDistance(current->leftSquare->location, end->location),visited);
    }


    int closest = 1000;
    int d = 0;
    for (int i = 0; i < 4; i++) {
        if (distance[i] < closest) {
            closest = distance[i];
            d = 2 * i;
        }

        else if(distance[i] == closest){
            if ((2*i - orientation + 8)%8 == 0){
                closest = distance[i];
                d = 2 * i;
            }
        }
    }

    if ((d - orientation + 8) % 8 == 2) {
        rr90();
        moveForward();
    }
    else if ((d - orientation + 8) % 8 == 4) {
        r180();
        moveForward();
    }
    else if ((d - orientation + 8) % 8 == 6) {
        rl90();
        moveForward();
    }
    else if ((d - orientation + 8) % 8 == 0) {
        moveForward();
    }
    else if ((d - orientation + 8) % 8 == 1) {
        rr45();
        moveForward();
    }
    else if ((d - orientation + 8) % 8 == 3) {
        rr135();
        moveForward();
    }
    else if ((d - orientation + 8) % 8 == 5) {
        rl135();
        moveForward();
    }
    else if ((d - orientation + 8) % 8 == 7) {
        rl45();
        moveForward();
    }
    isVisited[currentx][currenty] = true;
    current = &board[currentx][currenty];
    //printf(" to coordinates (%d,%d)\n", currentx, currenty);
    Serial.print(" to coordinates : ");
    Serial.print(currentx);
    Serial.print(" ,");
    Serial.println(currenty);
    scanArea(current, board);
}

tile* findTarget(tile* current, tile board[][DIMENSION], bool isVisited[][DIMENSION]){
    double maxRatio = 0;
    tile* currentBest = NULL;
    for(int i = 0; i < DIMENSION; i++){
        for(int j = 0; j < DIMENSION; j++){
            int tempSurr = 0;
            tile* tempSquare = &board[i][j];
            if(tempSquare->type != 2 && !tempSquare->isObstacle && !(currentx == i && currenty == j)) {
                if (tempSquare->type == 1 && !isVisited[i][j] && !foodFound) tempSurr = 2;
                if (!isVisited[i][j]) tempSurr++;//ADDED
                if (tempSquare->frontSquare != NULL && !tempSquare->frontSquare->isRevealed) tempSurr++;
                if (tempSquare->behindSquare != NULL && !tempSquare->behindSquare->isRevealed) tempSurr++;
                if (tempSquare->rightSquare != NULL && !tempSquare->rightSquare->isRevealed) tempSurr++;
                if (tempSquare->rightSquare != NULL) {
                    if (tempSquare->rightSquare->frontSquare != NULL &&
                        !tempSquare->rightSquare->frontSquare->isRevealed)
                        tempSurr++;
                    if (tempSquare->rightSquare->behindSquare != NULL &&
                        !tempSquare->rightSquare->behindSquare->isRevealed)
                        tempSurr++;
                }
                if (tempSquare->leftSquare != NULL && !tempSquare->leftSquare->isRevealed) tempSurr++;
                if (tempSquare->leftSquare != NULL){
                    if (tempSquare->leftSquare->frontSquare != NULL &&
                        !tempSquare->leftSquare->frontSquare->isRevealed)
                        tempSurr++;
                    if (tempSquare->leftSquare->behindSquare != NULL &&
                        !tempSquare->leftSquare->behindSquare->isRevealed)
                        tempSurr++;
                }
                double dist = findActualDistance(board, current, tempSquare);
                double temp = 100 * tempSurr / dist;
                //printf("Ratio: %f, Surr: %d, Dist: %f, Coord: (%d,%d)\n", 100*tempSurr/dist, tempSurr, dist, i,j);
                if (100 * tempSurr / dist == 50) return &board[i][j];
                else if (100 * tempSurr / dist > maxRatio) {
                    maxRatio = 100 * tempSurr / dist;
                    currentBest = &board[i][j];
                }
            }
        }
    }
    Serial.print("Trying to go to: ");
    Serial.print(currentBest->location.x);
    Serial.print(",");
    Serial.println(currentBest->location.y);
    return currentBest;
}

void roam(tile board[][DIMENSION]){
   bool isVisited[DIMENSION][DIMENSION]; 
    for (int i = 0; i < DIMENSION; i++){
        for (int j = 0; j < DIMENSION; j++){
            isVisited[i][j] = false;
        }
    }
  isVisited[currentx][currenty] = true;
  reveal(&board[currentx][currenty],1);
  scanArea(&board[currentx][currenty], board);
  tile* target;
  target = findTarget(&board[currentx][currenty], board, isVisited);
  while(!fireFound){
    while (!fireFound && &board[currentx][currenty] != target && !target->isObstacle && target->type != 2){
        //printf("Target: (%d,%d)\n", target->location.x, target->location.y);
        moveTowards(&board[currentx][currenty], target, isVisited, board);
        if(currentx == 1 && currenty == 1 && !board[0][0].isRevealed){
            moveTowards(&board[currentx][currenty], &board[0][0], isVisited, board);
            if(board[0][0].type == 2) moveTowards(&board[currentx][currenty], &board[0][0], isVisited, board);
        }
        else if(currentx == 1 && currenty == 4 && !board[0][5].isRevealed){
            moveTowards(&board[currentx][currenty], &board[0][5], isVisited, board);
            if(board[0][5].type == 2) moveTowards(&board[currentx][currenty], &board[0][5], isVisited, board);
        }
        else if(currentx == 4 && currenty == 4 && !board[5][5].isRevealed){
            moveTowards(&board[currentx][currenty], &board[5][5], isVisited, board);
            if(board[5][5].type == 2) moveTowards(&board[currentx][currenty], &board[5][5], isVisited, board);
        }
        else if(currentx == 4 && currenty == 1 && !board[5][0].isRevealed){
            moveTowards(&board[currentx][currenty], &board[5][0], isVisited, board);
            if(board[5][0].type == 2) moveTowards(&board[currentx][currenty], &board[5][0], isVisited, board);
        }
        target = findTarget(&board[currentx][currenty], board, isVisited);
    }
  }
    signal();
    bool foodSignal = false;
    bool survSignal = false;
    bool groupSignal = false;

    if (foodFound && !survivorFound) {
        Serial.println("Now going to food...");
        decodeList(findPath(&board[currentx][currenty], food, board));
        signal();
        foodSignal = true;
    }
    else if(survivorFound && !foodFound) {
        Serial.println("Now going to survivor...\n");
        decodeList(findPath(&board[currentx][currenty], survivor, board));
        signal();
        survSignal = true;
    }
    if(survivorFound && foodFound){
        int survivorDist = findDistance(board[currentx][currenty].location, survivor->location);
        int foodDist = findDistance(board[currentx][currenty].location, food->location);
        if(survivorDist < foodDist){
            Serial.println("Food and survivor already found, going to survivor first\n");
            decodeList(findPath(&board[currentx][currenty], survivor, board));
            signal();
            Serial.println("Now going to food\n");
            decodeList(findPath(&board[currentx][currenty], food, board));
            signal();
        }
        else{
            Serial.println("Food and survivor already found, going to food first\n");
            decodeList(findPath(&board[currentx][currenty], food, board));
            signal();
            Serial.println("Now going to survivor\n");
            decodeList(findPath(&board[currentx][currenty], survivor, board));
            signal();
        }
        foodSignal = true;
        survSignal = true;
    }

    //TRYING TO FIND THE FOOD/SURVIVOR IF NOT PREVIOUSLY FOUND
    target = findTarget(&board[currentx][currenty], board, isVisited);
    if(!foodFound) Serial.println("Looking for food and survivor\n");
    while(!foodFound || !survivorFound){
        //if (target != NULL) printf("Target: (%d,%d)\n", target->location.x, target->location.y);
        moveTowards(&board[currentx][currenty], target, isVisited, board);

        //SIGNAL IF FIRE/SURVIVOR FOUND
        if(foodFound && !foodSignal) {
            signal();
            foodSignal = true;
        }
        if(survivorFound && !survSignal) {
            signal();
            survSignal = true;
        }
                
        if(groupFound && !groupSignal && foodFound) {
            signal();
            groupSignal = true;
        }

        //CHECK CORNERS FOR SAND PITS (PRIMARY CASE)
        if(currentx == 1 && currenty == 1 && !board[0][0].isRevealed){
            moveTowards(&board[currentx][currenty], &board[0][0], isVisited, board);
            if(board[0][0].type == 1) moveTowards(&board[currentx][currenty], &board[0][0], isVisited, board);
        }
        else if(currentx == 1 && currenty == 4 && !board[0][5].isRevealed){
            moveTowards(&board[currentx][currenty], &board[0][5], isVisited, board);
            if(board[0][5].type == 1)moveTowards(&board[currentx][currenty], &board[0][5], isVisited, board);
        }
        else if(currentx == 4 && currenty == 4 && !board[5][5].isRevealed){
            moveTowards(&board[currentx][currenty], &board[5][5], isVisited, board);
            if(board[5][5].type == 1) moveTowards(&board[currentx][currenty], &board[5][5], isVisited, board);
        }
        else if(currentx == 4 && currenty == 1 && !board[5][0].isRevealed){
            moveTowards(&board[currentx][currenty], &board[5][0], isVisited, board);
            if(board[5][0].type == 1) moveTowards(&board[currentx][currenty], &board[5][0], isVisited, board);
        }
        target = findTarget(&board[currentx][currenty], board, isVisited);
    }

    if (foodFound && groupFound) {
        Serial.println("Food already found, now going to group...\n");
        decodeList(findPath(&board[currentx][currenty], group, board));
        signal();
    }
    while(!survivorFound | !groupFound){
        //if (target != NULL) printf("Target: (%d,%d)\n", target->location.x, target->location.y);
        moveTowards(&board[currentx][currenty], target, isVisited, board);
        if(currentx == 1 && currenty == 1 && !board[0][0].isRevealed){
            moveTowards(&board[currentx][currenty], &board[0][0], isVisited, board);
            if(board[0][0].type == 1) moveTowards(&board[currentx][currenty], &board[0][0], isVisited, board);
        }
        else if(currentx == 1 && currenty == 4 && !board[0][5].isRevealed){
            moveTowards(&board[currentx][currenty], &board[0][5], isVisited, board);
            if(board[0][5].type == 1)moveTowards(&board[currentx][currenty], &board[0][5], isVisited, board);
        }
        else if(currentx == 4 && currenty == 4 && !board[5][5].isRevealed){
            moveTowards(&board[currentx][currenty], &board[5][5], isVisited, board);
            if(board[5][5].type == 1) moveTowards(&board[currentx][currenty], &board[5][5], isVisited, board);
        }
        else if(currentx == 4 && currenty == 1 && !board[5][0].isRevealed){
            moveTowards(&board[currentx][currenty], &board[5][0], isVisited, board);
            if(board[5][0].type == 1) moveTowards(&board[currentx][currenty], &board[5][0], isVisited, board);
        }
        if(survivorFound && !survSignal) {
            signal();
            survSignal = true;
        }
        if(groupFound && !groupSignal) {
            signal();
            groupSignal = true;
        }
        target = findTarget(&board[currentx][currenty], board, isVisited);
    }
    decodeList(findPath(&board[currentx][currenty], &board[STARTINGX][STARTINGY], board)); //GO HOME
    Serial.print("DONE");
}
////////////////////////////////////////////

/////////////////////////////////////////////
///////////// SENSOR FUNCTIONS  /////////////
/////////////////////////////////////////////

//L/R/B ULTRASONIC FUNCTIONS
void SonarSensor(int sensor_num, int trigPinSensor,int echoPinSensor, int values[]) {
  pinMode(trigPinSensor, OUTPUT);
  pinMode(echoPinSensor, INPUT);   
 
  digitalWrite(trigPinSensor, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinSensor, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinSensor, LOW);

  US_duration = pulseIn(echoPinSensor, HIGH);
  US_distance= (US_duration/2) / 29.1;  

  values[US_counter] = US_distance;
  US = 0;
  for (int i=0; i<US_NumOfValues; i++) {
    US = US+values[i];
  }
  US = (US)/US_NumOfValues;
  delay(50); 

  if (US <= 30) {
    sensors_detected[sensor_num] = true;
  } else {
    sensors_detected[sensor_num] = false;
  }

}

bool detect_objects() {
  //Serial.print("Sensor FL: ");
  SonarSensor(0, trigPin,echoPin0,  US0_values);  //Front_Left
  
  //Serial.print("Sensor FR: ");
  SonarSensor(1, trigPin,echoPin1,  US1_values);  //Front_Right
  
  //Serial.print("Sensor RF: ");
  SonarSensor(2, trigPin,echoPin2,  US2_values);  //Right Front

  //Serial.print("Sensor BR: ");
  SonarSensor(6, trigPin,echoPin6,  US6_values);  //Right Back
   
  //Serial.print("Sensor LF: ");
  SonarSensor(4, trigPin,echoPin4,  US4_values);  //Left Front
  
  //Serial.print("Sensor LB: ");
  SonarSensor(5, trigPin,echoPin5,  US5_values);  //Left Back
  
  // Serial.print("Sensor RB: ");
  SonarSensor(3, trigPin,echoPin3,  US3_values);  //Back Right
  
//  //Serial.print("Sensor BL: ");
//  SonarSensor(7, trigPin,echoPin7,  US7_values);  //Back Left

  if (sensors_detected[0] == true || sensors_detected[1]== true) {
    object_location[0] = true;
    Serial.print("Front: Detected     ");

  } else {
    object_location[0] = false;
    Serial.print("Front: Nothing      ");
  }
  if (sensors_detected[2] == true || sensors_detected[6]== true) {
    object_location[1] = true;
    Serial.print("Right: Detected     ");

  } else {
    object_location[1] = false;
    Serial.print("Right: Nothing      ");
  }

  if (sensors_detected[4] == true || sensors_detected[5]== true) {
    object_location[2] = true;
    Serial.print("Left: Detected     ");

  } else {
    object_location[2] = false;
    Serial.print("Left: Nothing       ");
  }

  if (sensors_detected[3] == true) {
    object_location[3] = true;
    Serial.print("Back: Detected     ");
  } else {
    object_location[3] = false;
    Serial.print("Back: Nothing     ");
  }

  
  
  US_counter = (US_counter + 1)%US_NumOfValues;
  //Serial.print(US);
  Serial.println("   ");

  return object_location;
}

/*
 * Returns in mm
 * 
 * Pins:
 * FL - 9
 * FR - 13
 */
int sonar_dist(int echoPinSensor)
{
  int trigPinSensor = 8;
  long single_duration = 0;
  long single_distance = 0;
  
  pinMode(trigPinSensor, OUTPUT);
  pinMode(echoPinSensor, INPUT);   
 
  digitalWrite(trigPinSensor, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinSensor, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinSensor, LOW);

  single_duration = pulseIn(echoPinSensor, HIGH);
  single_distance = (single_duration/2) / 29.1 * 10;  

  return single_distance;
}

// HALL EFFECT
bool Mag_detect() {

  float TOMILLIGAUSS = 3.756010;
  //int pin_array[9] = {HE0, HE1, HE2, HE3, HE4, HE5, HE6, HE7, HE8};
  float gauss[9] = {0};
  int mag = 0;
  int no_mag = 0;
  int threshold = 7;

  for (int i = 0; i < 15; i++ ) {
    delay(100);
    gauss[0] = (analogRead(HE0) - 506) * TOMILLIGAUSS;
    gauss[1] = (analogRead(HE1) - 510) * TOMILLIGAUSS;
    gauss[2] = (analogRead(HE2) - 508) * TOMILLIGAUSS;
    gauss[3] = (analogRead(HE3) - 509) * TOMILLIGAUSS;
    gauss[4] = (analogRead(HE4) - 508) * TOMILLIGAUSS;
    gauss[5] = (analogRead(HE5) - 511) * TOMILLIGAUSS;
    gauss[6] = (analogRead(HE6) - 506) * TOMILLIGAUSS;
    gauss[7] = (analogRead(HE7) - 510) * TOMILLIGAUSS;
    gauss[8] = (analogRead(HE8) - 511) * TOMILLIGAUSS;
//    Serial.print(gauss[0]  );
//    Serial.print( "  " );
//     Serial.print(gauss[1]);
//     Serial.print( "  " );
//      Serial.print(gauss[2]);
//      Serial.print( "  " );
//       Serial.print(gauss[3]);
//       Serial.print( "  " );
//        Serial.print(gauss[4]);
//        Serial.print( "  " );
//         Serial.print(gauss[5]);
//         Serial.print( "  " );
//          Serial.print(gauss[6]);
//          Serial.print( "  " );
//           Serial.print(gauss[7]);
//           Serial.print( "  " );
//            Serial.println(gauss[8]);
   
  

    if ( (gauss[0] < -7) || (gauss[1] < -7) || (gauss[2] < -7) || (gauss[3] < -7) || (gauss[4] < -7) || (gauss[5] < -7) || (gauss[6] < -7) || (gauss[7] < -7) || (gauss[8] < -7)) {
      mag++;
    }
    else if ( (gauss[0] > 7) || (gauss[1] > 7) || (gauss[2] > 7) || (gauss[3] > 7) || (gauss[4] > 7) || (gauss[5] > 7) || (gauss[6] > 7) || (gauss[7] > 7) || (gauss[8] > 7)) {
      mag++;
       
    }
    else {
      no_mag++;
    }
//      Serial.print(mag);
//      Serial.println(no_mag);
  }
  if (mag > 4)
    return 1;
  else if (no_mag >= 10)
    return 0;
}

// COLOUR SENSOR RECEIVER // 0 = survior, 1 = house, 2 = candle
void init_colour_sensor() {
  pinMode(Colour_REQUEST, OUTPUT);
  pinMode(Colour_SENT, INPUT);
}

int requestColour(){
  int colour = 0;
  digitalWrite(Colour_REQUEST, HIGH);  //I want data

  while(!digitalRead(Colour_SENT)){ // while I haven't received anything
    while(!Serial.available());
    colour = Serial.read();
  }
  
  digitalWrite(Colour_REQUEST, LOW);
  return (colour);
}


// FLAME SENSOR
void init_flame_sensor() {
  pinMode(FLAME1, INPUT);
  pinMode(FLAME2, INPUT);
  pinMode(FLAME3, INPUT);
  pinMode(FLAME4, INPUT);
  pinMode(FAN, OUTPUT);
}
bool Detect_Flame() {
  int fire1 = digitalRead(FLAME1);
  int fire2 = digitalRead(FLAME2);
  int fire3 = digitalRead(FLAME3);
  int fire4 = digitalRead(FLAME4);

  if (fire1 == LOW) {
    Serial.print("Fire1    ");
  }
  if (fire2 == LOW) {
    Serial.print("Fire2    ");
  }
  if (fire3 == LOW) {
    Serial.print("Fire3    ");
  }
  if (fire4 == LOW) {
    Serial.print("Fire4    ");
  }

  bool fire_detected = false;
  
  if(fire1 == LOW || fire2 == LOW || fire3 == LOW ||  fire4 == LOW) {
    digitalWrite(LED_BUILTIN, HIGH);
    fan_control(1);

    Serial.println("Fire");
    return true;
    //delay(200); // ??? IS THIS DELAY REQUIRED?
  }else{
    Serial.println("Nothing");
    digitalWrite(LED_BUILTIN, LOW);
    fan_control(0);
    return false;
  }
  delay(200);
}

// TURN FAN ON FUNCTION   // 1 = ON, 0 = OFF
    
void fan_control(bool mode) {
  if (mode == 1) {
    digitalWrite(FAN, HIGH);
    delay(5000);
  } else if (mode == 0) {
    digitalWrite(FAN, LOW);
  }
}

// TOF SENSOR
void init_tof()
{ 
  pinMode(TOF_REQUEST, OUTPUT);
  pinMode(TOF_SENT, INPUT);
  
  if (tof_distance.begin() == false)
  {
    Serial.println("Sensor failed to initialize. Check wiring.");
    while (1); //Freeze!
  }
}

int getDistance_tof()
{
  tof_distance.takeMeasurement(); //Tell sensor to take measurement
  return tof_distance.getDistance();
}

int requestTOF(){
  int last3 = 0;
  int first3 = 0;
  digitalWrite(TOF_REQUEST, HIGH);  //I want data
  while(!digitalRead(TOF_SENT)){ // while I haven't received anything
    while(!Serial.available());
    last3 = Serial.read();
    while(!Serial.available());
    first3 = Serial.read();
  }
  digitalWrite(TOF_REQUEST, LOW);
  return (last3 + (first3 << 8));
}

// MOTOR FUNCTIONS

void init_IMU()
{
    /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void init_motor()
{
  // Motor Setup
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT); 
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);
  pinMode(en1, OUTPUT); // RL
  pinMode(en2, OUTPUT); // FL
  pinMode(en3, OUTPUT); // FR
  pinMode(en4, OUTPUT); // RR

  // Initial direction (forwards)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
}

void stopMotors()
{
  analogWrite(en4, 0);
  analogWrite(en3, 0);
  analogWrite(en2, 0);
  analogWrite(en1, 0);
}

void setForwards()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
}

void setBackwards()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
}

void setClockwise()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
}

void setCounterClockwise()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
}

void setMotorPWM(int pwm_r, int pwm_l)
{
  float fl_fac = 1;
  float rl_fac = -1.6094*pow(10, -7)*pow(pwm_l, 3) + 0.00012148*pow(pwm_l, 2) - 0.026964*pwm_l + 2.5528;
  float fr_fac = -1.3899*pow(10, -7)*pow(pwm_r, 3) + 0.00010587*pow(pwm_r, 2) - 0.023731*pwm_r + 2.3867;
  float rr_fac = -3.3351*pow(10, -7)*pow(pwm_r, 3) + 0.00023117*pow(pwm_r, 2) - 0.051059*pwm_r + 4.4313;

  float pwm_rl = rl_fac * pwm_l;
  float pwm_fl = fl_fac * pwm_l;
  float pwm_fr = fr_fac * pwm_r;
  float pwm_rr = rr_fac * pwm_r;

  pwm_rl = constrain(pwm_rl, -255, 255);
  pwm_fl = constrain(pwm_fl, -255, 255);
  pwm_fr = constrain(pwm_fr, -255, 255);
  pwm_rr = constrain(pwm_rr, -255, 255);

//  Serial.print("RL: ");
//  Serial.print(pwm_rl);
//  Serial.print("  FL: ");
//  Serial.print(pwm_fl);
//  Serial.print("  FR: ");
//  Serial.print(pwm_fr);
//  Serial.print("  RR: ");
//  Serial.println(pwm_rr);
  
  analogWrite(en1,pwm_rl);
  analogWrite(en2,pwm_fl);
  analogWrite(en3,pwm_fr);
  analogWrite(en4,pwm_rr);
}

/*
 *  bool dir 0 goes forwards, 1 goes backwards
 */
void goStraight(int dir)
{
  int delayTime_millis = 50;
  float curr_angle = 0;
  float prev_angle_x = 0;
  float error = 0, errorSum = 0;
  float sampleTime = 0.05;
  unsigned long currTime = 0, prevTime = 0;

  int samples = 5;
  int sonar_counter = 0;
  int sonar_array[samples] = {0};
  int sum = 0;
  int curr_dist = 0;
  float avg_dist = 0;
  bool limit_reached = false;
  float std = 0;
  float true_dist = 0;
  int init_distance = 0;
  int init_time = 0;
  int temp = 0;
  float diff = 0;
  float diffSum = 0;
  int diffCounter = 0;
  float distanceTravelled = 0;
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);


  int pwm_r = 250;
  int pwm_l = 250;


  if (dir == 0)
  {
    setForwards();
  }
  else if (dir == 1)
  {
    setBackwards();
  }

  // Get init distance
  while (init_distance == 0)
  {
    curr_dist = sonar_dist(9);
    //curr_dist = getDistance_tof();
    
    if (curr_dist > 0 && curr_dist < 2000)
    {
      sum -= sonar_array[sonar_counter];
      sum += curr_dist;
      sonar_array[sonar_counter] = curr_dist;
      sonar_counter = (sonar_counter + 1)%samples;

      if (limit_reached)
      {
        avg_dist = (float)sum/samples;

        std = 0;
        for (int i = 0; i < samples; i++)
        {
          std += pow((sonar_array[i] - avg_dist), 2);
        }
        std = sqrt(std/(samples - 1));

        if (std/avg_dist < 0.033)
        {
          init_distance = avg_dist;
          true_dist = avg_dist;
        }
      }

      if (sonar_counter == samples - 1) limit_reached = true;
  
    } 
  }

  init_time = millis();

  do
  {
    currTime = millis();
    sampleTime = (currTime - prevTime)/1000.0;
    prevTime = currTime;
    diffCounter++;

    curr_dist = sonar_dist(9); //raw value
    //curr_dist = getDistance_tof();
    
    if (curr_dist > 0 && curr_dist < 2000)
    {
      sum -= sonar_array[sonar_counter];
      sum += curr_dist;
      sonar_array[sonar_counter] = curr_dist;

      avg_dist = (float)sum/samples; //column B
      std = 0;
      for (int i = 0; i < samples; i++)
      {
        std += pow((sonar_array[i] - avg_dist), 2);
      }
      std = sqrt(std/(samples - 1));

      if (std/avg_dist < 0.05) //need to modify
      {
        //updating true average value
        true_dist = avg_dist;
        
        //calculating corrected diff
        diff = (float)sonar_array[(sonar_counter + (samples-1))%samples] - curr_dist;
        //Serial.print("DIFF VALID -----  ");
      }
      else {
        //diff = average of all previous diffs
        diff = (float)diffSum/diffCounter;
      }
      diffSum += diff;
      distanceTravelled += diff;
        Serial.print(" Raw Data value: ");
        Serial.print(curr_dist);
        Serial.print("  TOF Sensor reading: ");
        Serial.println(getDistance_tof());
//        Serial.print(" STD: ");
//        Serial.print(std);
//        Serial.print(" STD/AVG_DIST: ");
//        Serial.print(std/avg_dist);
//        Serial.print(" Diff value: ");
//        Serial.print(diff);
//        Serial.print(" Diff Prev Dist: ");
//        Serial.print(sonar_array[(sonar_counter + (samples-1))%samples]);
//        Serial.print(" Diff Curr Dist: ");
//        Serial.print(curr_dist);
//      Serial.print("  Distance Travelled ");
//      Serial.print(distanceTravelled);

      sonar_counter = (sonar_counter + 1)%samples;

//      Serial.print("Diff AVG: ");
//      Serial.println((float)diffSum/diffCounter);

    }

    if (true_dist < 120) break;
    
    // Motor Movement
    setMotorPWM(pwm_r, pwm_l);
  
    // IMU Data
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    curr_angle = euler.x();
    
    // Note: Clockwise is +ive
    error = curr_angle - target_angle_x;
  
    // Condition if angle shifts from 0 to 359...
    if (error > 300) {
      error = error - 360;
    }
    
    if (error < -300) {error = error + 360;}
  
    errorSum = errorSum + error;
    errorSum = constrain(errorSum, -300, 300);

    if (dir == 0)
    {
      pwm_r = pwm_r + ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(curr_angle - prev_angle_x)/sampleTime));
      pwm_l = pwm_l - ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(curr_angle - prev_angle_x)/sampleTime));
    }
    else if (dir == 1)
    {
      pwm_r = pwm_r - ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(curr_angle - prev_angle_x)/sampleTime));
      pwm_l = pwm_l + ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(curr_angle - prev_angle_x)/sampleTime));
    }
  
    prev_angle_x = curr_angle;
  
    pwm_r = constrain(pwm_r, -255, 255);
    pwm_l = constrain(pwm_l, -255, 255);

    if ((millis() - currTime) < delayTime_millis)
    {
      delay(delayTime_millis - (millis() - currTime));
    }

    temp = millis() - init_time;   

//  } while ((((distanceTravelled) < 250 && (temp) < 1850)) || temp < 1000);
  } while ((((distanceTravelled) < 250 && (temp) < 2000)) || temp < 1500);
  
  stopMotors();

  Serial.print("Initial Distance: ");
  Serial.print(init_distance);
  Serial.print("  Final measured distance: ");
  Serial.print(true_dist);
  Serial.print("  Final measured time: ");
  Serial.print(temp);
  Serial.print("  Distance Travelled: ");
  Serial.println(distanceTravelled);

  delay(1000);

  // Angle Correction
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  curr_angle = euler.x();


  if (curr_angle - target_angle_x > 200) {curr_angle = curr_angle - 360;}
  if (curr_angle - target_angle_x < -200) {curr_angle = curr_angle + 360;}
  
  if (abs(curr_angle - target_angle_x) > 1)
  {
    if (curr_angle < target_angle_x)
    {
      turnAngle(0, abs(curr_angle - target_angle_x), 1);
    }
    else if (curr_angle > target_angle_x)
    {
      turnAngle(1, abs(curr_angle - target_angle_x), 1);
    }
  }
  
}

///*
// *  bool dir 0 goes forwards, 1 goes backwards
// */
//void goStraight(int dir)
//{
//  int delayTime_millis = 50;
//  float curr_angle = 0;
//  float prev_angle_x = 0;
//  float error = 0, errorSum = 0;
//  float sampleTime = 0.05;
//  unsigned long currTime = 0, prevTime = 0;
//
//  int ultra_dist_left_init = sonar_dist(9);
//  int ultra_dist_right_init = sonar_dist(13);
//
//  int ultra_dist_left = 0;
//  int ultra_dist_right = 0;
//  
//  int tof_front = 0;
//  int tof_back = 0;
//  int samples = 3;
//
//  for (int i = 0; i < samples; i++){
//    tof_front += getDistance_tof()/samples;
//    tof_back += requestTOF()/samples;
//    ultra_dist_left += sonar_dist(9)/samples;
//    ultra_dist_right += sonar_dist(13)/samples;
//  }
//  
//  int current_front_dist[samples] = {0};
//  int current_back_dist[samples]  = {0};
//  
//  for (int i = 0; i < samples; i++)
//  {
//    current_front_dist[i] = tof_front;
//    current_back_dist[i] = tof_back;
//  }
//
//  int current_ultra_dist_left[samples] = {ultra_dist_left, ultra_dist_left, ultra_dist_left};
//  int current_ultra_dist_right[samples] = {ultra_dist_right, ultra_dist_right, ultra_dist_right};
//
//  Serial.print("Sonar Left: ");
//  Serial.print(ultra_dist_left_init);
//  Serial.print("  Sonar Right: ");
//  Serial.print(ultra_dist_right_init);
//  Serial.print("  TOF Front: ");
//  Serial.print(tof_front);
//  Serial.print("  TOF Back: ");
//  Serial.println(tof_back);
//  
//  int counter = 0;
//  int front_avg = tof_front;
//  int back_avg = tof_back;
//  int ultra_left_avg = ultra_dist_left;
//  int ultra_right_avg = ultra_dist_right;
//  int avg_error = 0;
//
//  
//  int curr_dist = 0;
//  
//  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//
//  int pwm_r = 250;
//  int pwm_l = 250;
//
//  if (dir == 0)
//  {
//    setForwards();
//  }
//  else if (dir == 1)
//  {
//    setBackwards();
//  }
//
//
////  for (int i = 0; i < counter; i++)
//  do
//  {
//    currTime = millis();
//    sampleTime = (currTime - prevTime)/1000.0;
//    prevTime = currTime;
//    
//    // Motor Movement
//    setMotorPWM(pwm_r, pwm_l);
//  
//    // IMU Data
//    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//    curr_angle = euler.x();
//    
//    // Note: Clockwise is +ive
//    error = curr_angle - target_angle_x;
//  
//    // Condition if angle shifts from 0 to 359...
//    if (error > 300) {
//      error = error - 360;
//    }
//    
//    if (error < -300) {error = error + 360;}
//  
//    errorSum = errorSum + error;
//    errorSum = constrain(errorSum, -300, 300);
//
//    if (dir == 0)
//    {
//      pwm_r = pwm_r + ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(curr_angle - prev_angle_x)/sampleTime));
//      pwm_l = pwm_l - ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(curr_angle - prev_angle_x)/sampleTime));
//    }
//    else if (dir == 1)
//    {
//      pwm_r = pwm_r - ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(curr_angle - prev_angle_x)/sampleTime));
//      pwm_l = pwm_l + ((Kp*error) + (Ki*errorSum*sampleTime) - (Kd*(curr_angle - prev_angle_x)/sampleTime));
//    }
//  
//    prev_angle_x = curr_angle;
//  
//    pwm_r = constrain(pwm_r, -255, 255);
//    pwm_l = constrain(pwm_l, -255, 255);
//
//    front_avg -= current_front_dist[counter]/samples;
//    back_avg -= current_back_dist[counter]/samples;
//    ultra_left_avg -= current_ultra_dist_left[counter]/samples;
//    ultra_right_avg -= current_ultra_dist_right[counter]/samples;
//    
//    current_front_dist[counter] = getDistance_tof();
//    current_back_dist[counter] = requestTOF();
//    current_ultra_dist_left[counter] = sonar_dist(9);
//    current_ultra_dist_right[counter] = sonar_dist(13);
//        
//    front_avg += current_front_dist[counter]/samples;
//    back_avg += current_back_dist[counter]/samples;
//    ultra_left_avg += current_ultra_dist_left[counter]/samples;
//    ultra_right_avg += current_ultra_dist_right[counter]/samples;
//    counter = (counter + 1)%samples;
//    // + abs(tof_back - back_avg);// + abs(ultra_left_avg - ultra_dist_left_init);// + abs(ultra_right_avg - ultra_dist_right_init);
//    //avg_error = (abs(tof_front - front_avg) + abs(tof_back - back_avg) + abs(current_ultra_dist_right - ultra_dist_right_init) + abs(current_ultra_dist_left - ultra_dist_left_init))/4;
//    avg_error = (ultra_dist_left_init - ultra_left_avg)  + (tof_front - front_avg) + (back_avg - tof_back) + (ultra_dist_right_init - ultra_right_avg);
////    delay(delayTime_millis);
//  } while(avg_error/4 < 200);
//
//  
//  stopMotors();
//  delay(1000);
//
//  Serial.print("Sonar Left: ");
//  Serial.print(ultra_left_avg);
//  Serial.print("  Sonar Right: ");
//  Serial.print(ultra_right_avg);
//  Serial.print("  TOF Front: ");
//  Serial.print(front_avg);
//  Serial.print("  TOF Back: ");
//  Serial.println(back_avg);
//  
////
////  // Angle Correction
////  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
////  curr_angle = euler.x();
////
////
////  if (curr_angle - target_angle_x > 200) {curr_angle = curr_angle - 360;}
////  if (curr_angle - target_angle_x < -200) {curr_angle = curr_angle + 360;}
////  
////  if (abs(curr_angle - target_angle_x) > 1)
////  {
////    if (curr_angle < target_angle_x)
////    {
////      turnAngle(0, abs(curr_angle - target_angle_x));
////    }
////    else if (curr_angle > target_angle_x)
////    {
////      turnAngle(1, abs(curr_angle - target_angle_x));
////    }
////  }
//  
//}

/*
 * bool dir:  0 is clockwise, 1 is counter-clockwise
 * 
 * angle MUST be positive!!!
 * 
 */
void turnAngle(bool dir, float angle, bool angle_correct)
{
  float curr_angle = 0; 
  int pwm_r = 250;
  int pwm_l = 250;

  curr_angle = target_angle_x;
  if (!angle_correct)  target_angle_x = (target_angle_x + angle - (2*dir)*angle + 360);
  while(target_angle_x >= 360.0) target_angle_x -= 360.0;
  
  // Setting Direction
  if (dir == 0)
  {
    // Modifying angle to account for momentum to stop
    if (angle > 6)
    {
      angle = angle - 6;
    }
    
    angle = curr_angle + angle;
    if(angle >= 360.0) angle -= 360.0;
    
    setClockwise();
    setMotorPWM(pwm_r, pwm_l);

    // For example, from 350 to 80 when going positive direction, go from 350 to 360, then 360=0, then 0 to 80 later
    while (curr_angle - angle > 0)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      curr_angle = euler.x();
    }
  
    while (curr_angle < angle)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      curr_angle = euler.x();
    }
  }

  else if (dir == 1)
  {
    // Modifying angle to account for momentum to stop
    if (angle > 6)
    {
      angle = angle - 5;
    }
    
    angle = curr_angle - angle; // Modifying angle to account for momentum to stop
    // Check for < 0
    if (angle < 0) {angle = angle + 360;}
    
    setCounterClockwise();

    setMotorPWM(pwm_r, pwm_l);

    // For example, from 80 to 350 when going negative direction, go from 80 to 0, then 0=360, then 360 to 350 later
    while (curr_angle - angle < 0)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      curr_angle = euler.x();
    }
  
    while (curr_angle > angle)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      curr_angle = euler.x();
    }
  }
  
  stopMotors();
}

/*
 *  Continuous Move
 */
void continuousAngle(bool dir)
{
  int pwm_r = 250;
  int pwm_l = 250;
  
  // Setting Direction
  if (dir == 0)
  {
    setClockwise();
      
    setMotorPWM(pwm_r, pwm_l);
  }

  else if (dir == 1)
  {
    setCounterClockwise();
      
    setMotorPWM(pwm_r, pwm_l);
  }
}

////////////////////////////////////////////


void setup()
{
  Serial.begin(19200);
  init_colour_sensor();
  init_flame_sensor();
  init_tof();
  init_motor();
  init_IMU();
  
//  detect_objects();
}

void loop() {
  Serial.println(requestColour());
  delay(500);
  
  //bool mag = Mag_detect();
//Serial.println(mag);
//  Serial.println("STARTING LOOP");
//  //INITIALIZATION
//    tile board[DIMENSION][DIMENSION];
//    createBoard(board,DIMENSION);
//    currentx = STARTINGX;
//    currenty = STARTINGY;
//
//  //SETTING MOCK COORDINATES - FOR TESTING PURPOSES ONLY
//mockFire = &board[3][4];
//    mockFood = &board[4][4];
//    mockGroup = &board[5][1];
//    mockSurvivor = &board[0][4];
//    mockSand1 = mockFood;
//    mockSand2 = NULL;
//    mockSand3 = &board[1][1];
//    mockSand4 = &board[3][2];
//    mockWater1 = &board[0][3];
//    mockWater2 = &board[3][5];
//    mockWater3 = NULL;
//    mockWater4 = &board[4][1];
//
//Detect_Flame();
//
//
//  roam(board);
//  while(true);
//    delay(10000000);


//Current Testing
//  detect_objects();

 // goStraight(0);
//  Serial.println(getDistance_tof());
//  delay(100);
 // while(true);  
}
