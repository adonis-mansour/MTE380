#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

/////////////////////////////////////////////
///////////// SENSOR DEFS  /////////////
/////////////////////////////////////////////

//PIN CONSTANTS
// FLAME SENSOR DEFS
#define FLAME1 50 
#define FLAME2 48 
#define FLAME3 51 
#define FLAME4 49 
#define FAN 52

//ULTRASONIC DEFS
#define trigPin 8  //Trigger                                                               
#define echoPin2 5 //Right_Front
#define echoPin3 6 //Right_Back
#define echoPin4 2  //Left_Front
#define echoPin5 3  //Left_Back
#define echoPin6 7  //Rear_R                                     
#define echoPin7 4 //Rear_L

#define US_REQUEST 45
#define US_DATA 46
#define US_SENT 47

// HALL EFFECT DEFS
#define HE0 A0
#define HE1 A1
#define HE2 A2
#define HE3 A3
#define HE4 A4
#define HE5 A5
#define HE6 A6
#define HE7 A7
#define HE8 A8

//COLOUR SENSOR RECEIVER DEFS
#define Colour_REQUEST 48
#define Colour_RED_HOUSE 49
#define Colour_YELLOW_HOUSE 50
#define Colour_OTHER 51
#define Colour_SENT 52

//IR SENSOR 
#define sensor0 A9  //F                              
#define sensor1 A10  //R
#define sensor2 A11  //L
#define sensor3 A12  //B
////////////////////////////////////////////

/////////////////////////////////////////////
///////////// SENSOR CONSTS  ///////////////
/////////////////////////////////////////////

//ULTRASONIC CONSTANTS
long US_duration, US_distance; 
int US_counter = 0, US = 0;
boolean sensors_detected[8] = {0};
boolean object_location[4] = {0};

const int US_NumOfValues = 3;
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

void getObstacles(bool* obstacles, tile* current){
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
        probe();
        fireDetected(current, board);
        groupDetected(current, board);
        survivorDetected(current, board);
        unprobe();
        relativePointer(current, 0)->isObstacle = true;
    }

    if(obstacles[1] && relativePointer(current, 2) != NULL && !relativePointer(current, 2)->isObstacle){
        rr90();
        probe();
        fireDetected(current, board);
        groupDetected(current, board);
        survivorDetected(current, board);
        unprobe();
        rl90();
        relativePointer(current, 2)->isObstacle = true;
    }

    if(obstacles[2] && relativePointer(current, 4) != NULL && !relativePointer(current, 4)->isObstacle){
        r180();
        probe();
        fireDetected(current, board);
        groupDetected(current, board);
        survivorDetected(current, board);
        unprobe();
        r180();
        relativePointer(current, 4)->isObstacle = true;
    }

    if(obstacles[3] && relativePointer(current, 6) != NULL && !relativePointer(current, 6)->isObstacle){
        rl90();
        probe();
        fireDetected(current, board);
        groupDetected(current, board);
        survivorDetected(current, board);
        unprobe();
        rr90();
        relativePointer(current, 6)->isObstacle = true;
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
        distance[0] = (1 + visited + !(current->frontSquare->type == 1 && !visited))*findDistance(current->frontSquare->location, end->location);
        //printf("front dist: %d Distance to goal:%d Is visited?: %d \n", distance[0],findDistance(current->frontSquare->location, end->location),visited);
    }
    if (current->rightSquare != NULL && current->rightSquare->type != 2 && !current->rightSquare->isObstacle){
        visited = isVisited[current->rightSquare->location.x][current->rightSquare->location.y];
        distance[1] = (1 + visited + !(current->rightSquare->type == 1 && !visited))*findDistance(current->rightSquare->location, end->location);
        //printf("right dist: %d Distance to goal:%d Is visited?: %d \n", distance[1],findDistance(current->rightSquare->location, end->location),visited);
    }

    if (current->behindSquare != NULL && current->behindSquare->type != 2 && !current->behindSquare->isObstacle){
        visited = isVisited[current->behindSquare->location.x][current->behindSquare->location.y];
        distance[2] = (1 + visited + !(current->behindSquare->type == 1 && !visited))*findDistance(current->behindSquare->location, end->location);
        //printf("behind dist: %d Distance to goal:%d Is visited?: %d \n", distance[2],findDistance(current->behindSquare->location, end->location),visited);
    }

    if (current->leftSquare != NULL && current->leftSquare->type != 2 && !current->leftSquare->isObstacle){
        visited = isVisited[current->leftSquare->location.x][current->leftSquare->location.y];
        distance[3] = (1 + visited + !(current->leftSquare->type == 1 && !visited))*findDistance(current->leftSquare->location, end->location);
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
                double dist = findDistance(current->location, tempSquare->location);
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
    return currentBest;
}

void roam(tile board[][DIMENSION]){
  Serial.println("In Roam");
   bool isVisited[DIMENSION][DIMENSION];
  Serial.println("In Roam2"); 
    for (int i = 0; i < DIMENSION; i++){
        for (int j = 0; j < DIMENSION; j++){
            isVisited[i][j] = false;
        }
    }
   Serial.println("In roam3");
  isVisited[currentx][currenty] = true;
  reveal(&board[currentx][currenty],1);
  scanArea(&board[currentx][currenty], board);
  tile* target;
  target = findTarget(&board[currentx][currenty], board, isVisited);
  Serial.println("In roam 4");
  while(!fireFound){
    while (!fireFound && &board[currentx][currenty] != target && !target->isObstacle && target->type != 2){
        //printf("Target: (%d,%d)\n", target->location.x, target->location.y);
        moveTowards(&board[currentx][currenty], target, isVisited, board);
        if(currentx == 1 && currenty == 1 && !board[0][0].isRevealed){
            Serial.println("Corner not revealed");
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
    Serial.println("In roam 5");
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
    while(!foodFound | !survivorFound){
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
        if(foodFound && !foodSignal) {
            signal();
            foodSignal = true;
        }
        if(survivorFound && !survSignal) {
            signal();
            survSignal = true;
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
    Serial.print("Ending Roam");
}
////////////////////////////////////////////

/////////////////////////////////////////////
///////////// SENSOR FUNCTIONS  /////////////
/////////////////////////////////////////////

//L/R/B ULTRASONIC FUNCTIONS
void init_US_sensor() {
  pinMode(US_REQUEST, OUTPUT);
  pinMode(US_DATA, INPUT);
  pinMode(US_SENT, INPUT);
}

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
bool front_sensors_check() {
  bool temp;
  digitalWrite(US_REQUEST, HIGH);
  while (digitalRead(US_SENT) == LOW){};
  if (digitalRead(US_DATA) == HIGH) {
    temp = 1;
  } else {
    temp = 0;
  }
  digitalWrite(US_REQUEST, LOW);
  return temp;
}

bool detect_objects() {
  //Serial.print("Sensor RF: ");
  SonarSensor(2, trigPin,echoPin2,  US2_values);   
  
  //Serial.print("Sensor RB: ");
  SonarSensor(3, trigPin,echoPin3,  US3_values);   
  
  //Serial.print("Sensor LF: ");
  SonarSensor(4, trigPin,echoPin4,  US4_values);   
  
  //Serial.print("Sensor LB: ");
  SonarSensor(5, trigPin,echoPin5,  US5_values);   
  
  //Serial.print("Sensor BR: ");
  SonarSensor(6, trigPin,echoPin6,  US6_values);   
  
  //Serial.print("Sensor BL: ");
  SonarSensor(7, trigPin,echoPin7,  US7_values);   

  if (front_sensors_check == true) {
    object_location[0] = true;
    Serial.print("Front: Detected     ");

  } else {
    object_location[0] = false;
    Serial.print("Front: Nothing      ");
  }

  
  if (sensors_detected[2] == true || sensors_detected[3]== true) {
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

  if (sensors_detected[6] == true || sensors_detected[7]== true) {
    object_location[3] = true;
    Serial.print("Back: Detected     ");
  } else {
    object_location[3] = false;
    Serial.print("Back: Nothing     ");
  }

  return object_location;
  
  US_counter = (US_counter + 1)%US_NumOfValues;

  Serial.println("   ");

  delay(50);
}

// HALL EFFECT
bool Mag_detect() {

  long NOFIELD = 512;
  float TOMILLIGAUSS = 3.756010;
  //int pin_array[9] = {HE0, HE1, HE2, HE3, HE4, HE5, HE6, HE7, HE8};
  float gauss[9] = {0};
  int mag = 0;
  int no_mag = 0;
  int threshold = 7;

  for (int i = 0; i < 13; i++ ) {
    delay(100);
    gauss[0] = (analogRead(HE1) - NOFIELD) * TOMILLIGAUSS;
    Serial.print(gauss[0]);
    Serial.println(i);
    gauss[1] = (analogRead(HE1) - NOFIELD) * TOMILLIGAUSS;
    gauss[2] = (analogRead(HE2) - NOFIELD) * TOMILLIGAUSS;
    gauss[3] = (analogRead(HE3) - NOFIELD) * TOMILLIGAUSS;
    gauss[4] = (analogRead(HE4) - NOFIELD) * TOMILLIGAUSS;
    gauss[5] = (analogRead(HE5) - NOFIELD) * TOMILLIGAUSS;
    gauss[6] = (analogRead(HE6) - NOFIELD) * TOMILLIGAUSS;
    gauss[7] = (analogRead(HE7) - NOFIELD) * TOMILLIGAUSS;
    gauss[8] = (analogRead(HE8) - NOFIELD) * TOMILLIGAUSS;
  }

  if ( (gauss[0] < -7) or (gauss[1] < -7) or (gauss[2] < -7) or (gauss[3] < -7) or (gauss[4] < -7) or (gauss[5] < -7) or (gauss[6] < -7) or (gauss[7] < -7) or (gauss[8] < -7)) {
    mag++;
  }
  else if ( (gauss[0] > 7) or (gauss[1] > 7) or (gauss[2] > 7) or (gauss[3] > 7) or (gauss[4] > 7) or (gauss[5] > 7) or (gauss[6] > 7) or (gauss[7] > 7) or (gauss[8] > 7)) {
    mag++;
  }
  else {
    no_mag++;
  }
    
  if (mag >= 3)
    return 1;
  else if (no_mag >= 10)
    return 0;
}

// COLOUR SENSOR RECEIVER // 0 = survior, 1 = house, 2 = candle
void init_colour_sensor() {
  pinMode(Colour_REQUEST, OUTPUT);
  pinMode(Colour_RED_HOUSE, INPUT);
  pinMode(Colour_YELLOW_HOUSE, INPUT);
  pinMode(Colour_OTHER, INPUT);
  pinMode(Colour_SENT, INPUT);
}
int colour_sensor_check() {
  int temp;
  digitalWrite(Colour_REQUEST, HIGH);
  while (digitalRead(Colour_SENT) == LOW){};
  if (digitalRead(Colour_YELLOW_HOUSE) == HIGH) {
    temp = 0;
  } else if (digitalRead(Colour_RED_HOUSE) == HIGH){
    temp = 1;
  } else {
    temp = 2;
  }
  digitalWrite(Colour_REQUEST, LOW);
  return temp;
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
  if(fire1 == LOW || fire2 == LOW || fire3 == LOW ||  fire4 == LOW) {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(FAN, HIGH);
    Serial.println("Fire");
    return true;
    //delay(200); // ??? IS THIS DELAY REQUIRED?
  }else{
    Serial.println("Nothing");
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(FAN, LOW);
    return false;
  }
  delay(200);
}
// IR FUNCTION
float Read_Distance(int sensor, int values[]) {
  volts = analogRead(sensor)*0.0048828125;  // value from sensor * (5/1024)
    IR_distance = 13*pow(volts, -1);             // worked out from datasheet graph
    delay(10); // slow down serial port 

    values[IR_counter] = IR_distance;
    IR = 0;
    for (int i=0; i<NumOfValues; i++) {
      IR = IR+values[i];
    }
    IR = (IR)/NumOfValues;
       
    Serial.print(IR);               // print the distance   
    return IR;
}
int detect_terrain() {
  //Front
  Serial.print("Front    ");
  terrain[0] = Read_Distance(sensor0, IR0_values);
  Serial.print("   ");
  
  //Right
  Serial.print("Right    ");
  terrain[1] = Read_Distance(sensor1, IR1_values);
  Serial.print("   ");
  
  //Left
  Serial.print("Left    ");
  terrain[2] = Read_Distance(sensor2, IR2_values);
  Serial.print("   ");
  
  //Back
  Serial.print("Rear    ");
  terrain[3] = Read_Distance(sensor3, IR3_values);
  IR_counter = (IR_counter + 1)%NumOfValues;
  Serial.print("   ");
  
  delay(50); 
}

// TURN FAN ON FUNCTION   // 1 = ON, 0 = OFF
    
void fan_control(bool mode) {
  if (mode == 1) {
    digitalWrite(FAN, HIGH);
  } else if (mode == 0) {
    digitalWrite(FAN, LOW);
  }
}
////////////////////////////////////////////


void setup()
{
  Serial.begin(9600);

}

void loop()
{
  Serial.println("STARTING LOOP");
  //INITIALIZATION
    tile board[DIMENSION][DIMENSION];
    createBoard(board,DIMENSION);
    currentx = STARTINGX;
    currenty = STARTINGY;

  //SETTING MOCK COORDINATES - FOR TESTING PURPOSES ONLY
//    mockFire = &board[1][3];
//    mockFood = &board[1][4];
//    mockGroup = &board[4][5];
//    mockSurvivor = &board[1][0];
//    mockSand1 = mockFood;
//    mockSand2 = &board[4][1];
//    mockSand3 = &board[3][3];
//    mockSand4 = NULL;
//    mockWater1 = &board[2][0];
//    mockWater2 = &board[4][4];
//    mockWater3 = &board[0][3];
//    mockWater4 = NULL;
//  roam(board);
//  //while(true);
//    delay(10000000);
}
