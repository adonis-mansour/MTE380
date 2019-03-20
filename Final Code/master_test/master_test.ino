#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

//PIN CONSTANTS


//L/R/B ULTRASONIC FUNCTIONS

// FLAME SENSOR



// Object ID

// TURN FAN ON FUNCTION

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
        Serial.print("Starting == end");
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
    Serial.print("Headnode: ");
    Serial.print(path.headNode->s->location.x);
    Serial.print(" ,");
    Serial.println(path.headNode->s->location.y);
    node* currentNode = (node*)malloc(sizeof(node));
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
    if(temp == NULL) {
      Serial.println("temp == null");
      return;
    }
    //Serial.println("In decode list 2");
    
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
        unprobe();
        
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
        unprobe();
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
    while(!foodFound | !survivorFound){
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
    mockFire = &board[0][4];
    mockFood = &board[4][4];
    mockGroup = &board[3][4];
    mockSurvivor = &board[0][0];
    mockSand1 = mockFood;
    mockSand2 = &board[5][1];
    mockSand3 = &board[3][3];
    mockSand4 = &board[0][2];
    mockWater1 = &board[2][1];
    mockWater2 = &board[3][1];
    mockWater3 = &board[4][2];
    mockWater4 = NULL;
  roam(board);
  while(true);
    //delay(10000000);
}
