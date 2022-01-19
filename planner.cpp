#include <math.h>
#include <mex.h>
#include <stdio.h>
#include <string.h>
#include <utility>
#include <stdio.h>
#include <string.h>
#include <queue>
#include <unordered_map>
#include <climits>
#include <iostream>
#include <ctime>
#include <cmath>
#include <stack>
#include <chrono>
#include <fstream>
#include "myMap.hpp"
using namespace std::chrono;
using namespace std;

#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]

/* Output Arguments */
#define	ACTION_OUT              plhs[0]


#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

#define printfunc(...) { mexPrintf(__VA_ARGS__); mexEvalString("drawnow;");}

int runCount = 0;
clock_t start;




bool myMap::check_validation(int index)
{
    if (indexToxy(index).first >= 1 && indexToxy(index).first <= x_size && indexToxy(index).second >= 1 && indexToxy(index).second <= y_size) {
        if ((map[index] >= 0) && (map[index] < collision_thresh)) {

            return true;
        }

    }

    return false;
}



int myMap::reachablePointIndex(int dir, int currentPoint_index)
{
    //get node coordinates from index
    pair<int, int> currentPoint = indexToxy(currentPoint_index);
    return xyToindex(currentPoint.first + dX[dir], currentPoint.second + dY[dir]);
}


int myMap::reachablePointIndex3D(int dir, int currentPoint_index3D)
{
    //get node coordinates from index
    pair<int, int> currentPoint = indexToxy(currentPoint_index3D);
    int x = getX3D(currentPoint_index3D);
    int y = getY3D(currentPoint_index3D, x);
    int t = getT3D(currentPoint_index3D);
    return xytToindex(x + dX[dir], y + dY[dir], t + 1);
}


int myMap::calculateH(int i) {



    int x = getX3D(i);
    int y = getY3D(i, x);
    int index_2d = xyToindex(x, y);
    return collision_thresh * 30 * pointdata_d[index_2d].g;
}


void myMap::dijkstra()
{

    int index;

    for (int i = target_steps / 2; i < this->target_steps; i++)
    {
        /*printf("(%d,%d)\n", this->target_traj[i], this->target_traj[i + this->target_steps]);*/
        int index = xyToindex(this->target_traj[i - 1], this->target_traj[i - i + this->target_steps]);
        pointdata_d[index].g = 0;
        this->openList_d.push(make_pair(pointdata_d[index].g, index));
    }

    while (!this->openList_d.empty()) {

        pair<int, int> node = this->openList_d.top();

        this->openList_d.pop();
        if (closedList_d[node.second] == true)
            continue;
        closedList_d[node.second] = true;
        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            int Index_next = reachablePointIndex(dir, node.second);
            if (!check_validation(Index_next))
                continue;

            if (pointdata_d[Index_next].g > pointdata_d[node.second].g + map[Index_next]) {
                pointdata_d[Index_next].g = pointdata_d[node.second].g + map[Index_next];
                //printf("%d\n", pointdata_d[Index_next].g);
                this->openList_d.push(make_pair(pointdata_d[Index_next].g, Index_next));
                /*                printf("lol\n");
                                int test = xyToindex(122, 247);
                                int toprint = pointdata_d[test].g;
                                printf("testdata %d\n", toprint)*/
            }

        }

    }

    //printf("di done");


}

pair<int, int> myMap::computePath()
{
    //int newx, newy;
    auto timer_start = high_resolution_clock::now();
    while (!this->openList.empty())
    {

        pair<int, int> currentPoint_F_and_Index = this->openList.top();     //f-value, cell index
        int Point_3dIndex = currentPoint_F_and_Index.second;
        //Remove it from the 
        this->openList.pop();

        this->closedList[Point_3dIndex] = true;     //cell index, closed

        int currentX = pointdata[Point_3dIndex].x;
        int currentY = pointdata[Point_3dIndex].y;
        int currentT = pointdata[Point_3dIndex].t;
        //printf("parent: (%d,%d,%d)\n", currentX, currentY, currentT);
        for (int dir = 0; dir < 9; dir++) {
            //printf("B\n ");
            int newt = currentT + 1;
            int X_2d = currentX + dX[dir];
            int Y_2d = currentY + dY[dir];
            //printf("child: (%d,%d,%d)\n", X_2d, Y_2d, newt);
            //printf("nextX:%d, nextY:%d, nextT:%d\n", X_2d, Y_2d, newt);
            int index_2d = xyToindex(X_2d, Y_2d);

            int index2d_next = reachablePointIndex(dir, index_2d);
            int index3d_next = reachablePointIndex3D(dir, Point_3dIndex);
            if (!check_validation(index2d_next) || newt <= 0 || newt >= target_steps)
                continue;
            auto search = pointdata.find(index3d_next);

            if (atGoal(X_2d, Y_2d, newt, target_steps)) {
                //printf("C\n ");
                auto timer_end = high_resolution_clock::now();
                auto duration = duration_cast<milliseconds>(timer_end - timer_start);

                printf("goal at: (%d,%d)\n", X_2d, Y_2d);
                pointdata[index3d_next].parent = Point_3dIndex;
                pointdata[index3d_next].x = X_2d;
                pointdata[index3d_next].y = Y_2d;
                pointdata[index3d_next].t = newt;

                pair<int, int> nextPose = trackPath2(index3d_next);

                printf("next pose: (%d, %d)\n", nextPose.first, nextPose.second);

                action_ptr[0] = nextPose.first;

                action_ptr[1] = nextPose.second;

                pair <int, int> toreturn = make_pair(action_ptr[0], action_ptr[1]);
                return toreturn;
            }
            //printf("hello!");
            //if not in openlist
            if (search == pointdata.end()) {
                //printf("D\ ");
                pointdata[index3d_next].x = X_2d;
                pointdata[index3d_next].y = Y_2d;
                pointdata[index3d_next].t = newt;
                pointdata[index3d_next].parent = Point_3dIndex;
                //printf("D1\ ");
                pointdata[index3d_next].h = calculateH(index3d_next);
                //printf("D2\ ");
                pointdata[index3d_next].g = pointdata[Point_3dIndex].g + (int)map[index2d_next];
                //printf("D3\ ");
                pointdata[index3d_next].f = pointdata[index3d_next].g + pointdata[index3d_next].h;
                //printf("Z\n ");

                this->openList.push(make_pair(pointdata[index3d_next].f, index3d_next));
                //printf("F\n ");


            }
            // already in the openlist
            else if ((search != pointdata.end()) && (pointdata[index3d_next].g > pointdata[Point_3dIndex].g + map[index2d_next])) {
                //printf("E\ ");
                pointdata[index3d_next].g = pointdata[Point_3dIndex].g + (int)map[index2d_next];
                pointdata[index3d_next].f = pointdata[index3d_next].h + pointdata[index3d_next].g;
                pointdata[index3d_next].parent = Point_3dIndex;
            }

        }

    }
}


pair<int, int> myMap::trackPath2(int index) {

    int nextNode = index;
    int currentNode = index;
    int prevNode;
    int x, y, t;
    pair<int, int> newone = indexToxy(index);
    path.push(newone);
    pair<int, int> next;

    do {
        //auto search = this->pointdata.find(nextNode);
        //x = nextNode / (y_size * t_size);
        x = pointdata[nextNode].x;
        y = pointdata[nextNode].y;
        //y = (nextNode - (x * t_size * y_size)) / t_size;
        t = pointdata[nextNode].t;

        prevNode = currentNode;
        currentNode = nextNode;
        nextNode = pointdata[nextNode].parent;
        next = { x,y };

        if (next != path.top())
        {
            printf("(%d,%d)\n", x, y);
            path.push(next);
        }

    } while (nextNode != currentNode);

    //pair<int,int> robotAction(x, y);
    this->pathFound = true;
    return next;

}


bool myMap::atGoal(int x, int y, int t, int t_size) {

    int targetIndex = xytToindex(target_traj[t], target_traj[t + target_steps], t);
    int robotIndex = xytToindex(x, y, t);
    return (targetIndex == robotIndex);


}



pair<int, int> startGO(
    double* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    double* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    double* action_ptr
)
{
    static myMap GG(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps,
        target_traj, targetposeX, targetposeY, curr_time, action_ptr);

    if (GG.pathFound) {


        if (GG.path.size() > 1) {
            printf("path size: %d\n", GG.path.size());
            pair<int, int> next = GG.path.top();
            pair<int, int> toReturn = make_pair(next.first, next.second);
            GG.path.pop();
            printf("next pose: (%d,%d)\n", next.first, next.second);
            return toReturn;
        }
        else {
            pair<int, int> toReturn = make_pair(robotposeX, robotposeY);
            printf("current pose: (%d,%d)\n", robotposeX, robotposeY);
            return toReturn;
        }
    }
    GG.initStartpoint();
    auto timer_start = high_resolution_clock::now();
    GG.dijkstra();
    auto timer_end = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(timer_end - timer_start);
    printf("Dijkstra done in: %d ms\n", duration);
    timer_start = high_resolution_clock::now();

    printf("hello\n");
    pair<int, int> nextXY = GG.computePath();
    return nextXY;

}

static void planner(
    double* map,
    int collision_thresh,
    int x_size,     //Number of columns
    int y_size,     //Number of rows
    int robotposeX,
    int robotposeY,
    int target_steps,
    double* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    double* action_ptr
)
{
    runCount++;

    pair<int, int> nextXY = startGO(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time, action_ptr);

    action_ptr[0] = nextXY.first;

    action_ptr[1] = nextXY.second;
}






















void mexFunction(int nlhs, mxArray* plhs[],
    int nrhs, const mxArray* prhs[])

{

    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
            "Six input arguments required.");
    }
    else if (nlhs != 1) {
        mexErrMsgIdAndTxt("MATLAB:planner:maxlhs",
            "One output argument required.");
    }

    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);

    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if (robotpose_M != 1 || robotpose_N != 2) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidrobotpose",
            "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];

    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);

    if (targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidtargettraj",
            "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;

    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if (targetpose_M != 1 || targetpose_N != 2) {
        mexErrMsgIdAndTxt("MATLAB:planner:invalidtargetpose",
            "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];

    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);

    /* Create a matrix for the return action */
    ACTION_OUT = mxCreateNumericMatrix((mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL);
    double* action_ptr = (double*)mxGetData(ACTION_OUT);

    /* Get collision threshold for problem */
    int collision_thresh = (int)mxGetScalar(COLLISION_THRESH);

    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;
}