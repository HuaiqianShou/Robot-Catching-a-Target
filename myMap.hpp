#pragma once

using namespace std;
#define NUMOFDIRS 8
class myMap
{
    double* map;
    int collision_thresh;
    int x_size;    
    int y_size;     
    int robotposeX;
    int robotposeY;
    int target_steps;
    double* target_traj;
    int targetposeX;
    int targetposeY;
    int curr_time;
    double* action_ptr;





    struct point
    {
        int parent = -100;
        int g = INT_MAX;
        int h = 0;
        int f = INT_MAX;
        int x, y, t;
    };


    struct point_d
    {
        int parent = -1;
        int g = INT_MAX;
    };

    unordered_map<int, bool> closedList;     //closedList of bool values for each cell
    unordered_map<int, point> pointdata;       //Stores info of each cell corresponding to the index of the cell

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> openList;   //f-value, cell index (sorted in increasing order of f-value)





    unordered_map<int, bool> closedList_d;     //closedList of bool values for each cell
    unordered_map<int, point_d> pointdata_d;       //Stores info of each cell corresponding to the index of the cell

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> openList_d;


    int dX[NUMOFDIRS + 1] = { -1, -1, -1,  0,  0,  1, 1, 1 ,0 };
    int dY[NUMOFDIRS + 1] = { -1,  0,  1, -1,  1, -1, 0, 1 ,0 };
public:
    int t_size = target_steps;
    int start_time = 30;
    int goalposeX = 1;
    int goalposeY = 1;
    bool destReached = false;
    stack <int> trackPath;
    stack<pair<int, int>, vector<pair<int, int>>> path;
    bool pathFound = false;
    myMap(
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
        this->map = map;
        this->collision_thresh = collision_thresh;
        this->x_size = x_size;
        this->y_size = y_size;
        this->robotposeX = robotposeX;
        this->robotposeY = robotposeY;
        this->target_steps = target_steps;
        this->target_traj = target_traj;

        this->curr_time = curr_time;
        this->targetposeX = targetposeX;
        this->targetposeY = targetposeY;
        this->action_ptr = action_ptr;
    }

    int xyToindex(int x, int y)
    {
        return (y - 1) * x_size + (x - 1);
    }

    long long int xytToindex(int x, int y, int t) {

        return((x * target_steps * y_size) + (y * target_steps) + t);

    }


    int getX3D(int index) {
        return (index / (y_size * target_steps));
    }

    int getY3D(int index, int x) {

        return ((index - (x * y_size * target_steps)) / target_steps);
    }

    
    int getT3D(int index) {
        return (index % target_steps);

    }

    pair<int, int> indexToxy(int index)
    {
        return (make_pair((index % x_size) + 1, (index / x_size) + 1));
    }

    void initStartpoint()
    {
        pointdata[xytToindex(robotposeX, robotposeY, start_time)].parent = xytToindex(robotposeX, robotposeY, start_time);
        pointdata[xytToindex(robotposeX, robotposeY, start_time)].g = 0;
        pointdata[xytToindex(robotposeX, robotposeY, start_time)].x = robotposeX;
        pointdata[xytToindex(robotposeX, robotposeY, start_time)].y = robotposeY;
        pointdata[xytToindex(robotposeX, robotposeY, start_time)].t = start_time;

        // ???0? ????F value ?0
        this->openList.push(make_pair(0, xytToindex(robotposeX, robotposeY, start_time)));
        printf("starting point: (%d,%d,%d)\n", robotposeX, robotposeY, start_time);

    }

    bool isInopenList(int, int);
    int calculateH(int);
    pair<int, int> computePath();
    int reachablePointIndex(int, int);

    int reachablePointIndex3D(int, int);
    bool check_validation(int);
    void dijkstra();
    pair<int, int>trackPath2(int);
    bool atGoal(int, int, int, int);
    //bool check_validation3D(int,int);

};
