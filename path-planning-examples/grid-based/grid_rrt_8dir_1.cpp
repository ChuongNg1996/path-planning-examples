/*
Grid-based, RRT, 8-direction movement.

Basic RRT algorithm:
1. Randomly pick a FREE/NEW node on map (or near neighbor of initial postion?) within a [MAXIMUM DISTANCE] 
2. Connect it to nearest node on the tree.
    -> Obstacle in between them? -> YES -> Ignore
* Potentiall less node than A* since the nodes are spaced out (within a [MAXIMUM DISTANCE]).

RRT_Star:
Same but don't have to connect to nearest node. Instead, check around a set Radius to see if we can connect 
to furthest viable node


*/


#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>
using namespace std;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds
int main () {

    // ---------------------------------------------------------- //
    // ---------        File Variables                  --------- //
    // ---------------------------------------------------------- //
    ifstream indata;                            // input file
    //ofstream result;
    int num;                                    // store read values from map
    
    // ---------------------------------------------------------- //
    // ---------        Algorithm Variables             --------- //
    // ---------------------------------------------------------- //
    
    // Map Variable
    vector<int> storage;                        // Store read values from input file
    //int max = 0;                              // Count number of read values
    vector<vector<int>> map;                    // Store read values as 2D map
    vector<int> map_row;                        // Arrange values as row to assign to map
    int rows = 100;                             // Height in pixel
    int cols = 100;                             // Width in pixel
    int index = 2;                              // Start reading map from index 2 bc the first two are rows and cols

    // Path planning 
    int row;                                    // row variable
    int col;                                    // col variable
    int init_cell[2] = {10,93};                  // index of initial position (in row, col)
    int goal_cell[2] = {70,50};                 // index of goal position (in row, col)
    int current_cell[2] = {-1,-1};              // index of current cell (in row, col)
    //int next_cell[2] = {-1,-1};                 // index of next cell (in row, col)
    float dist;                                   // distance from init_cell to a free cell
    float min_dist = 0;                           // minimum distance of a cell to init_cell
    float max_dist = 0;                           // maximum distance of a cell to init_cell
    int direction_num = 8;                      // Number of moving direction
    vector<int> visited_rows;                   // Store visited rows
    vector<int> visited_cols;                   // Store visited columns

    int rrt_tree_size = rows*cols;              // Size of RRT tree
    vector<vector<int>> rrt_tree(rrt_tree_size);// store vertices & edges of RRT tree
    vector<int> visited_cells;                   // Store visited cells
    vector<int> next_cells;                   // Store visited cells
    
    // ---------------------------------------------------------- //
    // ---------         Open & Store Data              --------- //
    // ---------------------------------------------------------- //

    indata.open("100_100_room.txt"); // opens the file
    if(!indata) { // file couldn't be opened
        cerr << "Error: file could not be opened" << endl;
        exit(1);
    }

    // Read and invert value
    indata >> num;
    if (num != 0) num = 0;  
    else num = 1;

    while ( !indata.eof() ) { // keep reading until end-of-file
        storage.push_back(num);
        //max++;

        // Read and invert value
        indata >> num; // sets EOF flag if no value found
        if (num != 0) num = 0;
        else num = 1;
    }
    indata.close();


    // ---------------------------------------------------------- //
    // ---------                Read Map                --------- //
    // ---------------------------------------------------------- //

    // Push the pixel to map.
    for (auto i = 0; i < rows; i++)
    {
        for (auto j = 0; j < cols; j++)
        {
            map_row.push_back(storage[index]);
            index++;
        }
        map.push_back(map_row);
        map_row.clear();
    }

    // Display the map to check.
    cout << "DISPLAY THE MAP: " << "\n\n";
    for (auto i = 0; i < rows; i++)
    {
        for (auto j = 0; j < cols; j++)
        {
            cout << map[i][j] << " ";
            index++;
        }
        cout << endl;
    }
    cout << endl;

    // ---------------------------------------------------------- //
    // ---------           Path Planning Prepare        --------- //
    // ---------------------------------------------------------- //

    // Check if initial position is valid
    if (map[init_cell[0]][init_cell[1]] == 1)
    {
        cout << "Invalid initial cell. " << endl;
        return 0;
    } 
    else cout << "Valid intial cell value: " << map[init_cell[0]][init_cell[1]] << endl;
    map[init_cell[0]][init_cell[1]] = 3; //Random UNIQUE values to distinct with the rest

    // Check if goal position is valid
    if (map[goal_cell[0]][goal_cell[1]] == 1)
    {
        cout << "Invalid goal cell. " << endl;
        return 0;
    } 
    else cout << "Valid goal cell value: " << map[goal_cell[0]][goal_cell[1]] << endl;
    map[goal_cell[0]][goal_cell[1]] = 5; //Random UNIQUE values to distinct with the rest

    // ---------------------------------------------------------- //
    // ---------           Path Planning Algorithm      --------- //
    // ---------------------------------------------------------- //

    /*
    Append current to tree, in singular, unique value (index on storage) instead of 2D. e.g.: sin_val = row*rows + col 
    Chose random direction on 2D map, within maximum distance,  
        -> Check for validity (is that an obstacle? is there obstacle in between? Is the cell visited?) 
            -> if valid -> append new node, vertices.
    Repeat For ALL vertices on the tree until goal is detect,
    */

    int vertice;                                // Current cell
    int edge;                                   // Next random cell
    int g_vertice;                              // Goal in a single, unique value instead of 2D 
    int max_dist_cell = 3;                      // Maximum distance
    //float max_dist_rrt = sqrt(pow(max_dist_cell,2) + pow(max_dist_cell,2));

    random_device rd;                           // Will be used to obtain a seed for the random number engine
    mt19937 gen(rd());                          //Standard mersenne_twister_engine seeded with rd()
    uniform_int_distribution<int> distribution(-max_dist_cell,max_dist_cell); // Random number distribution in [ - max_dist_cell, max_dist_cell]

    int row_rand = 0;                           // Random row increment in limit of [ - max_dist_cell, max_dist_cell]
    int col_rand = 0;                           // Random col increment in limit of [ - max_dist_cell, max_dist_cell]
    int row_i = 0;                              // Index of row increment
    int col_i = 0;                              // INdex of col increment
    int row_rand_unit;                          // Direction, unit of row increment, either 1 or -1
    int col_rand_unit;                          // Direction, unit of col increment, either 1 or -1
    bool valid_cell = 0;                        // Check if the cell is valid or not


    //- Add the initial cell and its next cell to the array first before the loop -//

    current_cell[0] = init_cell[0];
    current_cell[1] = init_cell[1];

    row_rand = distribution(gen);     
    col_rand = distribution(gen);
    
    if (row_rand != 0) row_rand_unit = row_rand/abs(row_rand);  // Avoid divide by 0
    else row_rand_unit = 1;                                     // Terminate the while condition
    if (col_rand != 0) col_rand_unit = col_rand/abs(col_rand);  // Avoid divide by 0
    else col_rand_unit = 1;                                     // Terminate the while condition

   
    while (abs(row_i) <= abs(row_rand))
    {
        while (abs(col_i) <= abs(col_rand))
        {
            row = current_cell[0] + row_i;
            col = current_cell[1] + col_i;
            
            if (map[row][col] == 1)
            {
                valid_cell = 0;
                break;
            }
            else valid_cell = 1;
            
            col_i = col_i + row_rand_unit;
        }
        row_i = row_i + row_rand_unit;
    }

    if (valid_cell == 1)
    {
        vertice = current_cell[0]*rows + current_cell[1];
        edge = (current_cell[0]+row_rand)*rows + (current_cell[1]+col_rand);

        // rrt_tree[vertice].push_back(edge);
        // rrt_tree[edge].push_back(vertice);
        visited_cells.push_back(vertice);
        visited_cells.push_back(edge);
    } 
    

    // for (auto x: visited_cells)
    // {
    //     row = x/100;
    //     col = x%100;
    //     cout << row << " " << col << endl;
    // }


    g_vertice = goal_cell[0]*rows + goal_cell[1];

    // while goal cell (g_vertice) is not in visited_cells
    while (!(find(visited_cells.begin(), visited_cells.end(), g_vertice) != visited_cells.end()))
    {
        for (auto x: visited_cells)
        {
            cout << "\nx: " << x << endl;

            row = x/rows;
            col = x%rows;

            current_cell[0] = row;
            current_cell[1] = col;

            cout << current_cell[0]  << " " << current_cell[1]  << endl;

            row_rand = distribution(gen);     
            col_rand = distribution(gen);
            
            if (row_rand != 0) row_rand_unit = row_rand/abs(row_rand);  // Avoid divide by 0
            else row_rand_unit = 1;                                     // Terminate the while condition
            if (col_rand != 0) col_rand_unit = col_rand/abs(col_rand);
            else col_rand_unit = 1;

            row_i = 0;
            col_i = 0;

            cout << "row_rand_unit: " << row_rand_unit << " col_rand_unit: " << col_rand_unit << endl;

            /*
            This condition needs to be revised again in smaller scale map
            */
            while (abs(row_i) <= abs(row_rand))
            {
                //cout<< "lol 0: " << map[row][col] <<endl;

                while (abs(col_i) <= abs(col_rand))
                {
                    row = current_cell[0] + row_i;
                    col = current_cell[1] + col_i;
                    
                    if (map[row][col] == 1)
                    {
                        valid_cell = 0;
                        cout<< "lol 1: " << map[row][col] <<endl;
                        break;
                    }
                    else valid_cell = 1;
                    //cout<< "lol 2: " << map[row][col] <<endl;
                    //cout<< valid_cell <<endl;

                    col_i = col_i + row_rand_unit;
                }
                if (valid_cell == 0) break;
                row_i = row_i + row_rand_unit;
            }

            cout<< "valid cell: " << valid_cell <<endl;

            edge = (current_cell[0]+row_rand)*rows + (current_cell[1]+col_rand);

            if ((valid_cell == 1) && (!(find(visited_cells.begin(), visited_cells.end(), edge) != visited_cells.end()))) 
            {
                valid_cell = 1;
            }
            else valid_cell = 0;

            if (valid_cell == 1)
            {
                vertice = current_cell[0]*rows + current_cell[1];
                //rrt_tree[vertice].push_back(edge);
                //rrt_tree[edge].push_back(vertice);
                next_cells.push_back(vertice);
                next_cells.push_back(edge);
            } 
            cout << "edge: " << edge << endl;
            cout << "vertice: " << vertice << endl;
        }

        for (auto x: next_cells)
        {
            visited_cells.push_back(x);
        }
        next_cells.clear();

    cout << "\n\n\n\n\n\n";
    //sleep_for(nanoseconds(500000000));

    }

    for (auto x: visited_cells)
    {
        cout << x << " ";
        row = x/100;
        col = x%100;
        map[row][col] = 4;
    }
   

    // Assign current cell as goal cell, make a reverse path to initial cell
    // current_cell[0] = goal_cell[0];
    // current_cell[1] = goal_cell[1];

    // // While current cell doesn't arrive at initial cell
    // while ((current_cell[0] != init_cell[0]) || (current_cell[1] != init_cell[1]))
    // {
    //     min_dist = max_dist + 1; // Initialize minimum distance as maximum distance

    //     for (int i = 0; i < direction_num; i++)
    //     {
    //         switch(i)
    //         {
    //             // each case is a direction
    //             case 0:
    //             {
    //                 row = current_cell[0] - 1;  // Up
    //                 col = current_cell[1];      // Same
    //                 break;
    //             }
    //             case 1:
    //             {
    //                 row = current_cell[0] - 1;  // Up
    //                 col = current_cell[1] - 1;  // Left
    //                 break;
    //             }
    //             case 2:
    //             {
    //                 row = current_cell[0];      // Same
    //                 col = current_cell[1] - 1;  // Left
    //                 break;
    //             }
    //             case 3:
    //             {
    //                 row = current_cell[0] + 1;  // Down
    //                 col = current_cell[1] - 1;  // Left
    //                 break;
    //             }
    //             case 4:
    //             {
    //                 row = current_cell[0] + 1;  // Down
    //                 col = current_cell[1];      // Same
    //                 break;
    //             }
    //             case 5:
    //             {
    //                 row = current_cell[0] + 1;  // Down
    //                 col = current_cell[1] + 1;  // Right
    //                 break;
    //             }
    //             case 6:
    //             {
    //                 row = current_cell[0];      // Same
    //                 col = current_cell[1] + 1;  // Right
    //                 break;
    //             }
    //             case 7:
    //             {
    //                 row = current_cell[0] - 1;  // Up
    //                 col = current_cell[1] + 1;  // RIght
    //                 break;
    //             }
    //         }

    //         // Check if direction has obstacle
    //         if (g_map[row][col] != -1) 
    //         {
    //             // Check if direction has been visited, avoid looping
    //             if ( ! ((find(visited_rows.begin(), visited_rows.end(), row) != visited_rows.end()) && (find(visited_cols.begin(), visited_cols.end(), col) != visited_cols.end())) )
    //             {
    //                 // Check if direction has minimum distance to initial position
    //                 if (g_map[row][col] < min_dist) 
    //                 {
    //                     // Store the direction if minimum distance is confirmed
    //                     min_dist = g_map[row][col];
    //                     next_cell[0] = row;
    //                     next_cell[1] = col;
    //                 }
    //             }
    //         }

    //     }

    //     // Move to direction that has minimum distance
    //     current_cell[0] = next_cell[0];
    //     current_cell[1] = next_cell[1];

    //     // Store the index of the cell
    //     visited_rows.push_back(next_cell[0]);
    //     visited_cols.push_back(next_cell[1]);


    // }   

    // // Show the visited cells
    // cout << "Visited cell: " << endl;
    // for (auto x : visited_rows) cout << " " << x;
    // cout << endl;
    // for (auto x : visited_cols) cout << " " << x;
    // cout << "\n\n";

    // // Plot the path
    // for (auto i = 0; i< visited_rows.size() - 1;i++)
    // {
    //     map[visited_rows[i]][visited_cols[i]] = 4; //Random UNIQUE values to distinct with the rest
    // }
    
    // cout << "The path plotted: " << endl;
    // Display the map again.
    for (auto i = 0; i < rows; i++)
    {
        for (auto j = 0; j < cols; j++)
        {
            cout << map[i][j] << " ";
            index++;
        }
        cout << endl;
    }

    return 0;
}
/*
Read File:          https://www.bgsu.edu/arts-and-sciences/computer-science/cs-documentation/reading-data-from-files-using-c-plus-plus.html 
vector:             https://cplusplus.com/reference/vector/vector/ 
2D Vector:          https://www.journaldev.com/42023/2d-vectors-in-c-plus-plus 
Path Planning:      https://www.researchgate.net/publication/320674819_Mapping-Based_Navigation 
Delay:              https://stackoverflow.com/questions/158585/how-do-you-add-a-timed-delay-to-a-c-program
Random Gen:         https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution 
RRT & RRT*:         https://youtu.be/QR3U1dgc5RE 
                    https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree 
                    https://www.scitepress.org/Papers/2019/81188/81188.pdf
*/