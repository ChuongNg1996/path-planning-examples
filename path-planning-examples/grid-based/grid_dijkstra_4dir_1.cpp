/*
Grid-based, Dijkstra, 4-direction movement.

Basic algorithm:
1. From grid map, create g(n) map where each cell on g(n) is distance from that cell to initial position.
2. Start from goal node, move gradually to initial node by moving to adjacent cell with smallest cost.
*/


#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
using namespace std;

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
    int next_cell[2] = {-1,-1};                 // index of next cell (in row, col)
    vector<vector<int>> g_map;                  // store g(n) values
    int dist;                                   // distance from init_cell to a free cell
    int min_dist = 0;                           // minimum distance of a cell to init_cell
    int max_dist = 0;                           // maximum distance of a cell to init_cell
    int direction_num = 4;                      // Number of moving direction
    vector<int> visited_rows;                   // Store visited rows
    vector<int> visited_cols;                   // Store visited columns
    
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

    // Calculating g(n) map
    g_map = map;
    for (auto i = 0; i < rows; i++)
    {
        for (auto j = 0; j < cols; j++)
        {
            if (g_map[i][j] != 1) 
            {
                dist = abs(i-init_cell[0]) + abs(j-init_cell[1]);
                g_map[i][j] = dist;
                if (dist > max_dist) max_dist = dist; // Find the maximum distance
            }
            else g_map[i][j] = -1; //Transfrom occupied zone from 1 to -1 
        }

    }

    // ---------------------------------------------------------- //
    // ---------           Path Planning Algorithm      --------- //
    // ---------------------------------------------------------- //

    // Assign current cell as goal cell, make a reverse path to initial cell
    current_cell[0] = goal_cell[0];
    current_cell[1] = goal_cell[1];

    // While current cell doesn't arrive at initial cell
    while ((current_cell[0] != init_cell[0]) || (current_cell[1] != init_cell[1]))
    {
        min_dist = max_dist + 1; // Initialize minimum distance as maximum distance

        for (int i = 0; i < direction_num; i++)
        {
            switch(i)
            {
                // each case is a direction
                case 0:
                {
                    row = current_cell[0] - 1;
                    col = current_cell[1];
                    break;
                }
                case 1:
                {
                    row = current_cell[0] + 1;
                    col = current_cell[1];
                    break;
                }
                case 2:
                {
                    row = current_cell[0];
                    col = current_cell[1] - 1;
                    break;
                }
                case 3:
                {
                    row = current_cell[0];
                    col = current_cell[1] + 1;
                    break;
                }
            }

            // Check if direction has obstacle
            if (g_map[row][col] != -1) 
            {
                // Check if direction has been visited, avoid looping
                if ( ! ((find(visited_rows.begin(), visited_rows.end(), row) != visited_rows.end()) && (find(visited_cols.begin(), visited_cols.end(), col) != visited_cols.end())) )
                {
                    // Check if direction has minimum distance to initial position
                    if (g_map[row][col] < min_dist) 
                    {
                        // Store the direction if minimum distance is confirmed
                        min_dist = g_map[row][col];
                        next_cell[0] = row;
                        next_cell[1] = col;
                    }
                }
            }

        }

        // Move to direction that has minimum distance
        current_cell[0] = next_cell[0];
        current_cell[1] = next_cell[1];

        // Store the index of the cell
        visited_rows.push_back(next_cell[0]);
        visited_cols.push_back(next_cell[1]);


    }   

    // Show the visited cells
    cout << "Visited cell: " << endl;
    for (auto x : visited_rows) cout << " " << x;
    cout << endl;
    for (auto x : visited_cols) cout << " " << x;
    cout << "\n\n";

    // Plot the path
    for (auto i = 0; i< visited_rows.size() - 1;i++)
    {
        map[visited_rows[i]][visited_cols[i]] = 4; //Random UNIQUE values to distinct with the rest
    }
    
    cout << "The path plotted: " << endl;
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
*/
