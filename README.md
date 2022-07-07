# Path Planning Examples
Examples of Path Planning

## Map Formatting.
* RGB to Greyscale (need OpenCV): use `map-formats/rgb_to_grey.cpp`
* Text map: GIMP -> Open image -> Export as `.pgm` -> Open exported `.pgm` in an Text Editor -> Modify if neccessary -> save as text.

## Grid-based Path Planning
* **Dijkstra, 4 directions**, fixed 100x100 text map, point object, no error margin: `grid-based/grid_dijkstra_4dir_1.cpp`.
  ```sh
  Basic Dijkstra algorithm:
  1. From Grid Map, create g(n) Map where each cell on g(n) is distance from that cell to Initial Position.
  2. Start from Goal/Initial node, move gradually to Initial node by moving to adjacent cell with smallest cost.
  ```
* **Dijkstra, 8 directions**, fixed 100x100 text map, point object, no error margin: `grid-based/grid_dijkstra_8dir_1.cpp`.
  ```sh
  change int direction_num to 8
  Add more case to switch case
  ```
* **A-star, 8 directions**, fixed 100x100 text map, point object, no error margin: `grid-based/grid_a_star_8dir_1.cpp`.
  ```sh
  Basic A-star algorithm:
  1. From Grid Map, create f(n) = g(n) + factor_h*h(n) Map where each cell on g(n) is distance from that cell to Initial Position and each cell on h(n) is heuristic value to the Goal
  2. Start from Goal/Initial node, move gradually to Initial node by moving to adjacent cell with smallest cost.
  ```
* **RRT, 8 directions**, fixed 100x100 text map, point object, no error margin: `grid-based/grid_rrt_8dir_1.cpp`.
  ```sh
  Basic RRT algorithm:
  1. For each vertice on the RRT Tree, randomly pick a FREE/NEW node on map within a [MAXIMUM DISTANCE] 
  2. Connect it to nearest node on the tree.
    -> Obstacle in between them? -> YES -> Ignore
                                 -> NO -> Add
  3. Stop at goal.
  * Potentiall less node than A* since the nodes are spaced out (within a [MAXIMUM DISTANCE]).
  ```
  In complete, to do: (1) Revise the *condition* that check for obstacles between cell on RRT tree with new cell (2) Try with much *smaller scale map*.

## Comments
These are all speculative insights of the author while implementing the process, not from any source and definitely not definite (jk :)), it's mean they can be very wrong):
 * On grid-based methods, *Dijkstra, A-star, Potential Field, FFM* have a major common which is assigning **artificial value** to each cell, the differences are in the way those artificial values and path cost are computed.* RRT tree* method is different in this regard because it creates an entirely new type of map (which is the RRT tree) from current grid map.
