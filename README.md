# Path Planning Examples
Examples of Path Planning

## Map Formatting.
* RGB to Greyscale (need OpenCV): use `map-formats/rgb_to_grey.cpp`
* Text map: GIMP -> Open image -> Export as `.pgm` -> Open exported `.pgm` in an Text Editor -> Modify if neccessary -> save as text.

## Grid-based Path Planning
* Dijkstra, 4 direction, fixed 100x100 text map, point object, no error margin: `grid-based/grid_dijkstra_4dir_1.cpp`.
  ```sh
  Basic algorithm:
  1. From Grid Map, create g(n) Map where each cell on g(n) is distance from that cell to Initial Position.
  2. Start from Goal node, move gradually to Initial node by moving to adjacent cell with smallest cost.
  ```
* Dijkstra, 8 direction, fixed 100x100 text map, point object, no error margin: `grid-based/grid_dijkstra_8dir_1.cpp`.

