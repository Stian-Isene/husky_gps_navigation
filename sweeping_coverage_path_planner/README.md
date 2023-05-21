# sweeping_coverage_path_planner
The sweeping coverage path planner is used to generate a path for a mobile robot to take. This path is intended to completely cover an area, as inputted by the user. It has been developed with the intent of using the Rotating Calipers method to generate antipodal pairs that it uses for generation of the covering path.

Installation: This pack relies on following libraries: Shapely, matplotlib and numpy. These can be installed with PIP or other python library managers

Usage:
To generate a path, all that is needed is in __main__. 

width specifies the width parameters of the path. This is the width your robot covers when moving through an area.

Vertices is the corners of your area.

Vertex_1 and vertex_2 is the corners that defines the bottomline. One of the antipodal pairs.

After running the program, a plot of the path and the polygon defining the area will pop up and a file with the name "sweeping_path.txt" with the path defined will be made.
