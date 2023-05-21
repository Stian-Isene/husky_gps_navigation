import sys
from math import cos


from numpy import deg2rad, rad2deg
from shapely import Polygon, LineString, Point

import matplotlib.pyplot as plt
import numpy as np


class PathPlanner:

    def __init__(self, path_width: float, vertex_1: tuple, vertex_2: tuple,
                 bounding_vertices: list):

        # Input: Path width, the two vertices that corresponds to the base line
        # and the vertices that bounds the polygon as a list of tuples
        self.path_width = path_width
        self.vertex_1 = vertex_1
        self.vertex_2 = vertex_2
        self.bounding_vertices = bounding_vertices

        # Creating the polygon
        self.poly = Polygon(bounding_vertices)

        # Creating the path that will be exported. This is a list of tuples
        self.path = list()

    def get_path(self):

        # Gets the vector that corresponds to the point.
        # Distance along the x-axis is x2 - x1
        # Distance along the y-axis is y2 - y1
        self.delta_x = (self.vertex_2[0] - self.vertex_1[0])
        self.delta_y = (self.vertex_2[1] - self.vertex_1[1])
        self.vertex_vector = [self.delta_x, self.delta_y]

        # The slope of the vector is:
        #   Distance traveled along the y-axis
        # -------------divided by-----------------------
        #   Distance traveled along the x-axis
        self.slope = self.delta_y/self.delta_x

        # The intercept along the y axis.
        self.y_intercept = self.vertex_1[1] - self.slope * self.vertex_1[0]

        # Initializing the unit vectors of the x and y-axis respectively
        self.unit_vector_i = [1, 0]
        self.unit_vector_j = [0, 1]

        # Getting the angle between the x-plane and the baseplane that is vertex_vector
        self.theta = rad2deg(self.angle_between(self.vertex_vector, self.unit_vector_i))

        # The angle of the right-angled triangle created between parallel lines
        self.phi = 90 - self.theta

        # Getting the translation along the x- or y-axis for creating the parallel lines
        # We will create parallel lines along the baseline with a width of half the path width.
        self.translation = (self.path_width / 2) / cos(deg2rad(self.phi))

        # Creating the inner polygon. A negative number indicates shrinking
        self.poly_inner = self.poly.buffer((self.path_width*-1)/2)

        # Getting the max parameters the polygons
        self.minx_1, self.miny_1, self.maxx_1, self.maxy_1 = self.poly.bounds
        self.minx_2, self.miny_2, self.maxx_2, self.maxy_2 = self.poly_inner.bounds

        # Creating a polygon that represents the extreme boundaries of the area
        self.poly_outer_extremes = Polygon([(self.minx_1, self.miny_1), (self.minx_1, self.maxy_1), (self.maxx_1, self.maxy_1), (self.maxx_1, self.miny_1)])

        # For now, we are only concerned about horizontal movement.
        # Finding if we are going left to right, or right to left.

        self.left_to_right = bool

        if max(self.vertex_2[0], vertex_1[0]) == self.maxx_1:
            self.left_to_right = False
        else:
            self.left_to_right = True

        if self.phi > 0 and self.left_to_right:
            self.left_to_right_acute()
        elif self.phi > 0 and not(self.left_to_right):
            self.right_to_left_acute()
        elif self.phi < 0 and self.left_to_right:
            self.left_to_right_obuse()
        elif self.phi < 0 and not(self.left_to_right):
            self.right_to_left_obuse()



        # This is for vizualising what is happening
        # Plotting original area
        x, y = self.poly.exterior.xy
        plt.plot(x, y)
        # Plotting inner area
        x, y = self.poly_inner.exterior.xy
        plt.plot(x, y)
        # Plotting outer ekstremes
        x, y = self.poly_outer_extremes.exterior.xy
        plt.plot(x, y)
        # plotting lines
        #plt.plot(*self.line_base_line_intersection.xy)
        plt.plot(*self.line_intersection_outer_extreme.xy)
        # plt.plot(*line_paralell_horizontal_movement.xy)
        # Plotting the path
        plt.plot(*zip(*self.path))

        plt.show()

        #Write path to file
        with open('sweeping_path.txt', 'w') as file:
            for i in range(len(self.path)):
                file.write(f"{self.path[i][0]},{self.path[i][1]}\n")


    def left_to_right_acute(self):
        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(vertex_1[0] + (-100), self.y_intercept + self.slope * (vertex_1[0] + (-100))),
             (vertex_2[0] + 100, self.y_intercept + self.slope * (vertex_2[0] + 100))])
        #plt.plot(*self.line_vertex_elongated.xy)
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.x2_outer_extreme + self.translation, self.maxy_1),
                          (self.x1_outer_extreme + self.translation, self.miny_1)])

        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.line_intersection_outer_extreme.intersection(self.poly_inner).boundary.bounds

        # Saves to path
        self.path.append((self.x1_intersection_base_line, self.y1_intersection_base_line))
        self.path.append((self.x2_intersection_base_line, self.y2_intersection_base_line))

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])

        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.x2_outer_extreme + 2 * self.translation, self.maxy_1),
                        (self.x1_outer_extreme + 2 * self.translation, self.miny_1)])

        # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x2_intersection_base_line, self.y2_intersection_base_line),
                                (self.x2_horizontal_line, self.y2_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))

        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.x2_outer_extreme + i, self.maxy_1), (self.x1_outer_extreme + i, self.miny_1)])

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y2_up_down))
                self.path.append((x1_up_down, y1_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.x2_outer_extreme + i + self.translation, self.maxy_1),
                                (self.x1_outer_extreme + i + self.translation, self.miny_1)])

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y1_up_down),
                                       (x1_down_up, y1_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y1_down_up))
                    self.path.append((x2_down_up, y2_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.x2_outer_extreme + i + 2 * self.translation, self.maxy_1),
                                    (self.x1_outer_extreme + i + 2 * self.translation, self.miny_1)])

                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_down_up, y2_down_up),
                                       (x2_up_down_second, y2_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation

        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxx of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            x_value = poly_inner_exterior_points[0][i]
            if x_value == self.maxx_2:
                y_value = poly_inner_exterior_points[1][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


        else:

            # Creating the crossing points:
            crossing_points = [(x1_down_up, y1_down_up),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))

            for path in range(len(self.path)):
                print(path)


    def right_to_left_acute(self):

        self.translation = self.translation * -1

        # Creating a line along the base-line, with extra length in both directions
        self.line_vertex_elongated = LineString(
            [(vertex_1[0] + 100, self.y_intercept + self.slope * (vertex_1[0] + 100)),
             (vertex_2[0] - 100, self.y_intercept + self.slope * (vertex_2[0] - 100))])
        plt.plot(*self.line_vertex_elongated.xy)
        # Finds the intersection of the extreme values of the area
        self.x1_outer_extreme, self.y1_outer_extreme, self.x2_outer_extreme, self.y2_outer_extreme, = self.poly_outer_extremes.intersection(
            self.line_vertex_elongated).boundary.bounds

        # Creates a line that follows the base-line on the inner polygon with end points on the outer extremes
        self.line_intersection_outer_extreme \
            = LineString([(self.x2_outer_extreme + self.translation, self.maxy_1),
                          (self.x1_outer_extreme + self.translation, self.miny_1)])
        plt.plot(*self.line_intersection_outer_extreme.xy)
        # Finds the intersection of that line to the inner polygon.
        self.x1_intersection_base_line, self.y1_intersection_base_line, self.x2_intersection_base_line, self.y2_intersection_base_line, \
            = self.line_intersection_outer_extreme.intersection(self.poly_inner).boundary.bounds

        # Saves to path
        self.path.append((self.x1_intersection_base_line, self.y1_intersection_base_line))
        self.path.append((self.x2_intersection_base_line, self.y2_intersection_base_line))

        # This line is only for vizualization
        self.line_base_line_intersection = \
            LineString([(self.x1_intersection_base_line, self.y1_intersection_base_line),
                        (self.x2_intersection_base_line, self.y2_intersection_base_line)])

        # Creating a parallel line to the base-line
        # This is for creating the first horizontal movement
        self.line_paralell_horizontal_movement = \
            LineString([(self.x2_outer_extreme + 2 * self.translation, self.maxy_1),
                        (self.x1_outer_extreme + 2 * self.translation, self.miny_1)])

        # Gets the crossing points with the inner polygon
        self.x1_horizontal_line, self.y1_horizontal_line, self.x2_horizontal_line, self.y2_horizontal_line = \
            self.poly_inner.intersection(self.line_paralell_horizontal_movement).boundary.bounds

        # Defines the endpoints of the horizontal movement
        self.crossing_points = [(self.x2_intersection_base_line, self.y2_intersection_base_line),
                                (self.x2_horizontal_line, self.y2_horizontal_line)]
        # Gets the horizontal movement. This might 2 or 3 tuples in a list
        self.horizontal_path = \
            self.get_line_from_polygon_within_two_points(self.crossing_points, self.poly_inner)

        # The first path point is always the last used, so we don't need this point.
        self.horizontal_path.pop(0)

        # Save all the path points in the path
        for path in self.horizontal_path:
            self.path.append((path[0]))

        # Getting ready for the rest of the path

        # Creates a counter that holds the translation along the desired axis
        i = 2 * self.translation  # 2 times because we are not interested in the first

        # Creating the rest of the path.

        # Bool for continuing the path planner
        continue_planning = True

        while continue_planning:

            # Creating the line that runs up to down

            # This is done by taking the crossing point of the elongated line
            # with the extreme boundings and extending it by x*translation where
            # x = the line number
            line_up_down = \
                LineString([(self.x2_outer_extreme + i, self.maxy_1), (self.x1_outer_extreme + i, self.miny_1)])

            # Check if the line crosses the inner polygon. Then stop planning
            if not (line_up_down.intersects(self.poly_inner)):
                continue_planning = False
            else:
                # Continues planning.

                # This is for vizualising during testing
                # plt.plot(*line_up_down.xy)

                # Crossing points
                x1_up_down, y1_up_down, x2_up_down, y2_up_down, = \
                    self.poly_inner.intersection(line_up_down).boundary.bounds

                # Adds to the path
                self.path.append((x2_up_down, y2_up_down))
                self.path.append((x1_up_down, y1_up_down))

                # Creates the next line
                line_down_up = \
                    LineString([(self.x2_outer_extreme + i + self.translation, self.maxy_1),
                                (self.x1_outer_extreme + i + self.translation, self.miny_1)])

                # Check if the line crosses the inner polygon. Then stop planning
                if not (line_down_up.intersects(self.poly_inner)):
                    continue_planning = False
                else:

                    # Finds the crossing points of our lines
                    x1_down_up, y1_down_up, x2_down_up, y2_down_up, = \
                        self.poly_inner.intersection(line_down_up).boundary.bounds

                    # plt.plot(*line_down_up.xy)
                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x1_up_down, y1_up_down),
                                       (x1_down_up, y1_down_up)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    # Save the down up line in the path
                    self.path.append((x1_down_up, y1_down_up))
                    self.path.append((x2_down_up, y2_down_up))

                    # Create a second up_down line. This is only for getting horizontal
                    line_up_down_second = \
                        LineString([(self.x2_outer_extreme + i + 2 * self.translation, self.maxy_1),
                                    (self.x1_outer_extreme + i + 2 * self.translation, self.miny_1)])

                    # Gets the crossing point
                    x1_up_down_second, y1_up_down_second, x2_up_down_second, y2_up_down_second = \
                        self.poly_inner.intersection(line_up_down_second).boundary.bounds

                    # Defines the endpoints of the horizontal movement
                    crossing_points = [(x2_down_up, y2_down_up),
                                       (x2_up_down_second, y2_up_down_second)]

                    # Gets the horizontal movement. This might 2 or 3 tuples in a list
                    horizontal_path = \
                        self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

                    # The first path point is always the last used, so we don't need this point.
                    horizontal_path.pop(0)

                    # Save all the path points in the path
                    for path in horizontal_path:
                        self.path.append((path[0]))

                    i += 2 * self.translation

        # Creating the last horizontal movement.
        # Getting the coordinates of the outermost point of the inner polygon
        poly_inner_exterior_points = self.poly_inner.exterior.xy

        # Finds the outermost point by using maxx of the inner polygon
        for i in range(len(poly_inner_exterior_points[0])):
            x_value = poly_inner_exterior_points[0][i]
            if x_value == self.minx_2:
                y_value = poly_inner_exterior_points[1][i]
                poly_inner_end_point = (x_value, y_value)

                x_check_value = x_value

        # Checking which line is the last line created
        if x1_up_down > x1_down_up:

            # Creating the crossing points:
            crossing_points = [(x1_up_down, y1_up_down),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))



        else:

            # Creating the crossing points:
            crossing_points = [(x1_down_up, y1_down_up),
                               (poly_inner_end_point[0], poly_inner_end_point[1])]

            # Gets the horizontal movement. This might 2 or 3 tuples in a list
            horizontal_path = \
                self.get_line_from_polygon_within_two_points(crossing_points, self.poly_inner)

            # The first path point is always the last used, so we don't need this point.
            horizontal_path.pop(0)

            # Save all the path points in the path
            for path in horizontal_path:
                self.path.append((path[0]))


    def left_to_right_obuse(self):
        pass

    def right_to_left_obuse(self):
        pass
    def bottom_to_top_acute(self):
        pass

    def top_to_bottom_acute(self):
        pass

    def bottom_to_top_obuse(self):
        pass

    def top_to_bottom_obuse(self):
        pass



    def unit_vector(self, vector):
        # Takes a list with two floats as input
        # Returns the unit vector those floats represents
        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        # Returns the angle between the vectors v1 and v2 in radians
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        # clip limits the unit vectors: -1 to 1
        # arccos is the arcosine of the vectors. Tha angle between them
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def get_sides_from_polygon(self, poly: Polygon):
        # Input: Shapely Polygon
        # Output: All sides as a list of Shapely LineSting
        sides = []

        # Gets the points in XY coord that bounds the polygon
        x, y = poly.exterior.xy

        #T akes the bounding of the polygon and makes new lines.
        for i in range(len(x) - 1):
            sides.append(LineString([(x[i], y[i]), (x[i + 1], y[i + 1])]))

        return sides

    def get_closest_side(self, point: Point, sides: list):
        # Input: Shapely Point and a list of LineString
        # Output: LineString that is closest to the side

        closest_side = None

        # Initializes an infinitely far away point as starting point
        distance_closest_side = float('inf')

        # Checks all individual sides to get the closest one
        # Shapely LineString has a function to check the closest distance from a Point
        for side in sides:
            if side.distance(point) < distance_closest_side:
                closest_side = side
                distance_closest_side = side.distance(point)

        return closest_side

    def get_line_from_polygon_within_two_points(self, points: list, poly: Polygon):
        # Get the line connecting two points on a polygon.
        # Input: list of tuples of two cartesian points, Shapely Polygon
        # Output:

        # Creating the path to be returned
        path_ = list()
        # Creating the points
        # Verbose for readability
        x1 = points[0][0]
        y1 = points[0][1]
        x2 = points[1][0]
        y2 = points[1][1]

        point_1 = Point(x1, y1)
        point_2 = Point(x2, y2)

        # initializes empty LineString that will be filled
        side1 = LineString([])
        side2 = LineString([])

        # Get all the individual sides from the polygon
        sides = self.get_sides_from_polygon(poly)

        # Get the sides of the polygon the points are on
        # Here they can both be the same side. This is fine
        for side in sides:
            if side.distance(point_1)<1e-8:
                side1 = side
            if side.distance(point_2)<1e-8:
                side2 = side

        # Check if it is the same side
        # If it is the same side, return only that side
        if side1.equals(side2):
            path_.append([(x1, y1)])
            path_.append([(x2, y2)])
        else:
            if side1.intersects(side2):
                intersection = side1.intersection(side2)
                x = intersection.x
                y = intersection.y

                path_.append([(x1, y1)])
                path_.append([(x, y)])
                path_.append([(x2, y2)])
            else:
                path_.append([(x1, y1)])
                path_.append([(x2, y2)])

        return path_


if __name__ == "__main__":

    test_width = 2
    #vertices = [(2., 0.),(10., 0.5), (25., 3.),  (30., 7.), (10., 10), (3., 7.)]
    vertices = [(423236.3895171815 , 6769335.20677714), (423233.38284477545, 6769328.5425624)
                , (423239.71184318454, 6769326.379024858), (423243.20974025177 , 6769332.30897843)]
    vertex_1 = (423233.38284477545, 6769328.5425624)
    vertex_2 = (423236.3895171815 , 6769335.20677714)
    #poly = Polygon(vertices)
    #x, y, = poly.exterior.xy
    #plt.plot(x, y)
    path_planner = PathPlanner(test_width, vertex_1, vertex_2, vertices)

    path_planner.get_path()
    plt.show()