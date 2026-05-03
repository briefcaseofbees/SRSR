import numpy as np
# import matplotlib.pyplot as plt
import math

# numpy settings for printing
np.set_printoptions(threshold=np.inf)
np.set_printoptions(linewidth=200)

## OCCUPANCY-GRID DEFAULT SETTINGS
_RESOLUTION = 10            # the edge length (in cm) of squares in the map-grid
_STARTING_AREA = 20         # #/Squares in all directions from origin... == (_STARTING_AREA + 1) ** 2 centered on origin
_EXPANSION_FACTOR = 2       # factor to expand when grid map edge is reached (_CURRENT_AREA * _EXPANSION_FACTOR + 1)

# confidence for square occupancy (in float) will be in range [0.0, 0.1, 0.2, ..., 0.9, 1.0] (inclusive)
_CONFIDENCE_LOWER_BOUND = 0     # lower bound (FREE) for confidence of grid square occupancy (0.0 in float)
_CONFIDENCE_DEFAULT = 5         # default value (UNKNOWN) for confidence of grid square occupancy (0.5 in float)
_CONFIDENCE_UPPER_BOUND = 10    # upper bound (OCCUPIED) for confidence of grid square occupancy (1.0 in float)
_CONF_DATATYPE = np.uint8       # we don't need a large datatype due to the range of confidence values...
                                # {uint8(0) === float(0.0); uint8(1) === float(0.1); ...; uint8(10) === float(1.0)}

_CONE_ANGLE = 15                # angle (in degrees) for the radius of the cone

class OccupancyGrid:
    def __init__(self, resolution=_RESOLUTION,
                 starting_edge_dim=_STARTING_AREA,
                 default_confidence=_CONFIDENCE_DEFAULT,
                 confidence_dtype=_CONF_DATATYPE):

        self.grid_square_resolution = resolution
        self.current_edge_dim = 2 * starting_edge_dim + 1
        self.unknown_square_def = default_confidence
        self.confidence_dtype = confidence_dtype
        self.origin_square = [self.current_edge_dim//2, self.current_edge_dim//2]

        self.occupancy_grid = np.ones((self.current_edge_dim,
                                       self.current_edge_dim),
                                      dtype=self.confidence_dtype) * 5
    """
        def visualize_grid(self):
            # Create a figure
            plt.figure(figsize=(10, 10))

            # Display grid as image
            # 0 = white (free), 5 = gray (unknown), 10 = black (occupied)
            plt.imshow(self.occupancy_grid, cmap='gray_r', vmin=0, vmax=10,

                    extent=(-self.current_edge_dim*5,
                            self.current_edge_dim*5,
                            self.current_edge_dim*5,     # flipped y-axis (corresponds to south-bound dimension)
                            -self.current_edge_dim*5)    # flipped y-axis (corresponds to north-bound dimension)
                    )

            # Add colorbar to show scale
            plt.colorbar(label='Confidence (0=free, 10=occupied)')

            # Add grid lines
            plt.grid(True, which='both', color='red', linewidth=0.5, alpha=0.3)

            # Labels
            plt.title('Occupancy Grid')
            plt.xlabel('Grid X')
            plt.ylabel('Grid Y')

            # Show
            plt.show()
    """

    def update_square_confidences(self, distance_reading = 50, current_position = [0,0], current_heading_degs:float = 0):

        # store robot's real coordinates
        current_pos_y = -current_position[0]
        current_pos_x = current_position[1]
        current_pos = [current_pos_y, current_pos_x]

        print(f"\t\tlooking at a {current_heading_degs} angle")

        # convert robot's heading to radians for later calculations
        current_heading_rads = math.radians(current_heading_degs)

        # calculate robot's position in og using origin as anchor
        current_pos_og_y = self.origin_square[0] + int(current_pos[0] / self.grid_square_resolution)
        current_pos_og_x = self.origin_square[1] + int(current_pos[1] / self.grid_square_resolution)

        self.occupancy_grid[current_pos_og_y, current_pos_og_x] = 88  # we know the position of jetbot is unoccupied

        # calculate real coordinates of focus square assuming [0.0,0.0] coordinate (round for simplicity)
        focus_square_y = round((distance_reading * math.cos(current_heading_rads)),2)
        focus_square_x = round((distance_reading * math.sin(current_heading_rads)),2)

        # determine focus square og offset from an assumed [0,0] location (absolute for negative values)
        focus_square_og_y_offset = abs(focus_square_y) // self.grid_square_resolution
        focus_square_og_x_offset = abs(focus_square_x) // self.grid_square_resolution

        # correct signs from previous calculation based on real coordinate sign values
        if focus_square_y < 0:
            focus_square_og_y_offset = -focus_square_og_y_offset
        if focus_square_x < 0:
            focus_square_og_x_offset = -focus_square_og_x_offset

        # determine the projected location of the focus square on og
        projected_focus_square_og_y = current_pos_og_y + focus_square_og_y_offset
        projected_focus_square_og_x = current_pos_og_x - focus_square_og_x_offset

        # expand occupancy grid if it goes off top, or left side of the OG
        while ((projected_focus_square_og_y < self.grid_square_resolution)
                or (projected_focus_square_og_x < self.grid_square_resolution)):
            self.expand_grid(2)

            # recalculate the current robot's position based off of expanded occupancy grid
            current_pos_og_y = self.origin_square[0] + int(current_pos_y / self.grid_square_resolution)
            current_pos_og_x = self.origin_square[1] + int(current_pos_x / self.grid_square_resolution)

            projected_focus_square_og_y = current_pos_og_y - focus_square_og_y_offset
            projected_focus_square_og_x = current_pos_og_x + focus_square_og_x_offset

        # expand occupancy grid if it goes off bottom, or right side of the OG
        while ((projected_focus_square_og_y >= self.current_edge_dim) or
            (projected_focus_square_og_x >= self.current_edge_dim)):
            self.expand_grid(2)

            # recalculate the current robot's position based off of expanded occupancy grid
            current_pos_og_y = self.origin_square[0] + int(current_pos_y / self.grid_square_resolution)
            current_pos_og_x = self.origin_square[1] + int(current_pos_x / self.grid_square_resolution)

            projected_focus_square_og_y = current_pos_og_y - focus_square_og_y_offset
            projected_focus_square_og_x = current_pos_og_x + focus_square_og_x_offset

        # identify where the focus square is on the grid...
        focus_square_og_location = [int(current_pos_og_y - focus_square_og_y_offset),
                                    int(current_pos_og_x + focus_square_og_x_offset)]

        print(f"focus square og position [y,x] = {focus_square_og_location}")
        print(f"focus square real position [y,x] = [{-focus_square_y}, {focus_square_x}]")
        print("--" * 20)

        # mark all squares in a rectangle OUT from the jetbot as unoccupied up to the focus_square_og_location

        # use make offset length the hypotenuse of a square with side-length equal to grid resolution (rounded up)
        offset_length = math.ceil(math.sqrt(self.grid_square_resolution**2 + self.grid_square_resolution**2))
        print(f"offset length = {offset_length}")

        # find perpendicular angle so we can find the coordinates of the offsets on either side of current position
        perpendicular_angle = current_heading_degs + 90
        perpendicular_angle_rads = math.radians(perpendicular_angle)
        print(f"rotated angle = {perpendicular_angle}")

        # calculate the left, and right offsets off of the current position
        l_offset_y = -round(-offset_length * math.cos(perpendicular_angle_rads), 1)
        l_offset_x = round(-offset_length * math.sin(perpendicular_angle_rads), 1)
        l_offset_coords = [l_offset_y, l_offset_x]

        r_offset_y = -round(offset_length * math.cos(perpendicular_angle_rads),1)
        r_offset_x = round(offset_length * math.sin(perpendicular_angle_rads),1)
        r_offset_coords = [r_offset_y, r_offset_x]

        print(f"current position = {current_pos}")

        print(f"left offset coords [y,x] = {l_offset_coords}")
        print(f"right offset coords [y,x] = {r_offset_coords}")


        # create dot-map to put in front of jetbot, with distances on x,y between dots being some delta
        delta = 1

        # need to create rectangular bands in front of jetbot to check squares for
        l_offset_band_coordinates = list()
        r_offset_band_coordinates = list()

        delta = 20

        l_offset_endpt = [round(l_offset_coords[0] - distance_reading * math.cos(current_heading_rads), 1),
                          round(l_offset_coords[1] + distance_reading * math.sin(current_heading_rads), 1)]

        r_offset_endpt = [round(r_offset_coords[0] - distance_reading * math.cos(current_heading_rads), 1),
                                 round(r_offset_coords[1] + distance_reading * math.sin(current_heading_rads), 1)]


        print(f"left_offset_endpt [y,x] = {l_offset_endpt}")
        print(f"right_offset_endpt [y,x] = {r_offset_endpt}")

        print("--" * 20)

        delta = 25  # constant for band's dot-map resolution
        band_step_y_x = [-(delta / self.grid_square_resolution) * math.cos(current_heading_rads),
                         (delta / self.grid_square_resolution) * math.sin(current_heading_rads)]

        print(f"band_step [y,x] = {band_step_y_x}")
        # add starting left offset and right offset coordinates
        l_offset_band_coordinates.append(l_offset_coords)
        r_offset_band_coordinates.append(r_offset_coords)

        distance_covered = 0
        # populate left band coordinates stepping it by the band step deltas for x and y
        while True:
            l_band_coord_next_step = [round(l_offset_band_coordinates[-1][0] + band_step_y_x[0],2),
                                         round(l_offset_band_coordinates[-1][1] + band_step_y_x[1],2)]

            distance_covered += math.sqrt((l_offset_band_coordinates[-1][0] - l_band_coord_next_step[0]) ** 2 +
                                          (l_offset_band_coordinates[-1][1] - l_band_coord_next_step[1]) ** 2)

            if distance_covered >= distance_reading:
                break

            l_offset_band_coordinates.append(l_band_coord_next_step)

        if l_offset_endpt not in l_offset_band_coordinates:
            l_offset_band_coordinates.append(l_offset_endpt)

        print(f"l_offset_band_coordinates [y,x] = {l_offset_band_coordinates}")

        distance_covered = 0
        # populate right band coordinates stepping it by the band step deltas for x and y
        while True:
            r_band_coord_next_step = [round(r_offset_band_coordinates[-1][0] + band_step_y_x[0], 2),
                                      round(r_offset_band_coordinates[-1][1] + band_step_y_x[1], 2)]

            distance_covered += math.sqrt((r_offset_band_coordinates[-1][0] - r_band_coord_next_step[0]) ** 2 +
                                          (r_offset_band_coordinates[-1][1] - r_band_coord_next_step[1]) ** 2)

            if distance_covered >= distance_reading:
                break

            r_offset_band_coordinates.append(r_band_coord_next_step)

        if r_offset_endpt not in r_offset_band_coordinates:
            r_offset_band_coordinates.append(r_offset_endpt)

        print(f"r_offset_band_coordinates [y,x] = {r_offset_band_coordinates}")

        l_band_length = len(l_offset_band_coordinates)
        r_band_length = len(r_offset_band_coordinates)

        print(f"length of left band = {len(r_offset_band_coordinates)}; length of right band = {len(l_offset_band_coordinates)}")

        if l_band_length != r_band_length:  # bands need to be the same length...
            raise ValueError("This should not happen!!!")

        band_range = []

        # need to find affected squares between left and right bands
        for band_index in range(l_band_length):  # bands are same length, so can just use length of one
            # for given coordinate: translates to spot on OG

            # left-most square
            left_square_y = int(l_offset_band_coordinates[band_index][0] / self.grid_square_resolution)
            left_square_x = int(l_offset_band_coordinates[band_index][1] / self.grid_square_resolution)

            # right-most square
            right_square_y = int(r_offset_band_coordinates[band_index][0] / self.grid_square_resolution)
            right_square_x = int(r_offset_band_coordinates[band_index][1] / self.grid_square_resolution)

            band_range.append([left_square_y, left_square_x])

            lower_y = left_square_y if left_square_y < right_square_y else right_square_y
            upper_y = left_square_y if left_square_y > right_square_y else right_square_y

            lower_x = left_square_x if left_square_x < right_square_x else right_square_x
            upper_x = left_square_x if left_square_x > right_square_x else right_square_x

            for y in range(lower_y, upper_y+1):
                for x in range(lower_x, upper_x+1):
                    band_range.append([y, x])

            band_range.append([right_square_y, right_square_x])

        for square in band_range:
            if self.occupancy_grid[square[0] + current_pos_og_y, square[1] + current_pos_og_x] != 10:
                self.occupancy_grid[square[0] + current_pos_og_y, square[1] + current_pos_og_x] = 0

        self.occupancy_grid[focus_square_og_location[0], focus_square_og_location[1]] = 10

        # fill in target squares using blob method

        print(f"focus_square_og_location [y,x] = {focus_square_og_location}")
        print(f"distance away = {distance_reading}")

        '''
        area AROUND focus square will be colored based on distance away, and angle of looking at the square...
        
        focus_square_tl = [y-1,x-1]             focus_square_t = [y-1,x]        focus_square_tr = [y-1,x+1]
        
        focus_square_l = [y,x-1]                focus_square_cen = [y,x]        focus_square_r = [y,x+1]
        
        focus_square_bl = [y+1,x-1]             focus_square_b = [y+1,x]        focus_square_br = [y+1,x+1]
        
        
        '''

        self.occupancy_grid[current_pos_og_y, current_pos_og_x] = 88  # mark off jetbot for debugging purposes

    def expand_grid(self, expansion_factor:int):

        # argument verification: MUST BE INTEGER
        if not isinstance(expansion_factor, int):
            raise TypeError("Expansion factor must be an integer")

        # argument verification: MUST BE POSITIVE AND NON-ZERO
        if expansion_factor <= 1:
            raise ValueError("Expansion factor must be positive and greater than one")

        # check if expansion factor is odd or even; calculate new dimension for new grid while maintaining oddness
        if expansion_factor % 2 == 0:
            new_dim = self.current_edge_dim * expansion_factor + 1
        else:
            new_dim = self.current_edge_dim * expansion_factor

        expanded_grid = np.ones((new_dim, new_dim), dtype=self.confidence_dtype) * 5  # create new grid with 5s

        # calculate offset of where to put old occupancy grid in new grid (must be centered on origin point of JetBot)
        offset_x = (new_dim - self.current_edge_dim) // 2
        offset_y = (new_dim - self.current_edge_dim) // 2

        # insert old grid into new expanded one using calculated offsets
        expanded_grid[offset_y:offset_y+self.current_edge_dim,
                        offset_x:offset_x+self.current_edge_dim] = self.occupancy_grid

        # update current edge dimension, and occupancy grid
        self.current_edge_dim, self.occupancy_grid = new_dim, expanded_grid
        # update origin square in relation to the expanded grid
        self.origin_square = [self.current_edge_dim//2, self.current_edge_dim//2]


# EOF