def search(self):
    self.camera = Camera()
    search_radius = 80
    num_seeds = 15

    # Assuming the input waypoint (origin) is the middle of the search field
    # Seed a bunch of random waypoints throughtout the environment
    buoy_loc = None
    while buoy_loc is None:
        # waypoints = []
        # for _ in range(num_seeds):
        #     angle = 2 * np.pi * np.random.rand()
        #     radius = np.random.rand() * search_radius
        #     x = radius * np.cos(angle)
        #     y = radius * np.sin(angle)
        #     w = coord.Vector(x=x, y=y)
        #     waypoints.append(w)

        self.configuration.waypoints = search([self.current_waypoint])
        self.current_waypoint = self.configuration.waypoints.pop(0)

        # Navigate between the seed waypoints until we see the buoy
        buoy_loc = self.navigate(use_camera=True)

    # Go to the buoy location
    coord_sys = self.configuration.boat.sensors.coordinate_system
    buoy_loc = coord.Vector.convertXYToLatLong(coord_sys, buoy_loc.x,
                                                buoy_loc.y)
    self.current_waypoint = buoy_loc
    self.configuration.waypoints = []
    self.DETECTION_RADIUS = 1.0
    self.navigate(use_camera=False)

    # Signal that we've found the buoy
    # TODO do something more interesting
    self.configuration.write_output("FOUND_BUOY")
    self.configuration.write_output("BUOY LAT: {} LONG: {}".format(
        buoy_loc.latitude, buoy_loc.longitude))

    # Station Keeping Mode
    while True:
        self.current_waypoint = buoy_loc
        self.navigate(use_camera=False)

def search(waypoints):
    """
    Requires: waypoints[0] must be the center point of the circle. Returns 
    waypoints for the search path depending on the initial wind direction. 
    """
    search_waypoints = []
    rotated_waypoints = []
    center_point = waypoints[0]
    radius = 100
    # Generating waypoints
    x = center_point.x - 100
    for i in range(-radius, radius, 1):
        y = center_point.y + fcn(x, 100)
        new_waypoint = coord.Vector(x, y)
        search_waypoints.insert(len(search_waypoints), new_waypoint)
        x += 1
    # Rotating waypoints based on initial wind direction
    theta = sensors.readWindDirection()
    for point in search_waypoints:
        rotatedx = point.x*math.cos(theta) - point.y*math.sin(theta)
        rotatedy = point.x*math.sin(theta) + point.y*math.cos(theta)
        new_rotated_waypoint = coord.Vector(rotatedx, rotatedy)
        rotated_waypoints.insert(len(rotated_waypoints), new_rotated_waypoint)
    return rotated_waypoints


