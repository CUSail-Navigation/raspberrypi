#from navigation_helper import *
from nav_algo.navigation import *
# TODO: figure out what else to import

def search(NavigationController):
    NavigationController.camera = Camera()
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

        NavigationController.configuration.waypoints = searchHelper([NavigationController.current_waypoint])
        NavigationController.current_waypoint = NavigationController.configuration.waypoints.pop(0)

        # Navigate between the seed waypoints until we see the buoy
        buoy_loc = NavigationController.navigate(use_camera=True)

    # Go to the buoy location
    coord_sys = NavigationController.configuration.boat.sensors.coordinate_system
    buoy_loc = coord.Vector.convertXYToLatLong(coord_sys, buoy_loc.x,
                                                buoy_loc.y)
    NavigationController.current_waypoint = buoy_loc
    NavigationController.configuration.waypoints = []
    NavigationController.DETECTION_RADIUS = 1.0
    NavigationController.navigate(use_camera=False)

    # Signal that we've found the buoy
    # TODO do something more interesting
    NavigationController.configuration.write_output("FOUND_BUOY")
    NavigationController.configuration.write_output("BUOY LAT: {} LONG: {}".format(
        buoy_loc.latitude, buoy_loc.longitude))

    # Station Keeping Mode
    while True:
        NavigationController.current_waypoint = buoy_loc
        NavigationController.navigate(use_camera=False)

def fcn(x, radius):
    """
    Creates path for sailboat and then rotates this path based on the wind 
    angle before the boat enters the circle
    """
    period = 1
    return math.sqrt((radius**2)-(x**2))*math.sin(period*x*(math.pi))



