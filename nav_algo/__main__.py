import nav_algo.navigation as nav
import nav_algo.configuration as config

def main():
    """Create and run and instance of the navigation algorithm.

    The configuration of the current context (e.g. are the sensors real or 
    fake, what event is being run, print output to radio?, etc.) is read from
    the configuration file. Instructions on the format of this file can be found
    in nav_algo/config/readme.txt.

    The waypoints are read from a csv file. It should have one waypoint per line
    in the format: latitude, longitude. See test.csv for an example.

    How to set the waypoints for the different event algorithms:
    - Precision navigation: set the waypoints as the positions of the 4 buoys
    in the order [top left, top right, bottom left, bottom right]
    - Station keeping: set the waypoints as the positions of the 4 buoys in the
    order [north west, north east, south east, south west]
    """
    waypoint_file = 'nav_algo/waypoints/test.csv'
    configuration_file = 'nav_algo/config/real_sensors_no_event.json'

    # Read the configuration and waypoint files and setup the current context
    configuration = config.NavigationConfiguration(config_filename=configuration_file, 
                                                   waypoint_filename=waypoint_file)

    # Start the navigation algorithm given the current configuration
    nav_controller = nav.NavigationController(configuration=configuration)


if __name__ == "__main__":
    main()
