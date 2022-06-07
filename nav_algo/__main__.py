import nav_algo.navigation as nav
import nav_algo.events as events


def readWaypoints(filename):
    waypoints = []
    with open(filename, 'r') as f:
        for line in f:
            l = line.split(",")
            waypoints.append((float(l[0]), float(l[1])))
    return waypoints


def main():
    """TODO fill this out with more instructions.

    The waypoints are read from a csv file. It should have one waypoint per line
    in the format: latitude, longitude. See test.csv for an example.

    How to set the waypoints for the different event algorithms:
    - Precision navigation: set the waypoints as the positions of the 4 buoys
    in the order [top left, top right, bottom left, bottom right]
    """
    # waypoints is an array of (lat, long) tuples
    waypoint_file = 'nav_algo/waypoints/test.csv'
    waypoints = readWaypoints(waypoint_file)

    nav_controller = nav.NavigationController(waypoints=waypoints)


if __name__ == "__main__":
    main()
