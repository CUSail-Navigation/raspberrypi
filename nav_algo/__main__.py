import nav_algo.navigation as nav
import nav_algo.events as events


def main():
    # waypoints is an array of (lat, long) tuples
    waypoints = [(42.444241, 76.481933), (42.446016, 76.484713)]  # test values

    nav_controller = nav.NavigationController(waypoints=waypoints)


if __name__ == "__main__":
    main()