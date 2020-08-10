import nav_algo.navigation as nav


def main():
    # waypoints is an array of (lat, long) tuples
    waypoints = []

    nav_controller = nav.NavigationController(waypoints)
    nav_controller.navigate()  # start navigation


if __name__ == "__main__":
    main()