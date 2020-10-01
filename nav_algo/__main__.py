import nav_algo.navigation as nav


def main():
    # waypoints is an array of (lat, long) tuples
    waypoints = [(42.444241, 76.481933), (42.446016, 76.484713)]  # test values

    nav_controller = nav.NavigationController(simulation=True)


if __name__ == "__main__":
    main()