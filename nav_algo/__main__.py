import nav_algo.navigation as nav
import nav_algo.events as events


def main():
    # waypoints is an array of (lat, long) tuples
    waypoints = [(42.04690135,-76.5024614),(42.4688799,-76.5028114)]  # test values

    nav_controller = nav.NavigationController(waypoints=waypoints,event=None)


if __name__ == "__main__":
    main()
