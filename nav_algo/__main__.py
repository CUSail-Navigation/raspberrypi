import nav_algo.navigation as nav
import nav_algo.events as events


def main():
    # waypoints is an array of (lat, long) tuples
    waypoints = [(42.4690333, -76.5026763),(42.4690488, -76.5031796),(42.4686133, -76.5026274),(42.4690286, -76.5023518),(42.4690333, -76.5026763)]  # test values

    nav_controller = nav.NavigationController(waypoints=waypoints)


if __name__ == "__main__":
    main()
