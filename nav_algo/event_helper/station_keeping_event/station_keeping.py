from nav_algo.navigation import time
from nav_algo.event_helper.station_keeping_event.station_keeping_helpers import stationKeepingHelper

def stationKeeping(navigationController):
    # TODO find an optimal radius, 10m for now
    buoy_waypoints = navigationController.configuration.waypoints
    exit_before = 300
    circle_radius = 10
    navigationController.configuration.waypoints = stationKeepingHelper(
        buoy_waypoints,
        circle_radius,
        "ENTRY",
        boat=navigationController.configuration.boat)
    navigationController.current_waypoint = navigationController.configuration.waypoints.pop(0)
    navigationController.navigate()

    # Set timer
    start_time = time.time()
    loop_waypoints = stationKeepingHelper(buoy_waypoints,
                                    circle_radius,
                                    "KEEP",
                                    boat=navigationController.configuration.boat)
    while time.time() - start_time < exit_before:
        navigationController.configuration.waypoints = loop_waypoints
        navigationController.current_waypoint = navigationController.configuration.waypoints.pop(0)
        navigationController.navigate()

    navigationController.configuration.waypoints = stationKeepingHelper(
        buoy_waypoints,
        circle_radius,
        "EXIT",
        boat=navigationController.configuration.boat)
