def stationKeepingHelper(waypoints, circle_radius, state, boat, opt_angle=45):
    # Entry: Finds the midpoint of the four corners of the square. Gets the current position 
    # of the boat and appends the midpoint that's closest to the boat
    # then appends the center of the square 
    if state == "ENTRY":
        stationKeepingWaypoints = []  # Necessary waypoints
        # entry point to the square
        square_entries = [
            waypoints[0].midpoint(waypoints[1]),
            waypoints[1].midpoint(waypoints[2]),
            waypoints[2].midpoint(waypoints[3]),
            waypoints[3].midpoint(waypoints[0])
        ]
        curr_pos = boat.getPosition()
        shortest_dist = min(
            (curr_pos.xyDist(square_entries[0]), square_entries[0]),
            (curr_pos.xyDist(square_entries[1]), square_entries[1]),
            (curr_pos.xyDist(square_entries[2]), square_entries[2]),
            (curr_pos.xyDist(square_entries[3]), square_entries[3]),
            key=lambda x: x[0])
        stationKeepingWaypoints.append(shortest_dist[1])

        # center of the square
        # we think that waypoints[2] is diagonal from waypoints[0]
        center = waypoints[0].midpoint(waypoints[2])
        stationKeepingWaypoints.append(center)
        return stationKeepingWaypoints

    # gets the x-coord and y-coord of the boat. Finds the first waypoint by adding
    # boat direction and opt_angle (which is always 45? - why). Plots waypoints
    # in a circle 90 degrees apart (all based on where the first waypoint is - why)
    elif state == "KEEP":
        # downwind=wind-yaw=0=clockwise,
        keep_waypoints = []
        # radian_angle = math.radians(opt_angle)
        x_coord = boat.getPosition().x
        y_coord = boat.getPosition().y
        boat_direction = boat.sensors.yaw
        # get wind direction relative to boat
        relative_wind = boat.sensors.wind_direction - boat_direction
        if relative_wind < 0:
            relative_wind += 360
        if relative_wind >= 180:
            # move ccw
            loop_direction = 1
        else:
            # move cw
            loop_direction = -1
        # compute angle of first waypoint
        first_angle = boat_direction + opt_angle * loop_direction
        if (first_angle < 0):
            first_angle += 360
        # convert to radians for computation of other waypoints using trig
        first_angle_rad = math.radians(first_angle)

        for i in range(4):
            # place 4 waypoints each 90 deg apart; when angle>2pi, trig functions know to shift input to be in range
            input_angle = first_angle_rad + \
                loop_direction * i * math.radians(90)
            keep_waypoints.append(
                (x_coord + circle_radius * math.cos(input_angle)),
                (y_coord + circle_radius * math.sin(input_angle)))
        return keep_waypoints

    elif state == "EXIT":
        # TODO this assumes that the buoys are cardinal aligned, but this is
        # probably not true. I set the distance to move away from the box to
        # be large to account for this, but in the future, this should be
        # rewritten without that assumption.

        # corner waypoint order: NW, NE, SE, SW
        units_away = 40
        # north exit
        north_exit = waypoints[0].midpoint(waypoints[1])
        north_exit.y += units_away
        # east exit
        east_exit = waypoints[1].midpoint(waypoints[2])
        east_exit.x += units_away
        # south exit
        south_exit = waypoints[2].midpoint(waypoints[3])
        south_exit.y -= units_away
        # west exit
        west_exit = waypoints[0].midpoint(waypoints[3])
        west_exit.x -= units_away
        # exit waypoint order in list: N, E, S, W
        curr_pos = boat.getPosition()
        shortest_dist = min((curr_pos.xyDist(north_exit), north_exit),
                            (curr_pos.xyDist(east_exit), east_exit),
                            (curr_pos.xyDist(south_exit), south_exit),
                            (curr_pos.xyDist(west_exit), west_exit),
                            key=lambda x: x[0])
        # why are we returning shortest_dist[1]? 
        print(shortest_dist[1])
        return [shortest_dist[1]]
    