def endurance(self):
    # Loop around the same waypoints for 7 hours.
    # 7 hrs = 25200 sec
    exit_before = 25200
    start_time = time.time()
    loop_waypoints = counterClockwiseRect(self.configuration.waypoints,
                                            self.configuration.boat,
                                            buoy_offset=5)

    while (time.time() - start_time < exit_before):
        self.configuration.waypoints = loop_waypoints
        self.current_waypoint = self.configuration.waypoints.pop(0)
        self.navigate()


def counterClockwiseRect(waypoints, boat, buoy_offset=2):
    if len(waypoints) > 4:
        raise RuntimeError('More than four waypoints given for endurance')

    # classify the waypoints as upper right, upper left, lower left, lower right
    # in increasing order of angle of the line from the center of the square to
    # the waypoint
    center_x = sum([w.x for w in waypoints]) / len(waypoints)
    center_y = sum([w.y for w in waypoints]) / len(waypoints)
    center = coord.Vector(x=center_x, y=center_y)

    disps = [w.vectorSubtract(center) for w in waypoints]
    angles = [d.angle() for d in disps]

    wa = []
    for i in range(len(angles)):
        wa.append((waypoints[i], angles[i]))
    wa.sort(key=lambda x: x[1])

    # get labeled buoys
    ur = wa[0][0]
    ul = wa[1][0]
    ll = wa[2][0]
    lr = wa[3][0]

    # get the angle of the line between ll and ur and the offset waypoints
    theta = np.arctan2(ur.y - ll.y, ur.x - ll.x)
    w_ll = coord.Vector(x=ll.x - buoy_offset * np.cos(theta),
                        y=ll.y - buoy_offset * np.sin(theta))
    w_ur = coord.Vector(x=ur.x + buoy_offset * np.cos(theta),
                        y=ur.y + buoy_offset * np.sin(theta))

    # get the angle of the line between lr and ul and the offset waypoints
    phi = np.arctan2(ul.y - lr.y, ul.x - ll.x)
    w_lr = coord.Vector(x=lr.x - buoy_offset * np.cos(phi),
                        y=lr.y - buoy_offset * np.sin(phi))
    w_ul = coord.Vector(x=ul.x + buoy_offset * np.cos(phi),
                        y=ul.y + buoy_offset * np.sin(phi))

    # put the waypoints into order starting from the closest to the boat in
    # a counter-clockwise direction
    boat_pos = boat.getPosition()
    boat_angle = boat_pos.vectorSubtract(center).angle()

    if boat_angle > wa[3][1] or boat_angle < wa[0][1]:
        return [w_ur, w_ul, w_ll, w_lr]
    elif boat_angle > wa[2][1]:
        return [w_lr, w_ur, w_ul, w_ll]
    elif boat_angle > wa[1][1]:
        return [w_ll, w_lr, w_ur, w_ul]
    else:
        return [w_ul, w_ll, w_lr, w_ur] 