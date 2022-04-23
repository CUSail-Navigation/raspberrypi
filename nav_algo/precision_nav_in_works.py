import nav_algo.coordinates as coord
import math



'''Returns the sign of the number given as input'''
def sign_of(num):
    if num < 0: 
        return -1
    return 1

def dir_check(waypoint, endpoint):
    return coord.Vector(x = waypoint.x * sign_of(endpoint.x), y = waypoint.y * sign_of(endpoint.y))

'''depreciated'''
def find_inner_outer_points(start_point, end_point, dist, flag):
    # 1 is in, -1 is out, 0 is on the line
    slope = (start_point.y - end_point.y) / (start_point.x - end_point.x)
    theta_slope = math.atan(slope)
    new_x = start_point.x + dist * math.cos(theta_slope)
    new_y = start_point.y + dist * math.sin(theta_slope)
    if flag != 0:
        new_x += -flag * 0.1 * dist * math.sin(theta_slope)
        new_y += flag * 0.1 * dist * math.cos(theta_slope)
    return coord.Vector(x=new_x, y=new_y)


'''Distance between the current location of the boat and a line defined by 2 points
All the inputs must be of the coordinate.Vector class'''
def dist_to_2points(current_loc, p1, p2):
    top = abs((p2.x - p1.x)*(p1.y - current_loc.y) - (p1.x - current_loc.x)*(p2.y - p1.y))
    bot = math.sqrt(math.pow((p2.x-p1.x),2) + (math.pow(p2.y-p1.y, 2)))
    return top/bot


    

p1 = coord.Vector(x=-1.5, y=0)
p2 = coord.Vector(x=1.5, y=0)
p0 = coord.Vector(x=0, y=1)
#print(dist_to_2points(p0,p1,p2))


'''Using the start point and the two buoys, this function can calulate offsets
It takes the chosen buoy and finds the four points at (x+f,y+f),(x-f,y+f),(x+f,y-f),(x-f,y-f) 
where f is the offset. Then it finds the point that is farthest from the two reference points.
sp: start point that is needed as a reference, this function does not need to be run on the sp
because of the nature of the event
chosen: buoy you want to offset
ob: 'other buoy' that is needed as a reference point '''
def buoy_offset(sp, chosen, ob, offset = 5):
    possibles = [coord.Vector(x = chosen.x + offset, y = chosen.y + offset),
                coord.Vector(x = chosen.x - offset, y = chosen.y + offset),
                coord.Vector(x = chosen.x + offset, y = chosen.y - offset),
                coord.Vector(x = chosen.x - offset, y = chosen.y - offset)]
    distances = []
    for i in range(0,4): 
        distances.append(possibles[i].xyDist(sp) + possibles[i].xyDist(ob))
    return possibles[distances.index(max(distances))]

'''Takes in 2 points and outputs the point at the given fraction'''
def cutIntoFrac(p1, p2, frac):
    return coord.Vector(x = p1.x + frac*(p2.x-p1.x), y = p1.y + frac*(p2.y-p1.y))

# def close_far_points(ref, in_out, start, end, dist, offset = 5):
#     ang = start.angleBetween(end)
#     slope = (start.y - end.y) / (start.x - end.x)
#     mid = start.midpoint(end)
#     angl = 30 + (math.atan(5/start.xyDist(mid)) * in_out)
#     start_to_new = math.sqrt(25 + math.pow(start.xyDist(mid), 2))
#     y_dif = start_to_new * math.cos(angl)
#     x_dif = start_to_new * math.sin(angl)

#     return coord.Vector(x = start.x + x_dif, y = start.y - y_dif)


'''close_far_points calculated the jibing points for the sailboat. 
The function takes in a start and end point and outputs the value in the middle of those two points
that is offset away. This value can either be 'inside' or 'outside' of the line 
    in_out: 1 or -1, 1 if outside. -1 if inside.
    start: coord.Vector start point of the line
    end: coord.Vector end point of the line
    side: this is used for finding the correct values given the 3 different sides of the triangle
'''
def close_far_points(in_out, start, end, side, offset = 5):
    
    if side == 1:
        mid = start.midpoint(end)
        angl = coord.degToRad(30) + (math.atan(offset/start.xyDist(mid)) * in_out)
        start_to_new = math.sqrt(math.pow(offset,2) + math.pow(start.xyDist(mid), 2))
        y_dif = start_to_new * math.cos(angl)
        x_dif = start_to_new * math.sin(angl)
        output = coord.Vector()
        output.x = start.x - x_dif
        output.y = start.y - y_dif
    if side == 2:
        mid = start.midpoint(end)
        angl = math.atan(offset/start.xyDist(mid)) * in_out * -1
        start_to_new = math.sqrt(math.pow(offset,2) + math.pow(start.xyDist(mid), 2))
        y_dif = start_to_new * math.sin(angl)
        x_dif = start_to_new * math.cos(angl)
        output = coord.Vector()
        output.x = start.x + x_dif
        output.y = start.y + y_dif
    if side == 3:
        mid = start.midpoint(end)
        angl = coord.degToRad(-30) + (math.atan(offset/start.xyDist(mid)) * in_out *1)
        start_to_new = math.sqrt(math.pow(offset,2) + math.pow(start.xyDist(mid), 2))
        y_dif = start_to_new * math.cos(angl)
        x_dif = start_to_new * math.sin(angl)
        output = coord.Vector()
        output.x = start.x - (x_dif * in_out)
        output.y = start.y + y_dif
    return output
    
     



'''This function is meant to calculate a set of waypoints'''
def precisionNavigation(waypoints, offset=5.0, side_length=50.0):
    # waypoints:[topleft_buoy, topright_buoy, botleft_buoy, botright_buoy]
    topleft_buoy = waypoints[0]
    topright_buoy = waypoints[1]
    botleft_buoy = waypoints[2]
    botright_buoy = waypoints[3]

    waypoints = []

    #calulating the offsets and start point where the boat will actually be moving around
    start_pos = topleft_buoy.midpoint(topright_buoy)
    bottom_left_offset = buoy_offset(start_pos, botleft_buoy, botright_buoy)
    bottom_right_offset = buoy_offset(start_pos, botright_buoy, botleft_buoy)
    waypoints.append(start_pos) #waypoint #1

    #find inner and outer waypoints on first side of triangle from start_pos to bottom_left_offset 
    #side = 1
    third_waypoint = cutIntoFrac(start_pos,bottom_left_offset, 1/3)
    second_waypoint = close_far_points(-1, start_pos, third_waypoint, 1)
    fifth_waypoint = cutIntoFrac(third_waypoint,bottom_left_offset, 1/2)
    fourth_waypoint = close_far_points(1, third_waypoint, fifth_waypoint, 1)
    sixth_waypoint = close_far_points(1, fifth_waypoint, bottom_left_offset, 1)

    waypoints.append(second_waypoint)
    waypoints.append(third_waypoint)
    waypoints.append(fourth_waypoint)
    waypoints.append(fifth_waypoint)
    waypoints.append(sixth_waypoint)
    waypoints.append(bottom_left_offset) #waypoint #7

    #end of first path, boat should be at the bottom_left offset point
    

    #begin of second path from bottom_left_offset to bottom_right_offset
    #side = 2
    ninth_waypoint = cutIntoFrac(bottom_left_offset,bottom_right_offset, 1/3) #point 1/3 of way to bottom__right_offset
    eighth_waypoint = close_far_points(1, bottom_left_offset, ninth_waypoint, 2) #point inside 1/6 of the way offset distance
    eleventh_waypoint = cutIntoFrac(ninth_waypoint,bottom_right_offset, 1/2) #point 2/3 of the way to bottom_right_offset
    tenth_waypoint = close_far_points(-1, ninth_waypoint, eleventh_waypoint, 2) #point 3/6 way and offset distance
    twelth_waypoint = close_far_points(1, eleventh_waypoint, bottom_right_offset, 2) #point 5/6 of the way and offset distance

    waypoints.append(eighth_waypoint)
    waypoints.append(ninth_waypoint)
    waypoints.append(tenth_waypoint)
    waypoints.append(eleventh_waypoint)
    waypoints.append(twelth_waypoint)
    waypoints.append(bottom_right_offset) #waypoint #13

    #begin of second path from bottom_right_offset to start_point
    #side = 3
    fifthteenth_waypoint = cutIntoFrac(bottom_right_offset,start_pos, 1/3)
    fourteenth_waypoint = close_far_points(1, bottom_right_offset, fifthteenth_waypoint, 3) 
    seventheeth_waypoint = cutIntoFrac(fifthteenth_waypoint,start_pos, 1/2) 
    sixteenth_waypoint = close_far_points(-1, fifthteenth_waypoint, seventheeth_waypoint, 3) 
    eighteenth_waypoint = close_far_points(1, seventheeth_waypoint, start_pos, 3) 

    waypoints.append(fourteenth_waypoint)
    waypoints.append(fifthteenth_waypoint)
    waypoints.append(sixteenth_waypoint)
    waypoints.append(seventheeth_waypoint)
    waypoints.append(eighteenth_waypoint)
    waypoints.append(start_pos) #waypoint 19

    

    
    return waypoints

    

way_p = [coord.Vector(x=-1.5, y=0), 
    coord.Vector(x=1.5, y=0), 
    coord.Vector(x= -25,y=-25 * math.sqrt(3)), 
    coord.Vector(x=25, y=-25 * math.sqrt(3))]

way_pp = [coord.Vector(x=-1.5, y= -25 *math.sqrt(3)), 
    coord.Vector(x=1.5, y= -25 *math.sqrt(3)), 
    coord.Vector(x= -25,y=0), 
    coord.Vector(x=25, y=0)]

new_ways = precisionNavigation(way_p)
i = 1
for way in new_ways:
    print(str(i) + " x: " + str(way.x) + ", y: " + str(way.y))
    i+=1





