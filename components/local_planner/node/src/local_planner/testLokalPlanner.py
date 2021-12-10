import math
class testLocalPlanner:
    globalRoute = []
    route = []
    gps_x_y = (0.0, 0.0)
    last_visited_wp = 0
    max_wp = 0
    send_Route = []
    vehicle_orientation_rad = 0.0

    def setRoute(self):
        pass

    def linear_interpolation(self, start, end, interval_m):
        listPol = []
        dist = self.dist(start, end)
        diff = (start[0]-end[0], start[1] - end[1])
        #+1 bei Komma
        steps = (int(dist/interval_m))
        adddiff = (0.0, 0.0)
        if dist > interval_m:
            adddiff = (diff[0]/ steps, diff[1]/ steps,)
        else:
            adddiff = diff

        diff = (0.0, 0.0)
        for index in range (steps):
            point = (start[0]+diff[0], start[1] + diff[1])
            diff = (diff[0]+adddiff[0], diff[1]+adddiff[1])
            if index > 0 and (listPol[-1]== point):
                continue
            listPol.append(point)
        return listPol

    def updateRoute(self):
        self.send_Route = []
        self.route = []
        dist_for_next_wp = 1.0
        interval = 0.5
        show_next_points  = 10

        #if len(self.route)==0 and len(self.globalRoute)>0 and self.last_visited_wp != self.max_wp:
        for index in range(len(self.globalRoute)-1):
            listPol = self.linear_interpolation(self.globalRoute[index], self.globalRoute[index+1], interval)
            self.route += listPol
        self.max_wp = len(self.route)


        while self.dist(self.gps_x_y, self.route[self.last_visited_wp]) < dist_for_next_wp:

            self.last_visited_wp+= 1

        till_wp = self.last_visited_wp + show_next_points
        if till_wp > self.max_wp:
            till_wp = self.max_wp
        self.send_Route.append(self.gps_x_y)

        for index in range(self.last_visited_wp, till_wp):
            self.send_Route.append(self.route[index])

        self._compute_steering_angle(self.send_Route)
        return self.send_Route
        

    def dist(self, start, end):
        return math.sqrt((end[0]-start[0])**2 +(end[1]-start[1])**2)

    def _norm_angle(self, angle_rad: float):
        while angle_rad > math.pi:
            angle_rad -= 2.0 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2.0 * math.pi
        return angle_rad

    def _compute_steering_angle(self, route):
        aim_pointID = 1
        aim_point = route[aim_pointID]
        dist_for_next_wp = 0.5

        while self.dist(aim_point, route[0]) < dist_for_next_wp and len(route) > aim_pointID:
            aim_pointID += 1
            aim_point = route[aim_pointID]

        diff = (route[0][0]-aim_point[0] , route[0][1]- aim_point[1])
        angle = math.atan2(diff[1], diff[0])
        steer_angle = self.vehicle_orientation_rad + angle
        #self.vehicle_orientation_rad += steer_angle
        steer_angle = self._norm_angle(steer_angle)
        print("Angle relative to start", math.degrees(angle))
        print("Steering: ", math.degrees(steer_angle))
        return steer_angle

if __name__ == "__main__":
    lp = testLocalPlanner()
    #Linkskurve
    lp.globalRoute = [(0.0, 0.0),(1.0, 0.0),(2.0, 1.0),(2.0,2.0),(3.0,3.0)]

    #Manuelles Setzen GPS
    lp.vehicle_orientation_rad = math.pi/2
    #Geradeaus
    lp.updateRoute()

    #(0.5, 0.0) -> (2.0, 1.0)
    lp.gps_x_y = (0.5, 0.0)
    lp.updateRoute()

    # (1.0, 0.1) -> (2.0, 1.0)
    lp.gps_x_y = (1.0, 0.1)
    #lp.vehicle_orientation_rad -= 0.46365
    lp.updateRoute()


    lp.gps_x_y = (1.5, 0.5)
    lp.updateRoute()
    lp.gps_x_y = (2.0, 1.0)
    lp.updateRoute()
    lp.gps_x_y = (2.5, 1.35)
    lp.updateRoute()


