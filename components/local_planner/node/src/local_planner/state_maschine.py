# # Final State Machine
#
#
# class SpeedStateMachin:
#     def __init__(self):
#         self.trafficLightDetected  = False
#         self.stopDetected  = False
#         self.stoppingDistance = 999.99
#         self.currentSpeed = 0.00
#         self.distanceToStopLine = 999.99
#         self.isIntersectionClear = True
#         self.currentTimeSec = 0
#         self.TrafficLightStatus = "GREEN"
#         self.past_pos = None
#         self.global_map_infos = None
#
#         # states = ['UNKNOWN', 'DRIVING', 'READY_DRIVING', 'READY_BREAKE', 'BREAKE', 'STOP']
#     def updateSpeed_gps(self, currentSpeed):
#         self.currentSpeed = currentSpeed
#         self.stoppingDistance = self.calcBreakDistance(currentSpeed)
#
#     def trafficLightDetection(self, trafficLightDetected, TrafficLightStatus):
#         self.trafficLightDetected = trafficLightDetected
#         self.TrafficLightStatus = TrafficLightStatus
#
#     def stopDetection(self, stopDetected):
#         self.stopDetected = stopDetected
#
#     def eval_TrafficLightState(self):
#         if (self.trafficLightDetected):
#             #Green
#             stop_smaler_break = self.distanceToStopLine < self.stoppingDistance
#             if (self.TrafficLightStatus == "STATE_GREEN" and not stop_smaler_break):
#                 return "TRAFFIC_LIGHT_GO"
#             if (self.TrafficLightStatus == "STATE_GREEN" and stop_smaler_break):
#                 return "TRAFFIC_LIGHT_NEAR"
#             elif(self.TrafficLightStatus == "STATE_YELLOW"):
#                 return "TRAFFIC_LIGHT_WILL_STOP"
#             elif(self.TrafficLightStatus == "STATE_RED" and stop_smaler_break):
#                 return "TRAFFIC_LIGHT_WILL_STOP"
#             elif (self.TrafficLightStatus == "STATE_RED" and not stop_smaler_break):
#                 return "TRAFFIC_LIGHT_SLOW_DOWN"
#             if self.currentSpeed == 0:
#                 return "TRAFFIC_LIGHT_WAITING"
#         return "UNKNOWN"
#
#     # overtake <> drive behind vehicle <> hold lane <> accelerate <> break <> keep_trajectory
#
#     def eval_traffic_detection(self):
#         pass
#
#     def eval_StopState(self):
#         stop_smaler_break = self.distanceToStopLine < self.stoppingDistance
#         if(self.stopDetected):
#             street_free = True
#             if(stop_smaler_break):
#                 return "STOP_NEAR"
#             if (self.currentSpeed == 0 and not stop_smaler_break):
#                 if (street_free):
#                     return "STOP_GO"
#                 else:
#                     return "STOP_WAITING"
#             if (not stop_smaler_break):
#                 return "STOP_WILL_STOP"
#         return "STOP_GO"
#
#     def eval_Linedetection(self):
#         pass
#
#     def eval_sign_detection(self):
#         pass
#
#     def eval_breake_steering(self):
#         pass
#
#     def eval_overtake_car(self):
#         pass
#
#     def eval_lane_Suitible(self):
#         pass
#
#     def eval_path_free(self):
#         pass
#
#     def eval_persons(self):
#         pass
#
#     def eval_collition(self):
#         pass
#
#     def eval_left_turn_possible(self):
#         pass
#
#     def eval_u_turn_possible(self):
#         pass
#
#     def calcBreakDistance(self, speed):
#         reakt_m = (speed/10)*3
#         bream_m = (speed/10)*(speed/10)
#         return reakt_m + bream_m
