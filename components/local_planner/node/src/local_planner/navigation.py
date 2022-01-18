"""A module providing navigation services"""

import json
from time import sleep
from math import dist
from random import choice
from typing import Callable, List, Tuple
from dataclasses import dataclass

import rospy

from nav_srvs.srv import NavigationRequest
from local_planner.core import Vehicle


def load_spawn_positions() -> List[Tuple[float, float]]:
    # TODO: put this into a config file
    return [(335.489990234375, -273.7433166503906), (299.3999938964844, -133.2400360107422),
        (299.3999938964844, -129.75), (299.3999938964844, -59.33003616333008),
        (299.3999938964844, -55.84000015258789), (272.2900085449219, -59.33003616333008),
        (272.2900085449219, -55.84000015258789), (234.26998901367188, -59.33001708984375),
        (234.26998901367188, -55.84001922607422), (202.5500030517578, -59.33001708984375),
        (202.5500030517578, -55.84000015258789), (92.1099853515625, -227.22000122070312),
        (144.0592803955078, -133.23988342285156), (191.0800018310547, -55.84000015258789),
        (153.75999450683594, -28.899999618530273), (144.0592803955078, -129.74986267089844),
        (176.58949279785156, -133.23915100097656), (158.0800018310547, -27.18000030517578),
        (295.0817565917969, -199.060302734375), (295.0817565917969, -195.5703125),
        (258.2618408203125, -199.06002807617188), (258.2618408203125, -195.57003784179688),
        (88.61997985839844, -226.9056396484375), (233.5437774658203, -198.7597198486328),
        (234.71487426757812, -195.26963806152344), (181.1729278564453, -198.75979614257812),
        (181.1729278564453, -195.26980590820312), (142.3605499267578, -198.75999450683594),
        (142.91998291015625, -195.26998901367188), (142.91998291015625, -330.4599914550781),
        (142.91998291015625, -326.9700012207031), (120.02603149414062, -330.4599914550781),
        (124.73997497558594, -326.9700012207031), (92.10997772216797, -247.16357421875),
        (173.11997985839844, -330.4599914550781), (173.11997985839844, -326.9700012207031),
        (199.94998168945312, -330.4599914550781), (199.94998168945312, -326.9700012207031),
        (232.19998168945312, -330.4599914550781), (232.19998168945312, -326.9700012207031),
        (262.5999755859375, -330.53997802734375), (262.5999755859375, -327.04998779296875),
        (301.3399658203125, -330.53997802734375), (301.3399658203125, -327.04998779296875),
        (88.61997985839844, -299.9212951660156), (194.4623260498047, -198.75997924804688),
        (195.00482177734375, -195.27003479003906), (358.39996337890625, -330.53997802734375),
        (366.53997802734375, -327.04998779296875), (229.01377868652344, -133.24026489257812),
        (228.99429321289062, -129.75006103515625), (60.10997772216797, -330.4599914550781),
        (61.21516036987305, -326.9700012207031), (46.14997863769531, -330.4599914550781),
        (46.14997863769531, -326.9700012207031), (92.10997772216797, -299.781494140625),
        (22.17997932434082, -330.4599914550781), (22.17997932434082, -326.9700012207031),
        (131.729736328125, -59.33001708984375), (161.10308837890625, -198.759765625),
        (116.22090911865234, -59.33001708984375), (118.94999694824219, -55.84000015258789),
        (392.4700012207031, -118.73902130126953), (392.4700012207031, -229.07254028320312),
        (396.6375732421875, -318.38226318359375), (396.6376037597656, -208.82986450195312),
        (396.6376037597656, -87.52951049804688), (2.0529332160949707, -318.38226318359375),
        (2.0529332160949707, -208.60116577148438), (2.0529332160949707, -99.2503662109375),
        (-1.76282799243927, -9.559755325317383), (-1.7628377676010132, -119.34102630615234),
        (-1.7628536224365234, -228.6918487548828), (322.7909240722656, 2.114088773727417),
        (322.09625244140625, -55.15309143066406), (322.09625244140625, -129.35906982421875),
        (121.22996520996094, -195.00999450683594), (321.7520446777344, -194.7324981689453),
        (321.7520446777344, -326.2574462890625), (212.64230346679688, -326.2574462890625),
        (212.64230346679688, -194.77291870117188), (212.64230346679688, -129.3616180419922),
        (339.0091247558594, -314.88629150390625), (92.48524475097656, -314.88629150390625),
        (105.57198333740234, -199.44183349609375), (210.86700439453125, -199.44183349609375),
        (210.86700439453125, -133.64796447753906), (-1.980019450187683, -249.42999267578125),
        (105.57198333740234, -133.7781524658203), (105.57198333740234, -330.8506164550781),
        (213.76470947265625, -330.8506164550781), (88.03582000732422, -210.6703338623047),
        (334.7348327636719, -210.6703338623047), (213.8804473876953, -59.902679443359375),
        (213.8804473876953, -2.4971039295196533), (170.5475311279297, -59.902679443359375),
        (212.7645721435547, -55.239990234375), (1.5099804401397705, -249.42999267578125),
        (212.7645721435547, 2.286572217941284), (334.3560485839844, -74.32872772216797),
        (339.01873779296875, -116.54576110839844), (88.13700866699219, -77.93560028076172),
        (92.7997055053711, -116.54576110839844), (87.68768310546875, -145.67129516601562),
        (92.76799774169922, -183.88839721679688), (334.21539306640625, -145.67129516601562),
        (339.2957763671875, -183.88839721679688), (334.83502197265625, -14.213335037231445),
        (-1.2800195217132568, -309.4599914550781), (339.2957763671875, -45.07813262939453),
        (153.65826416015625, -14.213335037231445), (158.11900329589844, -45.07813262939453),
        (87.71388244628906, -14.213335037231445), (92.17462921142578, -45.07813262939453),
        (104.8687973022461, -59.99010467529297), (140.8768310546875, -55.5607795715332),
        (104.8687973022461, -2.461965322494507), (140.8768310546875, 1.9673608541488647),
        (352.0755310058594, -2.2301647663116455), (1.5099139213562012, -295.4233093261719),
        (383.17987060546875, 2.1991612911224365), (13.568642616271973, -2.461965322494507),
        (76.44566345214844, 1.9673608541488647), (13.568642616271973, -330.4619445800781),
        (76.44566345214844, -326.4326171875), (350.8720703125, -330.9415283203125),
        (381.9764099121094, -326.35064697265625), (381.9800720214844, -330.8915100097656),
        (248.7059783935547, -133.23997497558594), (12.785676956176758, 2.073781967163086),
        (392.4700012207031, -212.89999389648438), (259.39117431640625, -133.23997497558594),
        (248.39857482910156, -129.7498016357422), (259.5984191894531, -129.74978637695312),
        (283.6458740234375, -133.43006896972656), (283.6722717285156, -129.48997497558594),
        (161.10308837890625, -195.26976013183594), (121.78265380859375, -198.75843811035156),
        (176.58949279785156, -129.74913024902344), (288.23748779296875, -59.33000946044922),
        (288.23748779296875, -55.839969635009766), (257.2313537597656, -59.330135345458984),
        (257.2313537597656, -55.84009552001953), (268.5865783691406, 1.9599138498306274),
        (256.55120849609375, -2.0201635360717773), (256.55120849609375, 1.9598363637924194),
        (293.288818359375, -2.019927501678467), (293.288818359375, 1.96007239818573),
        (229.78167724609375, -2.0201120376586914), (229.78167724609375, 1.959887981414795),
        (187.5231475830078, -2.020054578781128), (132.35960388183594, -195.01046752929688),
        (392.4700012207031, -249.42999267578125), (132.91229248046875, -198.75889587402344),
        (151.11973571777344, -198.7598419189453), (151.67916870117188, -195.26983642578125),
        (170.82284545898438, -198.7600860595703), (170.82284545898438, -195.2700958251953),
        (222.4978485107422, -198.75917053222656), (223.6689453125, -195.26910400390625),
        (245.8509063720703, -198.75924682617188), (247.02200317382812, -195.26918029785156),
        (270.94085693359375, -199.05966186523438), (395.9599914550781, -249.42999267578125),
        (270.94085693359375, -195.5696563720703), (283.02197265625, -199.05972290039062),
        (283.02197265625, -195.56971740722656), (307.39813232421875, -199.0607147216797),
        (307.39813232421875, -195.5707244873047), (92.10997772216797, -285.2532043457031),
        (88.6199722290039, -275.4779052734375), (92.10997009277344, -275.3381042480469),
        (88.6199722290039, -263.1473693847656), (92.10997009277344, -263.0075988769531),
        (392.4700012207031, -308.2099914550781), (88.61997985839844, -247.4580841064453),
        (395.9599914550781, -308.2099914550781), (392.4700012207031, -68.86003875732422),
        (395.9599914550781, -68.86003875732422), (392.4700012207031, -105.38999938964844),
        (395.9599914550781, -105.38999938964844), (392.4700012207031, -9.186657905578613),
        (395.9599914550781, -164.1699981689453), (392.4700012207031, -19.9200382232666),
        (396.4499816894531, -19.9200382232666), (378.17999267578125, -2.0200390815734863),
        (370.1506652832031, 1.9599329233169556), (366.5357666015625, -2.0200233459472656),
        (358.7441711425781, 1.9599329233169556), (334.8299865722656, -226.65386962890625),
        (306.28997802734375, -2.02001953125), (88.61997985839844, -285.39300537109375),
        (278.80999755859375, -2.02001953125), (278.80999755859375, 1.9599803686141968),
        (244.09999084472656, -2.02001953125), (244.09999084472656, 1.9599803686141968),
        (200.6326446533203, -2.020034074783325), (200.49667358398438, 1.959922432899475),
        (173.14999389648438, -2.02001953125), (185.55999755859375, 1.9599803686141968),
        (338.97998046875, -226.75), (130.8036651611328, -2.02001953125),
        (130.3652801513672, 1.95999014377594), (116.97260284423828, -2.019996404647827),
        (116.63999938964844, 1.95999014377594), (62.12999725341797, -2.020014524459839),
        (64.36146545410156, 1.9599950313568115), (47.939998626708984, -2.020014524459839),
        (47.939998626708984, 1.9599950313568115), (338.97998046875, -273.9259948730469),
        (29.91197395324707, 1.9599987268447876), (335.489990234375, -249.42999267578125),
        (1.55999755859375, -22.440019607543945), (-1.536295771598816, -26.766714096069336),
        (1.5599950551986694, -48.70001983642578), (-1.9084486961364746, -48.70000076293945),
        (1.5599950551986694, -79.32001495361328), (-1.7406071424484253, -79.31999969482422),
        (1.5599901676177979, -120.02001953125), (392.4700012207031, -87.41051483154297),
        (1.5599901676177979, -149.83001708984375), (-2.4200096130371094, -149.8300018310547),
        (338.97998046875, -249.42999267578125), (1.5599803924560547, -187.9700164794922),
        (-2.4200193881988525, -187.97000122070312), (88.61998748779297, -169.84999084472656),
        (92.10990142822266, -170.5440216064453), (395.9599914550781, -191.06936645507812),
        (92.1099853515625, -159.9499969482422), (88.61998748779297, -111.18923950195312),
        (92.1099853515625, -105.66153717041016), (88.61998748779297, -101.83394622802734),
        (92.1099853515625, -94.09305572509766), (335.489990234375, -298.80999755859375),
        (392.4700012207031, -190.6240997314453), (92.11000061035156, -81.83161926269531),
        (392.4700012207031, -163.92144775390625), (92.11000061035156, -30.820009231567383),
        (88.6199951171875, -26.559999465942383), (88.61998748779297, -90.58333587646484),
        (268.5865783691406, -2.0200860500335693), (119.46998596191406, -129.75),
        (128.94998168945312, -133.24002075195312), (128.94998168945312, -129.75),
        (338.97998046875, -301.2599792480469), (157.1899871826172, -133.24002075195312),
        (157.1899871826172, -129.75), (191.3199920654297, -133.24002075195312),
        (191.3199920654297, -129.75), (220.13681030273438, -133.24014282226562),
        (219.64501953125, -129.7499542236328), (237.6999969482422, -133.239990234375),
        (237.6999969482422, -129.75), (270.79998779296875, -133.43003845214844),
        (271.0400085449219, -129.489990234375)]


@dataclass
class InfiniteDrivingService():
    """Representing a proxy for requesting navigation services."""
    vehicle: Vehicle
    update_route: Callable
    destinations: List[Tuple[float, float]] = None
    current_dest: Tuple[float, float] = None

    def __post_init__(self):
        if self.destinations is None:
            # TODO: load this from a config file
            self.destinations = load_spawn_positions()

    def run_infinite_driving(self):
        """Launch the infinite driving mode. This will always request new routes
        to random destinations whenever the car completes the previous route."""
        list_except = lambda l, e: [x for x in l if x != e]

        while True:
            if not self.vehicle.is_ready:
                sleep(1)
                continue

            if self.current_dest is None:
                self.current_dest = self.vehicle.pos

            if dist(self.vehicle.pos, self.current_dest) > 10.0:
                sleep(0.01)
                continue

            start_pos = self.vehicle.pos
            orientation = self.vehicle.orientation_rad
            self.current_dest = choice(list_except(self.destinations, self.current_dest))

            route = self._request_new_route(start_pos, self.current_dest, orientation)
            self.update_route(route)

    def _request_new_route(self, start_pos: Tuple[float, float], end_pos: Tuple[float, float],
                           orientation_rad: float) -> List[Tuple[float, float]]:

        service_name = 'navigate'
        rospy.wait_for_service(service_name)

        while True:
            try:
                navigate_proxy = rospy.ServiceProxy(service_name, NavigationRequest)
                response = navigate_proxy(start_pos[0], start_pos[1],
                                          end_pos[0], end_pos[1], orientation_rad)

                if not response.success:
                    continue

                json_list = json.loads(response.waypoints_json)
                waypoints = [(wp['x'], wp['y']) for wp in json_list]
                return waypoints

            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")
                sleep(1)
