#!/usr/bin/env python
#
# @file
# @author
# @version 0.1
# @date
#
# @short:
#

from __future__ import print_function

import itertools
from threading import Lock

from Queue import Queue
from collections import namedtuple

import rospy  # Necessary to create a ros node
from geometry_msgs.msg import PoseWithCovarianceStamped, Point  # Msg of the position of the robot
from nav_msgs.msg import OccupancyGrid, GridCells  # Msg of the path

if False:  # for IDE
    from typing import List

Pos = namedtuple('Point', ['x', 'y'])
try:
    xrange
except NameError:
    xrange = range

MEASUREMENT_RADIUS = 0.5

class Coverage:
    def __init__(self):
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        self.pos_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pos_callback, queue_size=1)
        self.coverage_pub = rospy.Publisher('/cells', GridCells, queue_size=1)

        self.map = None  # type: OccupancyGrid
        self.costmap = None  # type: List[List[int]]
        self.dists = None  # type: List[List[int]]
        self.pos = None  # type: Pos
        self.map_pos = None  # type: Pos
        self.measurements = None  # type: List[List[int]]
        self.grid = None  # type: GridCells

        self._calc_lock = Lock()  # type:Lock

    def pos_callback(self, pose):
        """:type pose: PoseWithCovarianceStamped"""
        if None in [self.map, self.costmap, self.measurements]:
            return

        if self.pos != Pos(pose.pose.pose.position.x, pose.pose.pose.position.y):
            print("pose received")
            self.pos = Pos(pose.pose.pose.position.x, pose.pose.pose.position.y)

            self.map_pos = Pos(int(round((self.pos.x - self.map.info.origin.position.x) / self.map.info.resolution)),
                               int(round((self.pos.y - self.map.info.origin.position.y) / self.map.info.resolution)))

            m = int(round(MEASUREMENT_RADIUS / self.map.info.resolution))
            for x, y in itertools.product(xrange(self.map_pos.x - m, self.map_pos.x + m),
                                          xrange(self.map_pos.y - m, self.map_pos.y + m)):
                try:
                    self.measurements[x][y] = 0
                except IndexError:
                    pass

            self.calc_grid()

    def map_callback(self, cb_map):
        """:type cb_map: OccupancyGrid"""
        print("map received")
        self.map = cb_map
        costmap = [list(l) for l in zip(*[self.map.data[y * self.map.info.width:(y + 1) * self.map.info.width]
                                          for y in xrange(self.map.info.height)])]

        for x in xrange(len(costmap)):  # type: int
            for y in itertools.chain(xrange(len(costmap[x])), reversed(xrange(len(costmap[x])))):  # type: int
                try:
                    for i, j in itertools.product(*[[0, 1, -1]] * 2):
                        costmap[x + i][y + j] = max(costmap[x + i][y + j], costmap[x][y] - 1)
                except IndexError:
                    pass
        for y in xrange(len(costmap[0])):  # type: int
            for x in itertools.chain(xrange(len(costmap)), reversed(xrange(len(costmap)))):  # type: int
                try:
                    for i, j in itertools.product(*[[0, 1, -1]] * 2):
                        costmap[x + i][y + j] = max(costmap[x + i][y + j], costmap[x][y] - 1)
                except IndexError:
                    pass

        self.costmap = costmap
        self.measurements = [[-1] * self.map.info.height for _ in xrange(self.map.info.width)]
        self.calc_grid()

    def send(self):
        if self.grid is not None:
            print("grid sent, len:", len(self.grid.cells))
            self.coverage_pub.publish(self.grid)

    def calc_grid(self):
        if None in [self.pos, self.map_pos, self.map, self.costmap, self.measurements] or len(self.map.data) <= 0:
            return

        if self.costmap[self.map_pos.x][self.map_pos.y] >= 100:
            print("position in wall")
            return

        if not self._calc_lock.acquire(False):
            return

        if self.dists is None or self.dists[self.map_pos.x][self.map_pos.y] <= -1:
            self.calc_dists()

        grid = GridCells()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = '/map'
        grid.cell_width = grid.cell_height = self.map.info.resolution
        grid.cells = list(Point(x * self.map.info.resolution + self.map.info.origin.position.x,
                                y * self.map.info.resolution + self.map.info.origin.position.y, 0)
                          for x, y in itertools.product(xrange(len(self.costmap)), xrange(len(self.costmap[0])))
                          if self.dists[x][y] > -1 and self.measurements[x][y] <= -1)
        self.grid = grid

        self._calc_lock.release()
        self.send()

    def calc_dists(self):
        wave = Queue()
        wave.put(self.map_pos)
        dist = [[-1] * self.map.info.height for _ in xrange(self.map.info.width)]
        dist[self.map_pos.x][self.map_pos.y] = 0
        while not wave.empty():
            p = wave.get()  # pos
            for n in (Pos(p.x + x, p.y + y) for x, y in [[1, 0], [0, 1], [-1, 0], [0, -1]]):  # neighbors
                if n.x < 0 or n.y < 0 or n.x >= len(self.costmap) or n.y >= len(self.costmap[n.x]):
                    continue
                if self.costmap[n.x][n.y] >= 95 or dist[n.x][n.y] != -1:
                    continue
                dist[n.x][n.y] = dist[p.x][p.y] + 1
                wave.put(n)
        self.dists = dist


def main_loop():
    global t_loop

    cov = Coverage()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('coverage')
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
