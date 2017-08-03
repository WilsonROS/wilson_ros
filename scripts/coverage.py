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

t_loop = 0.5  # Time of the loop


class Coverage:
    def __init__(self):
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        self.pos_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pos_callback, queue_size=1)
        self.coverage_pub = rospy.Publisher('/cells', GridCells, queue_size=1)

        self.map = None  # type: OccupancyGrid
        self.costmap = None  # type: List[List[int]]
        self.pos = None  # type:Pos
        self.grid = None  # type: GridCells

    def pos_callback(self, pose):
        """:type pose: PoseWithCovarianceStamped"""
        if self.pos != Pos(pose.pose.pose.position.x, pose.pose.pose.position.y):
            print("pose received")
            self.pos = Pos(pose.pose.pose.position.x, pose.pose.pose.position.y)
            self.calc_grid()

    def map_callback(self, cb_map):
        """:type cb_map: OccupancyGrid"""
        print("map received")
        self.map = cb_map
        self.costmap = [list(l) for l in zip(*[self.map.data[y * self.map.info.width:(y + 1) * self.map.info.width]
                                               for y in xrange(self.map.info.height)])]

        for x in xrange(len(self.costmap)):  # type: int
            for y in itertools.chain(xrange(len(self.costmap[x])), reversed(xrange(len(self.costmap[x])))):  # type: int
                try:
                    for i, j in itertools.product(*[[0, 1, -1]] * 2):
                        self.costmap[x + i][y + j] = max(self.costmap[x + i][y + j], self.costmap[x][y] - 1)
                except IndexError:
                    pass
        for y in xrange(len(self.costmap[0])):  # type: int
            for x in itertools.chain(xrange(len(self.costmap)), reversed(xrange(len(self.costmap)))):  # type: int
                try:
                    for i, j in itertools.product(*[[0, 1, -1]] * 2):
                        self.costmap[x + i][y + j] = max(self.costmap[x + i][y + j], self.costmap[x][y] - 1)
                except IndexError:
                    pass

        self.calc_grid()

    def send(self):
        if self.grid is not None:
            print("grid sent, len:", len(self.grid.cells))
            self.coverage_pub.publish(self.grid)

    def calc_grid(self):
        if self.pos is None or self.map is None or self.costmap is None or len(self.map.data) <= 0:
            return

        map_pos = Pos(int(round((self.pos.x - self.map.info.origin.position.x) / self.map.info.resolution)),
                      int(round((self.pos.y - self.map.info.origin.position.y) / self.map.info.resolution)))

        if self.costmap[map_pos.x][map_pos.y] >= 100:
            print("position in wall")
            return

        wave = Queue()
        wave.put(map_pos)
        dist = [[-1] * self.map.info.height for _ in xrange(self.map.info.width)]
        dist[map_pos.x][map_pos.y] = 0
        while not wave.empty():
            p = wave.get()  # pos
            for n in (Pos(p.x + x, p.y + y) for x, y in [[1, 0], [0, 1], [-1, 0], [0, -1]]):  # neighbors
                if n.x < 0 or n.y < 0 or n.x >= len(self.costmap) or n.y >= len(self.costmap[n.x]):
                    continue
                if self.costmap[n.x][n.y] >= 95 or dist[n.x][n.y] != -1:
                    continue
                dist[n.x][n.y] = dist[p.x][p.y] + 1
                wave.put(n)

        grid = GridCells()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = '/map'
        grid.cell_width = grid.cell_height = self.map.info.resolution
        grid.cells = list(Point(x * self.map.info.resolution + self.map.info.origin.position.x,
                                y * self.map.info.resolution + self.map.info.origin.position.y, 0)
                          for x, y in itertools.product(xrange(len(self.costmap)), xrange(len(self.costmap[0])))
                          if dist[x][y] > -1)
        self.grid = grid


def main_loop():
    global t_loop

    cov = Coverage()
    rate = rospy.Rate(int(round(1.0 / t_loop)))
    while not rospy.is_shutdown():
        cov.send()
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('coverage')
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
