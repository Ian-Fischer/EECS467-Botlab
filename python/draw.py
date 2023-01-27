from __future__ import annotations
from dataclasses import dataclass
from typing import List, Callable, Union, Any

import numpy as np
import pygame

from collections import deque

import lcm
from threading import Thread

from lcmtypes import odometry_t


lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
robot_pos = np.array([0, 0]) 
robot_theta = np.pi/2
history = deque([], maxlen=1000)
def odom_handler(channel, data):
    global robot_pos, robot_theta
    msg = odometry_t.decode(data)
    #print("got encoder message: %d %d" % (msg.x, msg.y))
    robot_pos = np.array([msg.x , msg.y])
    robot_theta = msg.theta
    history.append(robot_pos)
    #print(robot_pos)
    #print(robot_theta)

#dataclass
class Sim:
    # World dims in meters
    world_dims: np.ndarray = np.array([3, 3])
    # Screen dims in pixels
    screen_dims: np.ndarray = np.array([500, 500])
    # Robot size in meters, RH coordinate with +X aligned to 'front'
    robot_size: np.ndarray = np.array([0.1, 0.1])
    drawables: Union[List[Callable[[Sim], Any]], None] = None
    state_history: List[np.ndarray] = []

    def world_to_screen_pos(self, world_pos: np.ndarray) -> np.ndarray:
        """
        Converts an Nx2 array of world positions to screen positions
        """
        pos = np.copy(world_pos)
        # This inversion is necessary due to standard y down image coordinates
        if len(pos.shape) == 1:
            pos[1] *= -1
        else:
            pos[:, 1] *= -1
        return pos * self.screen_dims / self.world_dims + self.screen_dims / 2

    def world_to_screen_dims(self, world_dims: np.ndarray) -> np.ndarray:
        """
        Converts an Nx2 array of world dimensions to screen dimensions
        """
        return world_dims * np.array(self.screen_dims) / np.array(self.world_dims)

    def draw(self) -> None:
        # Blank background
        self.screen.fill((0, 0, 0))

        pygame.draw.lines(self.screen, (255, 0, 0), False, list(history))
        # Display the robot
        #robot_pos, robot_theta = robot.get_drawable()
        robot_sprite = pygame.transform.rotate(
            self.robot_sprite, np.rad2deg(robot_theta)
        )
        hitbox = robot_sprite.get_rect(
            center=tuple(self.world_to_screen_pos(robot_pos))
        )
        self.screen.blit(robot_sprite, hitbox)

        #print("try graphics")
        # Flip buffers to draw graphics
        pygame.display.flip()

        #print("finish graphics")

    def run(self) -> None:
        pygame.init()
        self.screen: pygame.surface.Surface = pygame.display.set_mode(
            tuple(self.screen_dims)
        )

        self.robot_sprite = pygame.transform.scale(
            pygame.image.load("arrow.jpg"),
            tuple(self.world_to_screen_dims(self.robot_size).astype(int)),
        )
        # Necessary for alpha
        self.robot_sprite.convert()
        self.robot_sprite.set_colorkey((0, 0, 0))

        running: bool = True

        while running:
            # Check for closing event
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Graphics
            #print("draw")
            self.draw()
            #lc.handle()

        pygame.quit()

def handle_lcm(lcm_obj):
    while(True):
        lcm_obj.handle()


if __name__ == "__main__":

    sim = Sim()
    sub = lc.subscribe("ODOMETRY", odom_handler)
    lcm_kill_thread = Thread(target = handle_lcm, args= (lc, ), daemon = True)
    lcm_kill_thread.start()
    sim.run()

