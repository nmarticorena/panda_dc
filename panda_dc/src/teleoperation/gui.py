import numpy as np
from typing import Optional

import swift
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm


class SwiftGui:
    def __init__(
        self,
        i_follower_q: Optional[np.ndarray] = None,
        i_leader_q: Optional[np.ndarray] = None,
    ):
        env = swift.Swift()
        env.launch()

        self.follower = rtb.models.Panda()  # Real robot
        if i_follower_q:
            self.follower.q = i_follower_q
        env.add(self.follower)

        self.leader = rtb.models.Panda()  # Gello
        if i_leader_q:
            self.leader.q = i_leader_q
        env.add(self.leader, robot_alpha=0.5)

        self.env = env

    def step(self, follower_q: np.ndarray, leader_q: np.ndarray):
        self.follower.q = follower_q
        self.leader.q = leader_q
        self.env.step()
