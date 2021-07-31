# -*- coding: utf-8 -*-
"""
Created on Sat Jul 31 18:00:50 2021

@author: Sai Sathvik
"""

from gym.envs.registration import register

register(
    id='coptor-v0',
    entry_point='gym_coptor.envs:CoptorEnv',
)
register(
    id='coptor-extrahard-v0',
    entry_point='gym_coptor.envs:CoptorExtraHardEnv',
)
