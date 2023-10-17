#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

AREA_MARGIN = 150  # actual search area in the table (apply margins)
DIST_MARGIN = 350  # threshold of detection


def coord_obstacle(sonarpos, distance):
    """
    Compute abs coords of obstacles and apply a mask on them.
    """
    x_obs = sonarpos[0] + distance * math.cos(sonarpos[2])
    y_obs = sonarpos[1] + distance * math.sin(sonarpos[2])

    if distance > DIST_MARGIN:
        return False
    if not AREA_MARGIN < y_obs < 3000-AREA_MARGIN:
        return False
    if not AREA_MARGIN < x_obs < 3000-AREA_MARGIN:
        return False
    return [x_obs, y_obs]
