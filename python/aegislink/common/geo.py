"""Conversion between a local east/north frame in metres and latitude/longitude.

The simulated world is a few kilometres across, so a flat-earth (equirectangular) approximation
around a fixed origin is accurate to millimetres there and much simpler than full geodesy.
"""

import math

import numpy as np

EARTH_RADIUS_M = 6_378_137.0  # WGS-84 equatorial radius


def enu_to_latlon(east, north, origin):
    """East/north metres from `origin` (lat, lon in degrees) -> (lat, lon) in degrees."""
    lat0, lon0 = origin
    lat = lat0 + np.degrees(np.asarray(north) / EARTH_RADIUS_M)
    lon = lon0 + np.degrees(np.asarray(east) / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


def latlon_to_enu(lat, lon, origin):
    """(lat, lon) in degrees -> east/north metres from `origin`."""
    lat0, lon0 = origin
    north = np.radians(np.asarray(lat) - lat0) * EARTH_RADIUS_M
    east = np.radians(np.asarray(lon) - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return east, north
