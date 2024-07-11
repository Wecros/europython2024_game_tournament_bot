# SPDX-License-Identifier: BSD-3-Clause

# flake8: noqa F401
from collections.abc import Callable

import numpy as np

from vendeeglobe import (
    Checkpoint,
    Heading,
    Instructions,
    Location,
    Vector,
    config,
)
from vendeeglobe.utils import distance_on_surface


class Bot:
    """
    This is the ship-controlling bot that will be instantiated for the competition.
    """

    DEFAULT_RADIUS = 1

    def __init__(self):
        self.team = "SoldiersOfFortune"

        locations = [
            # caribbean start
            (20.273321, -71.773118),
            (20.169290, -73.285543),
            (18.640032, -74.767563),
            # panama start
            (9.451992, -79.955866),
            (8.876689, -79.520310),
            (7.167492, -79.859252),
            # pacific ocean start
            (-8.463695, -140.442420),
            (-13.425306, -161.525028),
            # australia start
            (-39.240152, 146.381325),
            (-39.230299, 143.932370),
            (-35.490303, 115.623861),
            # arabian sea start
            (12.134831, 51.438557),
            # red sea start
            (12.371990, 43.688699),
            (13.844502, 42.370135),
            # suez start
            (27.916184, 33.663942),
            (27.953120, 33.584128),
            (28.181169, 33.435903),
            (29.694917, 32.543999),
            (30.372107, 32.477476),
            # suez start start
            (31.829888, 32.589798),
            # malta start
            (36.087080, 14.843614),
            # tunis start
            (37.374629, 11.152614),
            (37.873564, 10.091743),
            # gibraltar start
            (35.977642, -5.423997),
            (35.933084, -5.686571),
            (36.974648, -9.147140),
            (38.789331, -9.721986),
            (43.689685, -9.366345),
        ]

        locations_east_to_west = locations[::-1]

        self.course = [
            Checkpoint(latitude=lat, longitude=lon, radius=self.DEFAULT_RADIUS) for lat, lon in locations
        ]
        start_checkpoint = Checkpoint(
            latitude=config.start.latitude,
            longitude=config.start.longitude,
            radius=self.DEFAULT_RADIUS
        )

        self.course += [start_checkpoint]

    def run(
        self,
        t: float,
        dt: float,
        longitude: float,
        latitude: float,
        heading: float,
        speed: float,
        vector: np.ndarray,
        forecast: Callable,
        world_map: Callable,
    ) -> Instructions:
        """
        This is the method that will be called at every time step to get the
        instructions for the ship.

        Parameters
        ----------
        t:
            The current time in hours.
        dt:
            The time step in hours.
        longitude:
            The current longitude of the ship.
        latitude:
            The current latitude of the ship.
        heading:
            The current heading of the ship.
        speed:
            The current speed of the ship.
        vector:
            The current heading of the ship, expressed as a vector.
        forecast:
            Method to query the weather forecast for the next 5 days.
            Example:
            current_position_forecast = forecast(
                latitudes=latitude, longitudes=longitude, times=0
            )
        world_map:
            Method to query map of the world: 1 for sea, 0 for land.
            Example:
            current_position_terrain = world_map(
                latitudes=latitude, longitudes=longitude
            )

        Returns
        -------
        instructions:
            A set of instructions for the ship. This can be:
            - a Location to go to
            - a Heading to point to
            - a Vector to follow
            - a number of degrees to turn Left
            - a number of degrees to turn Right

            Optionally, a sail value between 0 and 1 can be set.
        """
        instructions = Instructions()

        # === DEBUG PRINTS ===
        current_position_forecast = forecast(
            latitudes=latitude, longitudes=longitude, times=0
        )
        current_position_terrain = world_map(latitudes=latitude, longitudes=longitude)
        if int(t) % 10 == 0:  # 4 hours = 4 steps = 1 second at speedup 1
            print(f"Forecast: {current_position_forecast}", f"Terrain: {current_position_terrain}", f"Speed: {speed}")
            print(f"Latitude: {latitude}", f"Longitude: {longitude}")
        # # ===========================================================

        # IDEA: optimize the two longest routes
        # 1st route: panama -> australia
        # 2nd route: australia -> red sea
        # Create static bigger radius checkpoints for these routes, we need to go through those
        # Check vector forecast of current position, if it is < 5 degrees against our sailing direction,
        # we check forecasts on sides, and turn to the side which is more favorable
        # If we we would be turning too much that clear way to radius checkpoints would be lost,
        # we turn the other way

        # Go through all checkpoints and find the next one to reach
        for ch in self.course:
            # Compute the distance to the checkpoint
            dist = distance_on_surface(
                longitude1=longitude,
                latitude1=latitude,
                longitude2=ch.longitude,
                latitude2=ch.latitude,
            )

            # NOTE: slowing down doesn't seem to be better
            # # Consider slowing down if the checkpoint is close
            # jump = dt * np.linalg.norm(speed)
            # if dist < 2.0 * ch.radius + jump:
            #     instructions.sail = min(ch.radius / jump, 1)
            # else:
            #     instructions.sail = 1.0

            # Check if the checkpoint has been reached
            if dist < ch.radius:
                ch.reached = True
            if not ch.reached:
                instructions.location = Location(
                    longitude=ch.longitude, latitude=ch.latitude
                )
                break

        return instructions
