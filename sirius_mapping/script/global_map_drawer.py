#!/usr/bin/env python3

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid


class MapOverlayNode:
    def __init__(self):
        rospy.init_node("map_overlay_node")

        self.global_map = None
        self.local_map = None

        self.global_sub = rospy.Subscriber(
            "/global_costmap",
            OccupancyGrid,
            self.global_map_callback,
            queue_size=1
        )

        self.local_sub = rospy.Subscriber(
            "/local_costmap",
            OccupancyGrid,
            self.local_map_callback,
            queue_size=1
        )

        self.combined_pub = rospy.Publisher(
            "/combined_map",
            OccupancyGrid,
            queue_size=1,
            latch=True
        )

        rospy.loginfo("Map overlay node started.")

    def global_map_callback(self, msg):
        rospy.logdebug('Przyszła globalna')
        self.global_map = msg
        self.publish_combined_map()

    def local_map_callback(self, msg):
        rospy.logdebug('rzyszła lokalna')
        self.local_map = msg
        self.publish_combined_map()

    def publish_combined_map(self):
        if self.global_map is None or self.local_map is None:
            rospy.logdebug('Missing maps')
            return

        global_map = self.global_map
        local_map = self.local_map

        # This implementation assumes both maps have the same resolution.
        if not np.isclose(
            global_map.info.resolution,
            local_map.info.resolution
        ):
            rospy.logwarn_throttle(
                5.0,
                "Global and local map resolutions differ. "
                "Cannot directly overlay them."
            )
            return

        resolution = global_map.info.resolution

        # Use the global map as the output map.
        output = OccupancyGrid()
        output.header = global_map.header
        output.info = global_map.info

        global_width = global_map.info.width
        global_height = global_map.info.height

        # Convert global map into numpy array.
        # int8 is sufficient: OccupancyGrid values only range from
        # -1 to 100, and using int16 here would needlessly double
        # the memory traffic for every mask/compare/max operation.
        global_data = np.asarray(
            global_map.data,
            dtype=np.int8
        ).reshape((global_height, global_width))

        # Start with the global map.
        output_data = global_data.copy()

        # Local map data.
        local_width = local_map.info.width
        local_height = local_map.info.height

        local_data = np.asarray(
            local_map.data,
            dtype=np.int8
        ).reshape((local_height, local_width))

        # World coordinates of the local map origin.
        local_origin_x = local_map.info.origin.position.x
        local_origin_y = local_map.info.origin.position.y

        # World coordinates of the global map origin.
        global_origin_x = global_map.info.origin.position.x
        global_origin_y = global_map.info.origin.position.y

        # Calculate local-map position relative to global-map grid.
        local_start_x = int(
            np.floor(
                (local_origin_x - global_origin_x) / resolution
            )
        )

        local_start_y = int(
            np.floor(
                (local_origin_y - global_origin_y) / resolution
            )
        )

        # Overlay local map onto global map (vectorized).
        #
        # Compute the overlapping rectangle between the local map
        # (shifted into global-grid coordinates) and the global map,
        # then operate on that rectangle with array slicing instead
        # of iterating pixel-by-pixel in Python.
        local_end_x = local_start_x + local_width
        local_end_y = local_start_y + local_height

        # Clip the shifted local-map bounds to the global map's extent.
        gx0 = max(local_start_x, 0)
        gy0 = max(local_start_y, 0)
        gx1 = min(local_end_x, global_width)
        gy1 = min(local_end_y, global_height)

        if gx0 < gx1 and gy0 < gy1:
            # Corresponding indices into the local map array.
            lx0 = gx0 - local_start_x
            ly0 = gy0 - local_start_y
            lx1 = lx0 + (gx1 - gx0)
            ly1 = ly0 + (gy1 - gy0)

            local_region = local_data[ly0:ly1, lx0:lx1]
            # Basic (non-fancy) slicing returns a *view*, not a copy,
            # so writing into global_region mutates output_data
            # directly and we can skip the copy-then-write-back step.
            global_region = output_data[gy0:gy1, gx0:gx1]

            # OccupancyGrid convention:
            #   -1 = unknown
            #    0 = free
            #  100 = occupied
            #
            # For traversability maps, this assumes larger values
            # mean more traversable, so select the lower value.
            local_known = local_region != -1
            global_unknown = global_region == -1

            # No global information, use local information.
            use_local_only = local_known & global_unknown

            # Both maps have information: select lower traversability.
            both_known = local_known & ~global_unknown

            # Where local_region == -1, global_region is left untouched,
            # matching the original "continue" behavior.
            global_region[use_local_only] = local_region[use_local_only]
            global_region[both_known] = np.maximum(
                global_region[both_known],
                local_region[both_known]
            )

        # ravel() returns a view here (output_data is C-contiguous from
        # .copy() above) instead of flatten()'s unconditional copy;
        # tolist() still makes its own copy into a Python list, so this
        # avoids doing that copy twice.
        output.data = output_data.ravel().tolist()

        self.combined_pub.publish(output)


if __name__ == "__main__":
    try:
        MapOverlayNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass