#!/usr/bin/env python3

import rclpy

from avt_341.nodes import YoloGym

def main(args=None):
    rclpy.init(args=args)

    node = YoloGym()
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()