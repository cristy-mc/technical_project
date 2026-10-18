#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UserRequestNode(Node):

    def __init__(self):
        super().__init__("user_request_node")

        self.publisher = self.create_publisher(
            String,
            "/mission/tool_request",
            10
        )

    def request_object(self):
        while rclpy.ok():
            print("\nSelect surgical tool:")
            print("1 - scalpel")
            print("2 - scissors")
            print("3 - forceps")

            choice = input("> ").strip()

            if choice == "1":
                obj = "scalpel"
            elif choice == "2":
                obj = "scissors"
            elif choice == "3":
                obj = "forceps"
            else:
                print("Invalid choice, try again.")
                continue

            msg = String()
            msg.data = obj
            self.publisher.publish(msg)
            self.get_logger().info(f"Requested tool: {obj}")


def main(args=None):
    rclpy.init(args=args)
    node = UserRequestNode()
    try:
        node.request_object()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
