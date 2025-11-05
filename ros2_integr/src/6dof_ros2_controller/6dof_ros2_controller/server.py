import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from custom_interfaces.srv import SetPose
import os

# ROS_DOMAIN_ID = 123
# os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)


class MinimalServer(Node):

    def __init__(self):
        super().__init__("pose_server")
        self._srv = self.create_service(SetPose, "set_pose", self._server_callback)
        self.get_logger().info("Server Ready...")

    def _server_callback(self, req, resp):
        pose = req.pose
        self.get_logger().info(
            f"Request {req.id} Position: x={pose.position.x:,.2f}, y={pose.position.y:,.2f}, z={pose.position.z:,.2f}"
        )
        # self.get_logger().info(
        #     f"Orientation: x={pose.orientation.x}, y={pose.orientation.y}, z={pose.orientation.z}, w={pose.orientation.w}"
        # )

        resp.success = True
        resp.id = req.id
        resp.msg = f"x={pose.position.x:,.2f}, y={pose.position.y:,.2f}, z={pose.position.z:,.2f}"
        return resp


def main(args=None):
    server_node = None
    try:
        rclpy.init(args=args)
        server_node = MinimalServer()
        rclpy.spin(server_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if server_node is not None:
            server_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
