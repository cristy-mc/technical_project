#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


class ObjectDetector(Node):

    def __init__(self):
        super().__init__("object_detector")

        self.bridge = CvBridge()
        self.target_object = None

        # ============================================================
        # CAMERA
        # ============================================================

        self.image_sub = self.create_subscription(
            Image,
            "/iiwa2/camera_ee",
            self.image_callback,
            10
        )

        # ============================================================
        # USER REQUEST
        # ============================================================

        self.request_sub = self.create_subscription(
            String,
            "/object_request",
            self.request_callback,
            10
        )

        # ============================================================
        # RECOGNITION OUTPUT
        # ============================================================

        self.result_pub = self.create_publisher(
            String,
            "/recognized_object",
            10
        )

        self.get_logger().info(
            "[OBJECT DETECTOR] Detector started."
        )

    # ================================================================
    # REQUEST CALLBACK
    # ================================================================

    def request_callback(self, msg):

        self.target_object = msg.data.strip().lower()

        self.get_logger().info(
            f"[OBJECT DETECTOR] Searching for: {self.target_object}"
        )

    # ================================================================
    # COUNT HOLES
    # ================================================================

    def count_holes(self, hierarchy, contour_idx):

        if hierarchy is None:
            return 0

        holes = 0

        child = hierarchy[0][contour_idx][2]

        while child != -1:
            holes += 1
            child = hierarchy[0][child][0]

        return holes

    # ================================================================
    # CLASSIFICATION
    # ================================================================

    def classify_object(
        self,
        hierarchy,
        contour_idx,
        contour
    ):

        holes = self.count_holes(
            hierarchy,
            contour_idx
        )

        area = cv2.contourArea(contour)

        x, y, w, h = cv2.boundingRect(contour)

        ratio = max(w, h) / max(min(w, h), 1)

        # Classification based on holes
        if holes == 0:
            obj = "scalpel"

        elif holes == 3:
            obj = "scissors"

        else:
            obj = "forceps"

        return obj, holes, area, w, h, ratio

    # ================================================================
    # IMAGE CALLBACK
    # ================================================================

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        _, mask = cv2.threshold(
            gray,
            120,
            255,
            cv2.THRESH_BINARY_INV
        )

        contours, hierarchy = cv2.findContours(
            mask,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:

            for i, contour in enumerate(contours):

                # Solo contorni principali
                if (
                    hierarchy is not None
                    and hierarchy[0][i][3] != -1
                ):
                    continue

                area = cv2.contourArea(contour)

                if area < 150:
                    continue

                x, y, w, h = cv2.boundingRect(contour)

                obj, holes, area, w, h, ratio = \
                    self.classify_object(
                        hierarchy,
                        i,
                        contour
                    )

                # Disegno sul frame
                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    obj,
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                if (
                    self.target_object is not None
                    and obj == self.target_object
                ):

                
                    self.get_logger().info(
                        "================================"
                    )

                    self.get_logger().info(
                        f"[OBJECT DETECTOR] "
                        f" Object recognized: {obj}"
                    )

                    self.get_logger().info(
                        f"[OBJECT DETECTOR] "
                        f"Holes: {holes}"
                    )

                    self.get_logger().info(
                        f"[OBJECT DETECTOR] "
                        f"Height: {area:.0f}"
                    )

                    self.get_logger().info(
                        f"[OBJECT DETECTOR] "
                        f"Ratio: {w}"
                    )

                    self.get_logger().info(
                        f"[OBJECT DETECTOR] "
                        f"Height: {h}"
                    )

                    self.get_logger().info(
                        f"[OBJECT DETECTOR] "
                        f"Ratio: {ratio:.2f}"
                    )

                    self.get_logger().info(
                        "================================"
                    )

                    # -----------------------------------------------
                    # PUBBLICAZIONE
                    # -----------------------------------------------

                    result = String()
                    result.data = obj

                    self.result_pub.publish(result)

                    # Evita di riconoscere continuamente
                    self.target_object = None

                    break

        # ============================================================
        # NOTA: le finestre di debug (cv2.imshow "mask" e "object
        # detector") sono state rimosse - stesso motivo del nodo di
        # docking ArUco: possono bloccare/rallentare la callback sotto
        # carico nella simulazione completa, causando comportamenti
        # intermittenti a valle. Per debug visivo usare rqt_image_view
        # su un topic pubblicato, invece di imshow dentro la callback.
        # ============================================================

    # ================================================================
    # MAIN
    # ================================================================


def main(args=None):

    rclpy.init(args=args)

    node = ObjectDetector()

    rclpy.spin(node)

    node.destroy_node()

    cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
