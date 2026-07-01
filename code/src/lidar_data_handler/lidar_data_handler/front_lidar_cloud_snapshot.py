import csv
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


def pointcloud2_xy_array(msg):
    points = pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
    points = points if isinstance(points, np.ndarray) else np.asarray(list(points))

    if points.size == 0:
        return np.empty((0, 2), dtype=np.float32)

    if points.dtype.names:
        points = np.column_stack((points["x"], points["y"])).astype(np.float32, copy=False)
    else:
        points = np.asarray(points, dtype=np.float32)
        points = points.reshape((-1, 2)) if points.ndim == 1 else points[:, :2]

    return points[np.isfinite(points).all(axis=1)]


def world_to_pixel(points, x_min, y_min, scale, height, margin):
    pixels = np.empty_like(points, dtype=np.int32)
    pixels[:, 0] = np.round((points[:, 0] - x_min) * scale + margin).astype(np.int32)
    pixels[:, 1] = np.round(height - margin - (points[:, 1] - y_min) * scale).astype(np.int32)
    return pixels


def draw_label(image, text, origin, color):
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2.LINE_AA,
    )


def save_cloud_plot(points, output_image, image_size):
    image_size = int(image_size)
    margin = max(40, image_size // 20)
    image = np.full((image_size, image_size, 3), 255, dtype=np.uint8)

    if points.size == 0:
        draw_label(image, "No finite points", (margin, margin), (0, 0, 255))
        cv2.imwrite(output_image, image)
        return

    x_min = float(np.min(points[:, 0]))
    x_max = float(np.max(points[:, 0]))
    y_min = float(np.min(points[:, 1]))
    y_max = float(np.max(points[:, 1]))

    x_span = max(x_max - x_min, 1e-3)
    y_span = max(y_max - y_min, 1e-3)
    span = max(x_span, y_span)
    x_mid = (x_min + x_max) / 2.0
    y_mid = (y_min + y_max) / 2.0
    pad = span * 0.08
    x_min = x_mid - span / 2.0 - pad
    x_max = x_mid + span / 2.0 + pad
    y_min = y_mid - span / 2.0 - pad
    y_max = y_mid + span / 2.0 + pad

    drawable = image_size - 2 * margin
    scale = drawable / max(x_max - x_min, y_max - y_min)

    pixels = world_to_pixel(points, x_min, y_min, scale, image_size, margin)
    in_image = (
        (pixels[:, 0] >= 0)
        & (pixels[:, 0] < image_size)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < image_size)
    )
    pixels = pixels[in_image]
    for px, py in pixels:
        cv2.circle(image, (int(px), int(py)), 1, (255, 0, 0), -1, cv2.LINE_AA)

    origin = np.array([[0.0, 0.0]], dtype=np.float32)
    origin_px = world_to_pixel(origin, x_min, y_min, scale, image_size, margin)[0]
    ox, oy = int(origin_px[0]), int(origin_px[1])

    if 0 <= ox < image_size:
        cv2.line(image, (ox, margin // 2), (ox, image_size - margin // 2), (190, 190, 190), 1)
    if 0 <= oy < image_size:
        cv2.line(image, (margin // 2, oy), (image_size - margin // 2, oy), (190, 190, 190), 1)
    if 0 <= ox < image_size and 0 <= oy < image_size:
        cv2.circle(image, (ox, oy), 5, (0, 0, 0), -1, cv2.LINE_AA)

    axis_len_m = max(0.5, span * 0.22)
    axis_points = np.array([[0.0, 0.0], [axis_len_m, 0.0], [0.0, axis_len_m]], dtype=np.float32)
    axis_pixels = world_to_pixel(axis_points, x_min, y_min, scale, image_size, margin)
    o = tuple(axis_pixels[0])
    x_end = tuple(axis_pixels[1])
    y_end = tuple(axis_pixels[2])
    cv2.arrowedLine(image, o, x_end, (0, 0, 255), 2, cv2.LINE_AA, tipLength=0.15)
    cv2.arrowedLine(image, o, y_end, (0, 160, 0), 2, cv2.LINE_AA, tipLength=0.15)
    draw_label(image, "+X", (x_end[0] + 6, x_end[1] - 6), (0, 0, 255))
    draw_label(image, "+Y", (y_end[0] + 6, y_end[1] - 6), (0, 160, 0))

    draw_label(image, "front lidar raw /scan_cloud", (margin, margin), (20, 20, 20))
    draw_label(
        image,
        f"x:[{np.min(points[:, 0]):.2f},{np.max(points[:, 0]):.2f}] "
        f"y:[{np.min(points[:, 1]):.2f},{np.max(points[:, 1]):.2f}] n:{points.shape[0]}",
        (margin, image_size - margin // 2),
        (20, 20, 20),
    )

    cv2.imwrite(output_image, image)


class FrontLidarCloudSnapshot(Node):
    def __init__(self):
        super().__init__("front_lidar_cloud_snapshot")
        self.declare_parameter("topic", "/front_lidar/scan_cloud")
        self.declare_parameter("output_dir", ".")
        self.declare_parameter("output_image", "")
        self.declare_parameter("output_csv", "")
        self.declare_parameter("image_size", 1200)

        self.topic = self.get_parameter("topic").value
        self.output_dir = self.get_parameter("output_dir").value
        self.output_image = self.get_parameter("output_image").value
        self.output_csv = self.get_parameter("output_csv").value
        self.image_size = self.get_parameter("image_size").value
        self.saved = False

        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"Waiting for one PointCloud2 message on {self.topic}")

    def resolve_outputs(self):
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        image = self.output_image or os.path.join(self.output_dir, f"front_lidar_raw_{timestamp}.png")
        csv_path = self.output_csv or os.path.join(self.output_dir, f"front_lidar_raw_{timestamp}.csv")
        return image, csv_path

    def cloud_callback(self, msg):
        if self.saved:
            return
        self.saved = True

        points = pointcloud2_xy_array(msg)
        output_image, output_csv = self.resolve_outputs()

        with open(output_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            writer.writerows(points.tolist())

        save_cloud_plot(points, output_image, self.image_size)

        if points.size == 0:
            self.get_logger().warn(f"Saved empty cloud image: {output_image}")
        else:
            positive_x = int(np.count_nonzero(points[:, 0] >= 0.0))
            negative_x = int(np.count_nonzero(points[:, 0] < 0.0))
            self.get_logger().info(
                f"Saved {points.shape[0]} raw points to {output_csv}; "
                f"image to {output_image}; +X points={positive_x}, -X points={negative_x}"
            )

        rclpy.shutdown()


def main():
    rclpy.init()
    node = FrontLidarCloudSnapshot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
