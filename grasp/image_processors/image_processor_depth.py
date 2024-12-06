from typing import List, Type, Tuple, Any
from abc import ABC, abstractmethod
import copy
import numpy as np
import cv2
from PIL import Image, ImageDraw

from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge

class ImageProcessor(ABC):
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None

    @abstractmethod
    def detect_obj(self, image: Type[Image.Image], text: str = None, bbox: List[int] = None) -> Any:
        pass

    def depth_callback(self, data: ROSImage):
        """ROS callback function to receive depth images."""
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def get_center_of_bbox(self, bbox: List[int]) -> Tuple[int, int]:
        """Calculate the center point of a bounding box."""
        x_center = (bbox[0] + bbox[2]) // 2
        y_center = (bbox[1] + bbox[3]) // 2
        return x_center, y_center

    def get_depth_at_point(self, x: int, y: int) -> float:
        """Get depth value at a specific point in the depth image."""
        if self.depth_image is not None:
            return float(self.depth_image[y, x])
        else:
            return -1.0  # or handle as appropriate if no depth image is available

    def process_image(self, image: Type[Image.Image], bbox: List[int]):
        """Process an image to draw bounding box and get depth at center."""
        new_image = copy.deepcopy(image)
        x_center, y_center = self.get_center_of_bbox(bbox)
        depth = self.get_depth_at_point(x_center, y_center)

        # Draw rectangle and annotate depth
        draw = ImageDraw.Draw(new_image)
        draw.rectangle([(bbox[0], bbox[1]), (bbox[2], bbox[3])], outline="red")
        draw.text((x_center, y_center), f"Depth: {depth} m", fill="blue")

        return new_image, depth


class MyImageProcessor(ImageProcessor):
    def detect_obj(self, image: Type[Image.Image], text: str = None, bbox: List[int] = None) -> Any:
        # Here you can implement your object detection logic
        # For example, return a bounding box list if objects are detected
        # This is just a placeholder implementation
        return bbox

# Usage example, assuming ROS node initialization and subscriptions are handled elsewhere
# if __name__ == '__main__':
#     processor = MyImageProcessor()
#     # Assume bbox and image are defined elsewhere
#     bbox = [100, 150, 200, 250]  # Example bounding box
#     image = Image.open('path_to_image.jpg')
#     processed_image, object_depth = processor.process_image(image, bbox)
#     print(f"Depth at center of bbox: {object_depth} meters")
