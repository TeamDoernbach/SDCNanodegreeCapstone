import rospy
import tensorflow as tf
import numpy as np
from PIL import ImageDraw
from PIL import Image
from styx_msgs.msg import TrafficLight

"""
The following functions are have been adopted from CarND-Object-Detection-Lab by Udacity
(https://github.com/udacity/CarND-Object-Detection-Lab)

  filter_boxes, draw_boxes, to_image_coord, load_graph
"""


class TLClassifier(object):
    """
    NOTE: The job of the classifier is to provide only the state of the traffic light, 
    and not the bounding box. The boxes_tensor does not have to be evaluated and we 
    might remove it later.
    """

    def __init__(self, graph_path, min_score=0.8):
        
        # load classifier TF graph
        self.tfgraph = self.load_graph(graph_path)
        rospy.loginfo('Loaded inference graph at {}'.format(graph_path))

        # pick out relevant tensors from the graph
        self.input_tensor = self.tfgraph.get_tensor_by_name('image_tensor:0')
        self.boxes_tensor = self.tfgraph.get_tensor_by_name('detection_boxes:0')
        self.scores_tensor = self.tfgraph.get_tensor_by_name('detection_scores:0')
        self.classes_tensor = self.tfgraph.get_tensor_by_name('detection_classes:0')

        # start TF session
        self.tfsess = tf.Session(graph=self.tfgraph)
        rospy.loginfo('Session created.')

        # minimum score for filtering out the detection boxes
        self.MIN_SCORE = min_score
        self.COLOR_LIST = ['red', 'yellow', 'green']
        

    def get_classification(self, image, debug=False):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        

        # with tf.Session(graph=self.tfgraph) as sess:
        # convert image to numpy array of uint8 and add an axis to the shape
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)
        # evaluate the tensors for boxes, scores and classes
        boxes, scores, classes = self.tfsess.run(
            [self.boxes_tensor, self.scores_tensor, self.classes_tensor], 
            feed_dict={self.input_tensor: image_np})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        boxes, scores, classes = self.filter_boxes(boxes, scores, classes)

        
        if debug:
            image = Image.fromarray(image)
            width, height = image.size
            # rospy.loginfo('Image size: {}'.format(image_np.shape))
            box_coords = self.to_image_coords(boxes, height, width)
            self.draw_boxes(image, box_coords, classes)
            return np.array(image)

        else:
            if classes:  # if some traffic lights were detected
                # decode class ID
                class_id = classes[0]-1
                if class_id == 0:
                    return TrafficLight.RED
                elif class_id == 1:
                    return TrafficLight.YELLOW
                elif class_id == 2:
                    return TrafficLight.GREEN
                else:
                    return TrafficLight.UNKNOWN
            else:
                return TrafficLight.UNKNOWN


    def filter_boxes(self, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= self.MIN_SCORE:
                idxs.append(i)
        
        return boxes[idxs, ...], scores[idxs, ...], classes[idxs, ...]

    @staticmethod
    def to_image_coords(boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].
        
        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
        
        return box_coords

    def draw_boxes(self, image, boxes, classes, thickness=4):
        """Draw bounding boxes on the image"""
        draw = ImageDraw.Draw(image)
        for i in range(len(boxes)):
            bot, left, top, right = boxes[i, ...]
            color = self.COLOR_LIST[int(classes[i]) - 1]
            draw.line([(left, top), (left, bot), (right, bot), (right, top), (left, top)], 
                      width=thickness, fill=color)
    
    @staticmethod
    def load_graph(graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph