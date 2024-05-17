import cv2

class Map():
    def __init__(self, map, width, height, resolution):
        self.data = map
        self.width = width
        self.height = height
        self.resolution = resolution

    @staticmethod
    def load_map(filename):
        im_gray = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        (thresh, im_bw) = cv2.threshold(im_gray, 128, 255, cv2.THRESH_BINARY)
        return Map(im_bw, im_bw.shape[0], im_bw.shape[1], 1)