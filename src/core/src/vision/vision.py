import numpy as np
import cv2
from module import Module

class Colorspace_transform(Module):
    self.transforms = {"RGB2BGR": cv2.COLOR_RGB2BGR,
                       "RGB2GRAY": cv2.COLOR_RGB2GRAY,
                       "GRAY2RGB": cv2.COLOR_GRAY2RGB,
                       "RGB2HSV": cv2.COLOR_RGB2HSV,
                       "HSV2RGB": cv2.COLOR_HSV2RGB,
                       "RGB2YCrCb": cv2.COLOR_RGB2YCrCb,
                       "YCrCb2RGB": cv2.COLOR_YCrCb2RGB,
                      }

    def __init__(self, params):
        super.__init__(self, params, "Colorspace_transform")

        self.result = np.zeros((3,3,3), np.uint8)

        self.source_colorspace = params["from"]
        self.target_colorspace = params["to"]

    def apply(self, input):
        inp_listed = list(input.items())

        if (len(inp_listed) > 0):
            _, input_ = list(input.items())[0]
            _, input__ = list(input_.items())[0]

            self.work(input__)

        else:
            self.result = np.ones((20, 20, 3), np.uint8) * 12
            
        return self.result

    def work(self, inp):
        if self.source_colorspace == self.target_colorspace:
            self.result = inp

        else:
            self.result = cv2.cvtColor(inp, self.transforms[self.source_colorspace +
                                                            "2" + self.target_colorspace])
	
        return self.result


class Inrange(Module):
    def __init__(self, params):
        super().__init__(params, "Inrange")

        self.set_ths(params["low_th"], params["high_th"])

    def apply(self, input):
        _, input_ = list(input.items())[0]

        self.result = cv2.inRange(input_, tuple(self.low_th), tuple(self.high_th))

        return self.result

    def draw(self, img):
        return cv2.cvtColor(self.result, cv2.COLOR_GRAY2BGR)

    def set_ths(self, low_th_, high_th_):
        self.low_th = list(low_th_)
        self.high_th = list(high_th_)

class Filter_connected_components (Filter):
    def __init__(self, area_low_=-1, area_high_=-1, hei_low_=-1, hei_high_=-1,
                 wid_low_=-1, wid_high_=-1, den_low_=-1, den_high_=-1):
        Filter.__init__(self, "Filter_connected_components")

        self.area_low = area_low_
        self.area_high = area_high_
        self.hei_low = hei_low_
        self.hei_high = hei_high_
        self.wid_low = wid_low_
        self.wid_high = wid_high_
        self.den_low = den_low_
        self.den_high = den_high_

    def apply(self, img):  # , area_low = -1, area_high = -1, hei_low = -1, hei_high = -1,
        # wid_low = -1, wid_high = -1, den_low = -1, den_high = -1):
        return image_processing.filter_connected_components(img, self.area_low,
                                                            self.area_high, self.hei_low, self.hei_high, self.wid_low,
                                                            self.wid_high, self.den_low, self.den_high)

    def apply(self, img):
        result = np.array(mask)
        output = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)

        labels_count = output[0]
        labels = output[1]
        stats = output[2]
        sz = stats.shape[0]

        def _in_range(value, low, high):
            if ((value < low  and low  != -1) or
                (value > high and high != -1)):
                return False

            return True

        for label_num in range(0, sz):
            area    = stats[label_num, cv2.CC_STAT_AREA]
            height  = stats[label_num, cv2.CC_STAT_HEIGHT]
            width   = stats[label_num, cv2.CC_STAT_WIDTH]
            density = float(area) / (height * width)
        
            if (_in_range(area,    self.area_low, self.area_high) == False or
                _in_range(height,  self.hei_low,  self.hei_high)  == False or
                _in_range(width,   self.wid_low,  self.wid_high)  == False or
                _in_range(density, self.den_low,  self.den_high)  == False):
            result[labels == label_num] = 0

    return result

class Max_area_cc_bbox(Module):
    def __init__(self, params):
        super().__init__(params, "Max_area_cc_bbox")

        self.bbox_num = params["bbox_num"]

    def apply(self, input):
        _, input_ = list(input.items())[0]

        self.result, _ = self.find_max_bounding_box(input_, self.bbox_num)

        return self.result

    def draw(self, img):
        res = img.copy()

        for s in self.result:
            res = cv2.rectangle(
                res, s[0], s[1], (100, 200, 10), 5)

        return res

    def find_max_bounding_box(self, mask, bbox_num):
        output = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)
        stats = output[2]

        success = True

        sorted_components = stats[np.argsort(stats[:, cv2.CC_STAT_AREA])]
        sorted_components = sorted_components[: -1]
        sorted_components = sorted_components[- min(
            bbox_num, len(sorted_components)):]

        result = []

        for i in range(len(sorted_components)):
            top = sorted_components[i, cv2.CC_STAT_TOP]
            left = sorted_components[i, cv2.CC_STAT_LEFT]
            width = sorted_components[i, cv2.CC_STAT_WIDTH]
            height = sorted_components[i, cv2.CC_STAT_HEIGHT]

            result.append(((left, top), (left + width, top + height)))

        return result, success
