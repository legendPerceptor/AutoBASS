import time
import os
import logging
import threading
import cv2 as cv
import numpy as np
import json

ORG_IMG = True

logger = logging.getLogger("OffsetDetection")

CONFIG = dict(
CAM_PORT_BOTM = 1,

CAM_PORT_TOP = 2,

Anode_Drop=dict(name='Anode_Drop', diam=15, ksize=5, minDist=100, param1=100, param2=10, minR=80, maxR=85),

Anode_Grab=dict(name='Anode_Grab', diam=15, ksize=5, minDist=300, param1=120, param2=15, minR=115, maxR=135),

Cathode_Drop=dict(name='Cathode_Drop', diam=14, ksize=5, minDist=100, param1=100, param2=10, minR=75, maxR=80),

Cathode_Grab=dict(name='Cathode_Grab', diam=14, ksize=5, minDist=300, param1=120, param2=15, minR=115, maxR=125),

Separator_Drop=dict(name='Separator_Drop', diam=15.5, ksize=5, minDist=100, param1=120, param2=15, minR=85, maxR=93),

Separator_Grab=dict(name='Separator_Grab', diam=15.5, ksize=5, minDist=500, param1=120, param2=15, minR=135, maxR=145),

Anode_Spacer_Grab=dict(name='Anode_Spacer', diam=15.5, ksize=5, minDist=100, param1=120, param2=15, minR=135, maxR=140),

Cathode_Spacer_Grab=dict(name='Cathode_Spacer', diam=15.5, ksize=5, minDist=100, param1=120, param2=15, minR=135, maxR=140),

Cathode_Case_Grab=dict(name='Cathode_Case', diam=19.3, ksize=5, minDist=100, param1=120, param2=15, minR=187, maxR=195),

Reference=dict(name='Reference', diam=2, ksize=5, minDist=100, param1=120, param2=20, minR=8, maxR=15),

Suction_Cup=dict(name='Suction_Cup', diam=4, ksize=5, minDist=500, param1=120, param2=20, minR=50, maxR=60),

Customize=dict(name='Customize', diam=2, ksize=5, minDist=100, param1=100, param2=10, minR=110, maxR=115),
)

class AutoCorrection():

    def __init__(self) -> None:
        # self.caseOffset = np.array([0,0,0,0,0,0], dtype=np.float32)
        pass

    def detect_object_center(self, img, object_config:dict):
        img_color = np.copy(img)
        img_gray = cv.cvtColor(img_color, cv.COLOR_BGR2GRAY)
        img_gray = cv.medianBlur(img_gray, 5)

        circles = cv.HoughCircles(img_gray, cv.HOUGH_GRADIENT, 1, object_config['minDist'],
                                param1=object_config['param1'], param2=object_config['param2'],
                                minRadius=object_config['minR'], maxRadius=object_config['maxR'])
        h, w = img.shape[:2]
        found_circles = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for c in circles[0, :]:
                x, y, r = int(c[0]), int(c[1]), int(c[2])
                if x-r<=0 or y-r<=0 or x+r>=w or y+r>=h:
                    logger.warning("Detected circle out of scope discard result!")
                else:
                    found_circles.append([[x, y], r])
            found_circles = sorted(found_circles, key=lambda x:x[1])
            logger.debug(f"sorted circles: {found_circles}")
            found_circles = found_circles[-1] if len(found_circles) > 0 else []
        return found_circles

    def project_to_3d(self, image_coordinates, H_mtx): 
        """
        This method takes the Homography matrix and the 2d image cartesian coordinates. It returns the (x, y)
        cartesian coordinates in 3d cartesian world coordinates on floor plane(at z=0). Notice that z coordinate is omitted
        here and added inside the tracking funtion. 
        
        Parameters
        ----------
        image_coordinates: 2d pixel coordinates (x,y)
        h: 3x3 Homography matrix np.array[3x3]
        Returns
        ----------
        floor_coordinates: List of x, y coordinates in 3d world of same pixel on floor plane i.e. (x,y,z) Considering z=0 and 
        ommitted here.
        """
        # if H_mtx == None:
        #     os.chdir(os.path.join(PATH, 'camera_data'))
        #     H_mtx = np.load('H_mtx.npy')
        #adding 1 for homogenous coordinate system
        x, y, w = H_mtx @ np.array([[*image_coordinates, 1]]).T
        X, Y = np.around(x/w, decimals=3), np.around(y/w, decimals=3) # Transform homogenous coordinates into cart coordinates
        return np.array([X, Y, 0, 0, 0, 0], dtype=np.float32)

    def draw_detection(self, img, found_circle, text:str=None):
        img_output = np.copy(img)
        h, w = img_output.shape[:2]
        img_center = (w//2, h//2)
        cv.drawMarker(img_output, img_center, (0,255,0), cv.MARKER_CROSS, 10, 1)
        logger.debug(f"found_circles:{found_circle}")
        center = found_circle[0]
        cv.drawMarker(img_output, center, (0,0,255), cv.MARKER_CROSS, 10, 1)
        cv.circle(img_output, center, found_circle[1], (255, 0, 255), 1)
        if text:
            cv.putText(img_output, text, (20, h-20),
                    cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        return img_output

    def get_offset(self, img, component, state):
        with open(os.path.join(os.path.dirname(__file__), "data", "calibration.json"), "r") as json_file:
            H_mtx = np.array(json.load(json_file)[f"H_mtx_{state}"], dtype=np.float32)
        # H_mtx = CONFIG['H_mtx'][state]
        detectedObj = self.detect_object_center(img, CONFIG[f'{component}_{state}'])
        if len(detectedObj) > 0:
            xy = self.project_to_3d(detectedObj[0], H_mtx)
            # if component == 'Anode' and state == 'Drop':
            #     self.caseOffset = xy
            img_output = self.draw_detection(img, detectedObj, f'OffSet: {xy[:2]}')
            if component == "Separator" and state == 'Grab':
                correction = np.array([0,0,0,0,0,0], dtype=np.float32)
            else:
                correction = xy*np.array([-1,-1,0,0,0,0], dtype=np.float32)
            # if component == "Cathode" and state == 'Grab':
            #     correction = correction + self.caseOffset
            logger.info(f"{state} Offset detected: {xy}")
            return img_output, correction, True
        else:
            detectedObj = self.detect_object_center(img, CONFIG['Suction_Cup'])
            if len(detectedObj) > 0:
                logger.info(f"Object {component} failed being grabed")
                return img, np.array([0,0,0,0,0,0], dtype=np.float32), False
            else:
                logger.info(f"Object {component} failed detecting")
                return img, np.array([0,0,0,0,0,0], dtype=np.float32), True

    def take_img(self, state:str, component:str='', nr:int=None):
        self.time_stamp = time.strftime("%Y_%m_%d_%Hh_%Mm_%Ss", time.localtime())
        self.dir_name = os.path.join(os.path.dirname(__file__), 'Alignments', self.time_stamp[:10], f"Cell[{nr}]")
        org_dir = os.path.join(os.path.dirname(__file__), 'Alignments', self.time_stamp[:10], "Origin", f"{component}_{state}")
        # org_filename = os.path.join(org_dir, f"[No{nr}]_{component}_{state}_{self.time_stamp}.jpg")
        org_filename = os.path.join(org_dir, f"[No{nr}]_{component}_{state}_{self.time_stamp[:10]}.jpg")
        if state == 'Grab':
            alpha = 1.0 # Simple contrast control [1.0-3.0]
            beta = 5    # Simple brightness control [0-100]
            cam = cv.VideoCapture(CONFIG['CAM_PORT_BOTM'],cv.CAP_DSHOW)
            ret, img = cam.read()
        elif state == 'Drop':
            alpha = 2.0 # Simple contrast control [1.0-3.0]
            beta = 50    # Simple brightness control [0-100]
            cam = cv.VideoCapture(CONFIG['CAM_PORT_TOP'],cv.CAP_DSHOW)
            ret, img = cam.read()
            ret, img = cam.read()
            time.sleep(0.5)
        if ret == True:
            img = cv.convertScaleAbs(img, alpha=alpha, beta=beta)
            img_output = np.copy(img)
            img_output = cv.normalize(img_output, None, 0, 255, cv.NORM_MINMAX)
        else:
            logger.error(f"Camera {state} has lost its connection", exc_info=True)
        cam.release()
        if not os.path.exists(self.dir_name):
            os.makedirs(self.dir_name)
        if not os.path.exists(org_dir):
            os.makedirs(org_dir)
        if ORG_IMG:
            cv.imwrite(org_filename, img_output)
        return img

    def show_img(self, img, component, nr):
        cv.imshow(f'{component} NO.[{nr}]', img)
        cv.waitKey(2000)
        cv.destroyAllWindows()

    def run_autocorrection(self, state:str, component:str, nr:int=None, show:bool=False, save:bool=False):
        img = self.take_img(state, component, nr)
        img_output, correction, grabed = self.get_offset(img, component, state)
        if save == True and grabed == True:
            res_filename = os.path.join(self.dir_name, f"[No{nr}]_{component}_{state}_{self.time_stamp}.jpg")
            cv.imwrite(res_filename, img_output)
        if show:
            threading.Thread(name='show_image', target=self.show_img, args=(img_output, component, nr,), daemon=True).start()
        return correction, grabed
