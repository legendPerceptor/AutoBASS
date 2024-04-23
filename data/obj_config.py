import numpy as np

OBJECT_LIST = ('Anode', 'Anode_Grab', 'Cathode', 'Cathode_Grab', 'Anode_Spacer', 'Cathode_Spacer', 'Cathode_Case', 'Suction_Cup', 'Customize')

TOLERANCE = (0.2, 0,2)

CONFIG = dict(
CAM_PORT_BOTM = 1,

CAM_PORT_TOP = 2,

H_mtx = dict(Grab=np.array([[ 6.45098999e-02, -1.02565681e-03, -2.05041870e+01],
       [ 1.55206759e-03,  6.40013817e-02, -1.58372091e+01],
       [-1.64628068e-04,  2.22208647e-04,  1.00000000e+00]], dtype=np.float32),

            Drop=np.array([[ 1.06852913e-01, -1.33552234e-03, -3.40491577e+01],
       [-2.67118394e-03, -1.06847357e-01,  2.66103344e+01],
       [ 2.22361027e-04, -1.11269626e-05,  1.00000000e+00]], dtype=np.float32),),

cam_mtx = np.array([[1.04836761e+03, 0.00000000e+00, 2.96619370e+02],
       [0.00000000e+00, 1.40258193e+03, 2.56415830e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float32),

dist = np.array([[ 8.46838653e-01, -1.16566443e+01,  2.81273876e-03,
         5.70453427e-02,  2.82609343e+01]], dtype=np.float32),

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