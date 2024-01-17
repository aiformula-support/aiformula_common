import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import matplotlib.pyplot as plt

class BevCamera(Node):
    def __init__(self):
        super().__init__('bev_camera')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.callback_points,10)
        self.right_points_pub = self.create_publisher(PointCloud2, 'line_points/right', 10)
        self.center_points_pub = self.create_publisher(PointCloud2, 'line_points/center', 10)
        self.left_points_pub = self.create_publisher(PointCloud2, 'line_points/left', 10)
        # self.line_points_pub = self.create_publisher(PointCloud2, 'line_points', 10)

        self.odom = Odometry()
        self.right_points = []
        self.center_points = []
        self.left_points = []
        self.line_points = []

    def callback_points(self, data):
        self.odom = data

        curve_1 = np.array([[-19.15, 5.1, 0.001]
                                ,[-21.05, 5.017, 0.001]
                                ,[-22.936, 4.769, 0.001]
                                ,[-24.792, 4.357, 0.001]
                                ,[-26.606, 3.785, 0.001]
                                ,[-28.363, 3.058, 0.001]
                                ,[-30.05, 2.179, 0.001]
                                ,[-31.654, 1.158, 0.001]
                                ,[-33.163, 0, 0.001]
                                ,[-34.565, -1.285, 0.001]
                                ,[-35.85, -2.687, 0.001]
                                ,[-37.008, -4.196, 0.001]
                                ,[-38.029, -5.8, 0.001]
                                ,[-38.908, -7.487, 0.001]
                                ,[-39.635, -9.244, 0.001]
                                ,[-40.207, -11.058, 0.001]
                                ,[-40.619, -12.914, 0.001]
                                ,[-40.867, -14.8, 0.001]
                                ,[-40.95, -16.7, 0.001]])
        straight_1=np.array([
                                [-40.95, -17.7, 0.001]
                                ,[-40.95, -18.7, 0.001]
                                ,[-40.95, -19.7, 0.001]
                                ,[-40.95, -20.7, 0.001]
                                ,[-40.95, -21.7, 0.001]
                                ,[-40.95, -22.7, 0.001]
                                ,[-40.95, -23.7, 0.001]
                                ,[-40.95, -24.7, 0.001]
                                ,[-40.95, -25.7, 0.001]
                                ,[-40.95, -26.7, 0.001]
                                ,[-40.95, -27.7, 0.001]
                                ,[-40.95, -28.7, 0.001]
                                ,[-40.95, -29.7, 0.001]
                                ,[-40.95, -30.7, 0.001]
                                ,[-40.95, -31.7, 0.001]
                                ,[-40.95, -32.7, 0.001]
                                ,[-40.95, -33.7, 0.001]
                                ,[-40.95, -34.7, 0.001]
                                ,[-40.95, -35.7, 0.001]
                                ,[-40.95, -36.7, 0.001]
        ])

        curve_2 = np.array([
                                [-40.95, -37.7, 0.001]
                                ,[-40.867, -39.6, 0.001]
                                ,[-40.619, -41.486, 0.001]
                                ,[-40.207, -43.342, 0.001]
                                ,[-39.635, -45.156, 0.001]
                                ,[-38.908, -46.913, 0.001]
                                ,[-38.029, -48.6 ,0.001]
                                ,[-37.008 ,-50.204 ,0.001]
                                ,[-35.85, -51.713, 0.001]
                                ,[-34.565, -53.115, 0.001]
                                ,[-33.163, -54.4 ,0.001]
                                ,[-31.654, -55.558 ,0.001]
                                ,[-30.05, -56.579 ,0.001]
                                ,[-28.363 ,-57.458 ,0.001]
                                ,[-26.606, -58.185, 0.001]
        ])

        straight_2 = np.array([
                                [-25, -58.835, 0.001]
                                ,[-24, -59.241, 0.001]
                                ,[-23, -59.646, 0.001]
                                ,[-22, -60.051, 0.001]
                                ,[-21, -60.456, 0.001]
                                ,[-20, -60.862, 0.001]
                                ,[-19, -61.267, 0.001]
                                ,[-18, -61.672, 0.001]
                                ,[-17, -62.077, 0.001]
                                ,[-16, -62.483, 0.001]
                                ,[-15, -62.888, 0.001]
                                ,[-14, -63.293, 0.001]
                                ,[-13, -63.698, 0.001]
                                ,[-12, -64.104, 0.001]
                                ,[-11, -64.509, 0.001]
                                ,[-10, -64.914, 0.001]
                                ,[-9, -65.319, 0.001]
                                ,[-8, -65.725, 0.001]
                                ,[-7, -66.130, 0.001]
                                ,[-6, -66.535, 0.001]
                                ,[-5, -66.940, 0.001]
                                ,[-4, -67.346, 0.001]
                                ,[-3, -67.751, 0.001]
                                ,[-2, -68.156, 0.001]
                                ,[-1, -68.562, 0.001]
                                ,[0, -68.967, 0.001]
                                ,[1, -69.372, 0.001]
                                ,[2, -69.777, 0.001]
                                ,[3, -70.183, 0.001]
                                ,[4, -70.588, 0.001]
                                ,[5, -70.993, 0.001]
                                ,[6, -71.398, 0.001]
                                ,[7, -71.804, 0.001]
                                ,[8, -72.209, 0.001]
                                ,[9, -72.614, 0.001]
                                ,[10, -73.019, 0.001]
                                ,[11, -73.425, 0.001]
                                ,[12, -73.830, 0.001]
                                ,[13, -74.235, 0.001]
                                ,[14, -74.640, 0.001]
                                ,[15, -75.046, 0.001]
                                ,[16, -75.451, 0.001]
                                ,[17, -75.856, 0.001]
                                ,[18, -76.261, 0.001]
        ])
        curve_3 = np.array([  
                                [19.044, -76.685, 0.001]
                                ,[20.858, -77.257 ,0.001]
                                ,[22.714, -77.669 ,0.001]
                                ,[24.6, -77.917 ,0.001]
                                ,[26.5 ,-78 ,0.001]
                                ,[28.4 ,-77.917 ,0.001]
                                ,[30.286 ,-77.669 ,0.001]
                                ,[32.142, -77.257 ,0.001]
                                ,[33.956 ,-76.685 ,0.001]
                                ,[35.713 ,-75.958 ,0.001]
                                ,[37.4 ,-75.079 ,0.001]
                                ,[39.004 ,-74.058 ,0.001]
                                ,[40.513, -72.9 ,0.001]
                                ,[41.915, -71.615 ,0.001]
                                ,[43.2 ,-70.213 ,0.001]
                                ,[44.358 ,-68.704 ,0.001]
                                ,[45.379 ,-67.1 ,0.001]
                                ,[46.258 ,-65.413 ,0.001]
                                ,[46.985, -63.656, 0.001]
                                ,[47.557, -61.842 ,0.001]
                                ,[47.969, -59.986 ,0.001]
                                ,[48.217, -58.1 ,0.001]
        ])
        straight_3 = np.array([
                                [48.3 ,-56.2 ,0.001]
                                ,[48.3 ,-55.2 ,0.001]
                                ,[48.3 ,-54.2 ,0.001]
                                ,[48.3 ,-53.2 ,0.001]
                                ,[48.3 ,-52.2 ,0.001]
                                ,[48.3 ,-51.2 ,0.001]
                                ,[48.3 ,-50.2 ,0.001]
                                ,[48.3 ,-49.2 ,0.001]
                                ,[48.3 ,-48.2 ,0.001]
                                ,[48.3 ,-47.2 ,0.001]
                                ,[48.3 ,-46.2 ,0.001]
                                ,[48.3 ,-45.2 ,0.001]
                                ,[48.3 ,-44.2 ,0.001]
                                ,[48.3 ,-43.2 ,0.001]
                                ,[48.3 ,-42.2 ,0.001]
                                ,[48.3 ,-41.2 ,0.001]
                                ,[48.3 ,-40.2 ,0.001]
                                ,[48.3 ,-39.2 ,0.001]
                                ,[48.3 ,-38.2 ,0.001]
                                ,[48.3 ,-37.2 ,0.001]
                                ,[48.3 ,-36.2 ,0.001]
                                ,[48.3 ,-35.2 ,0.001]
                                ,[48.3 ,-34.2 ,0.001]
                                ,[48.3 ,-33.2 ,0.001]
                                ,[48.3 ,-32.2 ,0.001]
                                ,[48.3 ,-31.2 ,0.001]
                                ,[48.3 ,-30.2 ,0.001]
                                ,[48.3 ,-29.2 ,0.001]
                                ,[48.3 ,-28.2 ,0.001]
                                ,[48.3 ,-27.2 ,0.001]
                                ,[48.3 ,-26.2 ,0.001]
                                ,[48.3 ,-25.2 ,0.001]
                                ,[48.3 ,-24.2 ,0.001]
                                ,[48.3 ,-23.2 ,0.001]
                                ,[48.3 ,-22.2 ,0.001]
                                ,[48.3 ,-21.2 ,0.001]
                                ,[48.3 ,-20.2 ,0.001]
                                ,[48.3 ,-19.2 ,0.001]
                                ,[48.3 ,-18.2 ,0.001]
                                ,[48.3 ,-17.2 ,0.001]
                                ,[48.3 ,-16.7 ,0.001]
        ])

        curve_4 = np.array([
                                [48.217 ,-14.8 ,0.001]
                                ,[47.969 ,-12.914 ,0.001]
                                ,[47.557 ,-11.058 ,0.001]
                                ,[46.985 ,-9.244 ,0.001]
                                ,[46.258 ,-7.487 ,0.001]
                                ,[45.379, -5.8 ,0.001]
                                ,[44.358, -4.196 ,0.001]
                                ,[43.2, -2.687 ,0.001]
                                ,[41.915, -1.285 ,0.001]
                                ,[40.513, 0 ,0.001]
                                ,[39.004 ,1.158 ,0.001]
                                ,[37.4 ,2.179 ,0.001]
                                ,[35.713, 3.058 ,0.001]
                                ,[33.956, 3.785 ,0.001]
                                ,[32.142, 4.357 ,0.001]
                                ,[30.286, 4.769 ,0.001]
                                ,[28.4, 5.017 ,0.001]
                                ,[26.5, 5.1 ,0.001]
        ])

        straight_4 = np.array([                      
                                [25.5, 5.1 ,0.001]
                                ,[24.5, 5.1 ,0.001]
                                ,[23.5, 5.1 ,0.001]
                                ,[22.5, 5.1 ,0.001]
                                ,[21.5, 5.1 ,0.001]
                                ,[20.5, 5.1 ,0.001]
                                ,[19.5, 5.1 ,0.001]
                                ,[18.5, 5.1 ,0.001]
                                ,[17.5, 5.1 ,0.001]
                                ,[16.5, 5.1 ,0.001]
                                ,[15.5, 5.1 ,0.001]
                                ,[14.5, 5.1 ,0.001]
                                ,[13.5, 5.1 ,0.001]
                                ,[12.5, 5.1 ,0.001]
                                ,[11.5, 5.1 ,0.001]
                                ,[10.5, 5.1 ,0.001]
                                ,[9.5, 5.1 ,0.001]
                                ,[8.5, 5.1 ,0.001]
                                ,[7.5, 5.1 ,0.001]
                                ,[6.5, 5.1 ,0.001]
                                ,[5.5, 5.1 ,0.001]
                                ,[4.5, 5.1 ,0.001]
                                ,[3.5, 5.1 ,0.001]
                                ,[2.5, 5.1 ,0.001]
                                ,[1.5, 5.1 ,0.001]
                                ,[0.5, 5.1 ,0.001]
                                ,[-1.5, 5.1 ,0.001]
                                ,[-2.5, 5.1 ,0.001]
                                ,[-3.5, 5.1 ,0.001]
                                ,[-4.5, 5.1 ,0.001]
                                ,[-5.5, 5.1 ,0.001]
                                ,[-6.5, 5.1 ,0.001]
                                ,[-7.5, 5.1 ,0.001]
                                ,[-8.5, 5.1 ,0.001]
                                ,[-9.5, 5.1 ,0.001]
                                ,[-10.5, 5.1 ,0.001]
                                ,[-11.5, 5.1 ,0.001]
                                ,[-12.5, 5.1 ,0.001]
                                ,[-13.5, 5.1 ,0.001]
                                ,[-14.5, 5.1 ,0.001]
                                ,[-15.5, 5.1 ,0.001]
                                ,[-16.5, 5.1 ,0.001]
                                ,[-17.5, 5.1 ,0.001]
                                ,[-18.5, 5.1 ,0.001]
                                ])
        #center line
        center_points = []

        for i in range(len(straight_1)): #right
            point_x = straight_1[i][0] +3.5
            point_y = straight_1[i][1]
            point_z = 0.001
            center_points.append([point_x,point_y,point_z])
 
        for i in range(len(straight_2)):
            point_x = straight_2[i][0]
            point_y = -0.4052573932092 * point_x -68.967278203724 +3.5
            point_z = 0.001
            center_points.append([point_x,point_y,point_z])
            
        for i in range(len(straight_3)): #left
            point_x = straight_3[i][0] -3.5
            point_y = straight_3[i][1]
            point_z = 0.001
            center_points.append([point_x,point_y,point_z])

        for i in range(len(straight_4)): #under
            point_x = straight_4[i][0]
            point_y = straight_4[i][1] - 3.5
            point_z = 0.001
            center_points.append([point_x,point_y,point_z])

        for i in range(len(curve_1)):
            point_x = curve_1[i][0] +3.5
            point_y = np.sqrt(abs(18.5**2 - (point_x+19.15)**2)) -16.7
            point_z = 0.001
            center_points.append([point_x, point_y, point_z])

        for i in range(len(curve_2)):
            point_x = curve_2[i][0] + 3.5 
            point_y = np.sqrt(abs(18.5**2 - (point_x+19.15)**2))+37.7
            point_z = 0.001
            center_points.append([point_x, -point_y, point_z])

        for i in range(len(curve_3)):
            point_x = curve_3[i][0] - 3.5 
            point_y = np.sqrt(abs(18.5**2 - (point_x-26.5)**2))+56.2
            point_z = 0.001
            center_points.append([point_x, -point_y, point_z])
 
        for i in range(len(curve_4)):
            point_x = curve_4[i][0] -3.5 
            point_y = np.sqrt(abs(18.5**2 - (point_x-26.15)**2))-16.7
            point_z = 0.001
            center_points.append([point_x, point_y, point_z])

        #left line
        left_points = [] 
        for i in range(len(straight_1)): #right
            point_x = straight_1[i][0] +7.0
            point_y = straight_1[i][1]
            point_z = 0.001
            left_points.append([point_x,point_y,point_z])
 
        for i in range(len(straight_2)):
            point_x = straight_2[i][0]
            point_y = -0.4052573932092 * point_x -68.967278203724 +7.0
            point_z = 0.001
            left_points.append([point_x,point_y,point_z])
            
        for i in range(len(straight_3)): #left
            point_x = straight_3[i][0] -7.0
            point_y = straight_3[i][1]
            point_z = 0.001
            left_points.append([point_x,point_y,point_z])

        for i in range(len(straight_4)): #under
            point_x = straight_4[i][0]
            point_y = straight_4[i][1] - 7.0
            point_z = 0.001
            left_points.append([point_x,point_y,point_z])

        for i in range(len(curve_1)):
            point_x = curve_1[i][0] +7.0
            point_y = np.sqrt(abs(15**2 - (point_x+19.15)**2)) -16.7
            point_z = 0.001
            left_points.append([point_x, point_y, point_z])

        for i in range(len(curve_2)):
            point_x = curve_2[i][0] + 7.0
            point_y = np.sqrt(abs(15**2 - (point_x+19.15)**2))+37.7
            point_z = 0.001
            left_points.append([point_x, -point_y, point_z])

        for i in range(len(curve_3)):
            point_x = curve_3[i][0] - 7.0
            point_y = np.sqrt(abs(15**2 - (point_x-26.5)**2))+56.2
            point_z = 0.001
            left_points.append([point_x, -point_y, point_z])
 
        for i in range(len(curve_4)):
            point_x = curve_4[i][0] -7.0
            point_y = np.sqrt(abs(15**2 - (point_x-26.15)**2))-16.7
            point_z = 0.001
            left_points.append([point_x, point_y, point_z])

        #right line
        right_points = []
        right_points.extend(straight_1)
        right_points.extend(straight_2)
        right_points.extend(straight_3)
        right_points.extend(straight_4)
        right_points.extend(curve_1)
        right_points.extend(curve_2)
        right_points.extend(curve_3)
        right_points.extend(curve_4)
        
        #transfer to pointcloud
        self.right_points = np.array(right_points)
        self.center_points = np.array(center_points)
        self.left_points = np.array(left_points)
       
        self.right_points = point_cloud(self.right_points,'odom')
        self.center_points = point_cloud(self.center_points,'odom')
        self.left_points = point_cloud(self.left_points,'odom')

        self.right_points_pub.publish(self.right_points)
        self.center_points_pub.publish(self.center_points)
        self.left_points_pub.publish(self.left_points)

        # line_points = []
        # line_points.extend(right_points)
        # line_points.extend(center_points)
        # line_points.extend(left_points)
        # self.line_points = np.array(line_points)
        # self.line_points = point_cloud(np.array(line_points),'odom')
        # self.line_points_pub.publish(self.line_points)

def point_cloud(points, parent_frame):
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
)    
            
def main(args=None):
   rclpy.init(args=args)
   bev_camera = BevCamera()
   rclpy.spin(bev_camera)
   
   bev_camera.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
