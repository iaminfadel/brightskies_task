import sys, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
bag=sys.argv[1]
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
n=0
while r.has_next():
    topic,data,t=r.read_next()
    if topic!='/carla/hero/lidar': continue
    m=deserialize_message(data,PointCloud2)
    pts=point_cloud2.read_points_numpy(m,field_names=('x','y','z','intensity'),skip_nans=True)
    if pts.shape[0]<100: continue
    print('stamp',m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'frame',m.header.frame_id,'n',pts.shape[0])
    for i,name in enumerate('xyz'):
        print(name, np.percentile(pts[:,i],[0,1,5,50,95,99,100]))
    print('intensity', np.percentile(pts[:,3],[0,50,100]))
    n+=1
    if n>=3: break
