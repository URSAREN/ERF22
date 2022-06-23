import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np

gmap_publishing_topic = '/map'
gmap_height = int(rospy.get_param("~map_size", 400))
gmap_width = int(rospy.get_param("~map_size", 400))
gmap_resolution = rospy.get_param("~map_resolution", .025)


def occupancygrid_to_numpy(msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

        return np.ma.array(data, mask=data==-1, fill_value=-1)

def numpy_to_occupancy_grid(arr, info=None):
        if not len(arr.shape) == 2:
                raise TypeError('Array must be 2D')
        if not arr.dtype == np.int8:
                raise TypeError('Array must be of int8s')

        grid = OccupancyGrid()
        if isinstance(arr, np.ma.MaskedArray):
                # We assume that the masked value are already -1, for speed
                arr = arr.data
        grid.data = arr.ravel()
        grid.info = info or MapMetaData()
        grid.info.height = arr.shape[0]
        grid.info.width = arr.shape[1]

        return grid


def callback(data):
    arr = occupancygrid_to_numpy(data)
    np.savetxt('/home/alex/Desktop/gmap.cvs', arr)
    print('Saved map.')
    
def listener():
    rospy.init_node('gmap_listener', anonymous=True)

    rospy.Subscriber(gmap_publishing_topic, OccupancyGrid, callback)

    rospy.spin()
    

if __name__ == '__main__':
    listener()