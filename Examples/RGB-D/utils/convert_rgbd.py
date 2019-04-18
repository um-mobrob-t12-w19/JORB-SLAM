import os, sys
import os.path as osp
import cv2
from cv_bridge import CvBridge
import rosbag
import sensor_msgs
import sensor_msgs.point_cloud2 as pc
import zipfile
import tqdm

def convert(filepath, timeoffset=0): # Time offset is in seconds, added to the timestamps
    output = zipfile.ZipFile(filepath[:-4] + ".zip", 'w', allowZip64=True)
    # color_counter = 0
    # depth_counter = 0
    br = CvBridge()

    print("Loading rosbag...")

    with rosbag.Bag(filepath, 'r') as bag:
        depth_topic = "/device_0/sensor_0/Depth_0/image/data"
        depth_timelist = []
        color_topic = "/device_0/sensor_1/Color_0/image/data"
        color_timelist = []
        print("Saving images...")
        with tqdm.tqdm(total=bag.get_end_time(), unit="s") as tl:
            for topic, data, time in bag.read_messages(topics=[depth_topic, color_topic]):
                timest = time.secs + time.nsecs/1e9
                timestr = "%.6f" % (timest + timeoffset)
                if topic == depth_topic:
                    ret, buf = cv2.imencode(".png", br.imgmsg_to_cv2(data, desired_encoding="mono16"))
                    if ret != True: raise RuntimeError("Error encoding image!")
                    output.writestr("depth/%s.png" % timestr, buf)
                    depth_timelist.append("{0} depth/{0}.png".format(timestr))
                if topic == color_topic:
                    ret, buf = cv2.imencode(".png", br.imgmsg_to_cv2(data, desired_encoding="bgr8"))
                    if ret != True: raise RuntimeError("Error encoding image!")
                    output.writestr("rgb/%s.png" % timestr, buf)
                    color_timelist.append("{0} rgb/{0}.png".format(timestr))
                tl.n = timest
                tl.refresh()
    timeheader = '''# %s images
# file: '{0}'
# timestamp filename
'''.format(filepath)

    output.writestr("depth.txt", timeheader % 'depth maps' + '\n'.join(depth_timelist))
    output.writestr("rgb.txt", timeheader % 'color images' + '\n'.join(color_timelist))
    output.close()
    print("Done.")

if __name__ == "__main__":
    if len(sys.argv) == 3:
        convert(sys.argv[1], float(sys.argv[2]))
    elif len(sys.argv) == 2:
        convert(sys.argv[1])
    else:
        print("Usage: python convert_rgbd <path/to/bag> [time_offset]")

