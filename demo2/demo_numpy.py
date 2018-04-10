import numpy as np
import matplotlib.pyplot as plt

# First import the library
import pyrealsense2 as rs

# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
pipeline.start()


while True:
    # This call waits until a new coherent set of frames is available on a device
    # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    depth_data = depth.as_frame().get_data()
    np_image = np.asanyarray(depth_data)
    np.set_printoptions(precision=3)
    print(np_image)
    plt.pcolormesh(np.flipud(np_image)/10,vmin=0.0,vmax=1000.0,cmap=plt.cm.jet)
    plt.colorbar()
    #plt.savefig('asdf.png')
    plt.show()