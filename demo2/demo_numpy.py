import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# First import the library
import pyrealsense2 as rs

# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
pipeline.start()

f = plt.figure()
ax = f.add_subplot(111)
frames = pipeline.wait_for_frames()
depth = frames.get_depth_frame()
depth_data = depth.as_frame().get_data()
np_image = np.asanyarray(depth_data)
pmesh = ax.imshow((np_image)/100,vmin=0.0,vmax=100.0,cmap=plt.cm.gray)
f.colorbar(pmesh)

def animate(iter):
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    depth_data = depth.as_frame().get_data()
    np_image = np.asanyarray(depth_data)
    pmesh.set_array(np_image/100)

print('asdf')
anim = animation.FuncAnimation(f,animate,frames=1000,interval=50,blit=False,repeat=False)
plt.show()

# while True:
#     # This call waits until a new coherent set of frames is available on a device
#     # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
    
#     ax.imshow((np_image)/100,vmin=0.0,vmax=100.0,cmap=plt.cm.jet)
#     plt.show()