# First import the library
import pyrealsense2 as rs
import cv2
import numpy as np

# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
pipeline.start()


# First import the library
import pyrealsense2 as rs

# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
pipeline.start()

img = np.zeros((512,512,3), np.uint8)
# mouse callback function
drawing = False # true if mouse is pressed
mode = True # if True, draw rectangle. Press 'm' to toggle to curve
ix,iy = -1,-1

# mouse callback function
previous_point = ()
initial_point = ()
def draw_circle(event,x,y,flags,param):
    global ix,iy,drawing,mode, previous_point, initial_point

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y
        previous_point = (x,y)
        initial_point = (x,y)

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            if mode == True:
                cv2.rectangle(img,(ix,iy),(x,y),(0,255,0),-1)
            else:
                # cv2.circle(img,(x,y),5,(0,0,255),-1)
                cv2.line(img, previous_point, (x,y), (0,0,255) )
                previous_point = (x,y)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        if mode == True:
            cv2.rectangle(img,(ix,iy),(x,y),(0,255,0),-1)
        else:
            # cv2.circle(img,(x,y),5,(0,0,255),-1)
            cv2.line(img, previous_point, (x,y), (255,0,0) )
            cv2.line(img, (x,y), initial_point, (255,0,0) )
            previous_point = []
            

    cv2.imshow('image',img)
    

# Create a black image, a window and bind the function to window
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)


for i in range(5):
    # This call waits until a new coherent set of frames is available on a device
    # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
    frames = pipeline.wait_for_frames()
    depth_data = frames[0].as_frame().get_data()
    color_data = frames[1].as_frame().get_data()
    np_image = np.asanyarray(color_data)
    img = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)


    # Display the resulting frame
    cv2.imshow('image',img)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

cv2.imshow('image',img)
while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('m'):
        mode = not mode
    elif k == 27:
        break

cv2.destroyAllWindows()
    # if not depth: continue

    # # Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
    # coverage = [0]*64
    # for y in range(480):
    #     for x in range(640):
    #         dist = depth.get_distance(x, y)
    #         if 0 < dist and dist < 1:
    #             coverage[int(x/10)] += 1

    #     if y%20 is 19:
    #         line = ""
    #         for c in coverage:
    #             line += " .:nhBXWW"[int(c/25)]
    #         coverage = [0]*64
    #         print(line)