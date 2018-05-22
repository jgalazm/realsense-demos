# First import the library
import pyrealsense2 as rs
import cv2
import numpy as np
depth_image = None
color_image = None
frames = None

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
contour_points = []
def draw_circle(event,x,y,flags,param):
    global ix,iy,drawing,mode, previous_point, initial_point, contour_points

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y
        previous_point = (x,y)
        initial_point = (x,y)
        contour_points.append([x,y])

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            if mode == True:
                cv2.rectangle(img,(ix,iy),(x,y),(0,255,0),-1)
            else:
                # cv2.circle(img,(x,y),5,(0,0,255),-1)
                cv2.line(img, previous_point, (x,y), (0,0,255) )
                previous_point = (x,y)
                contour_points.append([x,y])

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        if mode == True:
            cv2.rectangle(img,(ix,iy),(x,y),(0,255,0),-1)
        else:
            # cv2.circle(img,(x,y),5,(0,0,255),-1)
            cv2.line(img, previous_point, (x,y), (255,0,0) )
            cv2.line(img, (x,y), initial_point, (255,0,0) )
            previous_point = []

            do_everything(depth_image, contour_points)
            contour_points = []
            

    cv2.imshow('image',img)

def  get_points_cloud(depth_image):
    nrows = depth_image.shape[0]
    ncols = depth_image.shape[1]

    # distance to middle pixel
    d = depth_image[int(nrows/2), int(ncols/2)]

    pc = rs.pointcloud()
    points = pc.calculate(frames[0])

    vertices = np.asanyarray(points.get_vertices())

    vertices = np.array([[vi for vi in v] for v in vertices])[::3,:]

    import matplotlib.pyplot as plt
    plt.scatter(vertices[:,0], vertices[:,1], c=vertices[:,2],linewidth=0,cmap=plt.cm.jet)
    plt.show()

    


def  get_mesh(points):
    pass

def  get_masked_triangles(contour_points, triangles, depth_image):
    pass

def  get_area_from_triangles(masked_triangles, points):
    pass

def do_everything(depth_image, contour_points):

    # obtener nube de puntos

    points = get_points_cloud(depth_image)

    # triangular la malla de pixeles

    triangles = get_mesh(points)

    # seleccionar triangulos dentro del contorno en coordenadas de pixeles

    masked_triangles = get_masked_triangles(contour_points, triangles, depth_image)

    # calcular area

    area = get_area_from_triangles(masked_triangles, points)





# Create a black image, a window and bind the function to window
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)


for i in range(5):
    # This call waits until a new coherent set of frames is available on a device
    # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
    frames = pipeline.wait_for_frames()
    depth_data = frames[0].as_frame().get_data()
    color_data = frames[1].as_frame().get_data()
    depth_image = np.asanyarray(depth_data)/100 #cm?
    color_image = np.asanyarray(color_data)
    img = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)


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

