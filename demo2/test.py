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

    xmin = np.array(contour_points)[:,0].min()
    xmax = np.array(contour_points)[:,0].max()
    ymin = np.array(contour_points)[:,1].min()
    ymax = np.array(contour_points)[:,1].max()


    # distance to middle pixel
    d = depth_image[int(nrows/2), int(ncols/2)]


    align = rs.align(rs.stream.color)
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    pc = rs.pointcloud()
    pc.map_to(aligned_depth_frame)
    points = pc.calculate(color_frame)

    # pc = rs.pointcloud()
    # points = pc.calculate(frames[0])



    vertices = np.asanyarray(points.get_vertices())

    vertices = np.array([list(v) for v in vertices])[::3,:]

    import matplotlib.pyplot as plt
    plt.scatter(vertices[:,0], vertices[:,1], c=vertices[:,2],linewidth=0,cmap=plt.cm.jet)

    # from mpl_toolkits.mplot3d import Axes3D
    # fig = plt.figure(figsize=(4,4))
    # ax = fig.add_subplot(1, 1, 1, projection='3d')
    # ax.scatter3D(vertices[::10,0],vertices[::10,1],vertices[::10,2])
    # # https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/include/librealsense2/rsutil.h#L15

    plt.show()


    # grey_color = 153
    # clipping_distance = 10000
    # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
    # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
    

    # Render images
    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    # images = np.hstack((bg_removed, depth_colormap))
    # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('Align Example', images)
    # cv2.waitKey(1)
    # points_image_coord = pc.calculate(frames[1])
    


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

