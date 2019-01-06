from lib import *

img_mod = img_module()

img_mod.read()

(a,b,c,d) = img_mod.get_img_target() #d is to view image, debug purposes

while True:
    img_mod.read()
    (centroid,dist,angle,img) = img_mod.get_img_target()
    if abs(angle) >= img_mod.angle_tolerance: # required angle not met 
        if(angle > 0):
            print("right")
#             move_right()
#             stop()
        else:
            print("left")
#             move_left()
#             stop()
    else:
        if dist[1] > dist_threshold:
            print("front")
#             move_front()
        else:
            print("stop")
#             stop()
    
