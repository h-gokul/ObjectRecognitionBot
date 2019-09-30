from lib import *

img_mod = img_module()

dist_threshold=30
while True:
    img_mod.read_pi()
    (centroid,dist,angle,img) = img_mod.get_img_target()
    if centroid[1] + dist[1] > 480:
        stop()
        break
    if abs(angle) >= img_mod.angle_tolerance: # required angle not met 
        if(angle < 0):
             print("right")
             glide_right()
             #stop()
        else:
             print("left")
             glide_left()
             #stop()
    else:
        if dist[1] > dist_threshold:
             print("front")
             forward_slow()
        else:
             print("stop")
             stop() 
    
                     
