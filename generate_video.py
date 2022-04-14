import cv2
import os
from natsort import natsorted

env_folder = 'tmp/dump/ans/episodes/1/6'
image_folder = env_folder
video_name = env_folder+'video.avi'

images = [img for img in natsorted(os.listdir(image_folder)) if img.endswith(".png")]
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(video_name, 0, 15, (width//2,height//2))
for image in images:
    image_ = cv2.imread(os.path.join(image_folder, image))
    image_ = cv2.resize(image_,(width//2,height//2),interpolation=cv2.INTER_AREA)
    video.write(image_)

cv2.destroyAllWindows()
video.release()