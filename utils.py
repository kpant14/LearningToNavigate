import sys
import matplotlib
import numpy as np

if sys.platform == 'darwin':
    matplotlib.use("tkagg")
elif sys.platform == 'linux':
    matplotlib.use("tkagg")
else:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt

import seaborn as sns
import skimage
from habitat_sim.utils import viz_utils as vut
import cv2


def visualize(fig, ax, img, depth, dump_dir, rank, ep_no, t,
              visualize, print_images):
    for i in range(2):
        ax[i].clear()
        ax[i].set_yticks([])
        ax[i].set_xticks([])
        ax[i].set_yticklabels([])
        ax[i].set_xticklabels([])

    ax[0].imshow(img)
    ax[0].set_title("RGB", fontsize=20)

  
    ax[1].imshow(depth)
    ax[1].set_title("Depth",fontsize=20)

    for _ in range(5):
        plt.tight_layout()

    if visualize:
        plt.gcf().canvas.flush_events()
        fig.canvas.start_event_loop(0.001)
        plt.gcf().canvas.flush_events()

    if print_images:
        fn = '{}/episodes/{}/{}/{}-{}-Vis-{}.png'.format(
            dump_dir, (rank + 1), ep_no, rank, ep_no, t)
        plt.savefig(fn)