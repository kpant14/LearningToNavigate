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


def visualize(fig, ax, img, depth,map, dump_dir, rank, ep_no, t,
              visualize, print_images,dcrop,task):
    for i in range(3):
        ax[i].clear()
        ax[i].set_yticks([])
        ax[i].set_xticks([])
        ax[i].set_yticklabels([])
        ax[i].set_xticklabels([])

    ax[0].imshow(img)
    ax[1].imshow(depth[:,:,0])
    ax[2].imshow(map)

    if task == "milestone1":
        ax[0].set_title("RGB", fontsize=20)
        ax[1].set_title("Depth",fontsize=20)
        ax[2].set_title("Ground Turth Map",fontsize=20)
        
        for _ in range(5):
            plt.tight_layout()


    if visualize:
        plt.gcf().canvas.flush_events()
        fig.canvas.start_event_loop(0.001)
        plt.pause(0.001)
        plt.gcf().canvas.flush_events()

    if print_images:
        if task=="generate_train":
            ax[0].set_axis_off()
            ax[1].set_axis_off()
            plt.margins(0,0)

            ylims_depth = [0,1]
            xlims_depth = [0,1]

            ylims_rgb = [0,1]
            xlims_rgb = [0,1]

            bbox_depth = matplotlib.transforms.Bbox([[((1+dcrop[0])/2)*xlims_depth[0] + xlims_depth[1]*((1-dcrop[0])/2),((1+dcrop[1])/2)*ylims_depth[0] + ylims_depth[1]*((1-dcrop[1])/2)],[((1-dcrop[0])/2)*xlims_depth[0] + xlims_depth[1]*((1+dcrop[0])/2),((1-dcrop[1])/2)*ylims_depth[0] + ylims_depth[1]*((1+dcrop[1])/2)]])
            bbox_depth = bbox_depth.transformed(ax[1].transAxes).transformed(fig.dpi_scale_trans.inverted())

            bbox_rgb = matplotlib.transforms.Bbox([[((1+1)/2)*xlims_rgb[0] + xlims_rgb[1]*((1-1)/2),((1+1)/2)*ylims_rgb[0] + ylims_rgb[1]*((1-1)/2)],[((1-1)/2)*xlims_rgb[0] + xlims_rgb[1]*((1+1)/2),((1-1)/2)*ylims_rgb[0] + ylims_rgb[1]*((1+1)/2)]])
            bbox_rgb = bbox_rgb.transformed(ax[0].transAxes).transformed(fig.dpi_scale_trans.inverted())

            fn_depth = '{}/episodes/{}/{}/{}-{}-Vis-depth-{}.png'.format(   
                dump_dir, (rank + 1), ep_no, rank, ep_no, t)
            fn_rgb = '{}/episodes/{}/{}/{}-{}-Vis-rgb-{}.png'.format(
                dump_dir, (rank + 1), ep_no, rank, ep_no, t)

            plt.savefig(fn_depth,bbox_inches=bbox_depth,dpi=1/bbox_depth.width*16,pad_inches=0)
            plt.savefig(fn_rgb,bbox_inches=bbox_rgb,dpi=1/bbox_rgb.width*84,pad_inches=0)#,dpi=fig.dpi/(xlims_rgb[1]-xlims_rgb[0])*84
        if task == "milestone1":
            fn = '{}/episodes/{}/{}/{}-{}-Vis-{}.png'.format(
                dump_dir, (rank + 1), ep_no, rank, ep_no, t)
            plt.savefig(fn)
