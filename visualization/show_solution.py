#!/usr/bin/env python3

import sys, os
import random
import numpy as np

import matplotlib as mpl
if os.environ.get('DISPLAY','') == '':
    print('no display found. Using non-interactive Agg backend')
    mpl.use('Agg')
import matplotlib.pyplot as plt

from mpl_toolkits.axes_grid1 import make_axes_locatable
import shapely.geometry as geometry
from shapely.ops import cascaded_union, polygonize
import math
from matplotlib.pyplot import arrow
import dubins
this_script_path = os.path.dirname(__file__)   
path_to_utils = os.path.join(this_script_path, "utils")  
sys.path.append(path_to_utils)
import figure_utils
import orienteering_utils
from orienteering_utils import ProblemType


legend_font_size = 24
tick_font_size = 20
NUM_POINTS_TO_GEN = 16
SCATTER_SIZE = 80
FIG_HEIGHT = 7.5
SHOW_FIGURE = True

RESULT_FILE = "../sources/results/results.log"
RESULT_FILE = os.path.join(this_script_path, RESULT_FILE)
                                                                                                                 
#use nice latex fonts if latex is installed
#figure_utils.configure_latex_fonts_latex()

data_vns_sop = orienteering_utils.parse_op_log(RESULT_FILE)

print("using the last results")
record = data_vns_sop[-1]
print("record", record)

problem_type = ProblemType.UNKNOWN

PROBLEM_FILE = record['PROBLEM_FILE']
PROBLEM_FILE = os.path.join(this_script_path, PROBLEM_FILE)

if "datasets/sop/" in PROBLEM_FILE:
    print("showing SOP")
    problem_type = ProblemType.SOP
    SAVE_TO_FIGURE = "solution_sop.png"

elif "datasets/dop_sop_dataset/" in PROBLEM_FILE:
    print("showing DOP")
    problem_type = ProblemType.DOP
    SAVE_TO_FIGURE = "solution_dop.png"

elif "datasets/opn_sop_dataset/" in PROBLEM_FILE:
    print("showing OPN")
    problem_type = ProblemType.OPN
    SAVE_TO_FIGURE = "solution_opn.png"
    
else:
    error("can not decide problem type based on problem file location")
    problem_type = ProblemType.UNKNOWN

op = orienteering_utils.SetOrienteeringProblemDefinition()
op.load_problem_file(PROBLEM_FILE)
nodes = op.nodes
sets_prices = op.get_sets_prices()
sets = op.get_sets()
original_nodes = op.get_set_centers()

result_target_ids = record['RESULT_TARGET_IDS']
result_cluster_ids = record['RESULT_CLUSTER_IDS']
result_rewards = record['REWARDS']
print("problem loaded")
print("result_target_ids:", result_target_ids)
print("result_cluster_ids:", result_cluster_ids)
print("result_rewards", result_rewards)
print("sets_prices", sets_prices)
print("sets", sets)
print("nodes", nodes)

# for the DOP only
result_head_angs = []
sampling_heading = len(sets[0])

calc_reward = 0
for clust_idx in range(len(result_cluster_ids)):
    clust = result_cluster_ids[clust_idx]
    node = result_target_ids[clust_idx]

    if problem_type == ProblemType.DOP:
        node_inside_cluster = node - sets[clust][0]
        # result_node_inside_cluster.append(node_inside_cluster)
        head_ang = math.pi + (2 * math.pi * node_inside_cluster) / sampling_heading
        result_head_angs.append(head_ang)

    calc_reward += sets_prices[clust]
    if node not in sets[clust]:
        print("what the hell, it is not good")

print("calc_reward", calc_reward)

mycmap = plt.cm.get_cmap('RdYlBu_r')

maxx, maxy = -sys.float_info.max,-sys.float_info.max
minx, miny = sys.float_info.max,sys.float_info.max

circle_radiuses = np.ones([len(nodes), 1])
circle_radiuses1 = np.multiply(2.0, circle_radiuses)

nodes_w_rewards = np.zeros((len(nodes), 3))
if problem_type == ProblemType.DOP:
    xses = [i[0] for i in original_nodes]
    yses = [i[1] for i in original_nodes]
    maxx = max(xses)
    minx = min(xses)
    maxy = max(yses)
    miny = min(yses)
    
    nodes_w_rewards = np.zeros((len(original_nodes), 3))
    for nidx in range(len(original_nodes)):
        nodes_w_rewards[nidx, 0] = original_nodes[nidx][0] 
        nodes_w_rewards[nidx, 1] = original_nodes[nidx][1] 
        nodes_w_rewards[nidx, 2] = sets_prices[nidx]
elif problem_type == ProblemType.OPN :
    xses = [nodes[i][0] for i in nodes]
    yses = [nodes[i][1] for i in nodes]
    maxx = max(xses)
    minx = min(xses)
    maxy = max(yses)
    miny = min(yses)
    
    nodes_w_rewards = np.zeros((len(nodes), 3))
    for nidx in nodes:
        nodes_w_rewards[nidx, 0] = nodes[nidx][0]
        nodes_w_rewards[nidx, 1] = nodes[nidx][1]
        
        for set_idx in sets:
            if nidx in sets[set_idx]:
                nodes_w_rewards[nidx, 2] = sets_prices[set_idx]
                break
else:
    xses = [nodes[i][0] for i in nodes]
    yses = [nodes[i][1] for i in nodes]
    maxx = max(xses)
    minx = min(xses)
    maxy = max(yses)
    miny = min(yses)
    
    nodes_w_rewards = np.zeros((len(nodes), 3))
    for nidx in nodes:
        nodes_w_rewards[nidx, 0] = nodes[nidx][0]
        nodes_w_rewards[nidx, 1] = nodes[nidx][1]

        for set_idx in sets:
            if nidx in sets[set_idx]:
                nodes_w_rewards[nidx, 2] = sets_prices[set_idx]
                break

minrew = min(nodes_w_rewards[:, 2])
maxrew = max(nodes_w_rewards[:, 2])


cNorm = mpl.colors.Normalize(vmin=minrew, vmax=maxrew + 0.1 * (maxrew - minrew))       
mycmapScalarMap = mpl.cm.ScalarMappable(norm=cNorm, cmap=mycmap)

fig_width = FIG_HEIGHT*(maxx-minx)/(maxy-miny)
figsize = (fig_width*0.9,FIG_HEIGHT)
print(figsize)

fig = plt.figure(num=None, figsize=figsize, dpi=80, facecolor='w', edgecolor='k')
circles = figure_utils.circles(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1], circle_radiuses1, c=nodes_w_rewards[:, 2] , alpha=0.05, edgecolor='black', linewidth=0.9, linestyle=':')
sc = plt.scatter(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1], c=nodes_w_rewards[:, 2], cmap=mycmap , alpha=1.0, s=1, facecolor='black', lw=0.5)
plt.plot(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1], 'ok', ms=4.0)

# print(nodes_w_rewards[:, 2])

if problem_type == ProblemType.DOP:
    for nidx1 in range(len(nodes_w_rewards)): 
        points = []
        node1 = nodes_w_rewards[nidx1, :]
        points.append([node1[0], node1[1]])
        
        for hind in range(sampling_heading):
            head_ang = math.pi + (2 * math.pi * hind) / sampling_heading
            arrow_len = 30
            arrow(node1[0], node1[1], arrow_len * math.cos(head_ang), arrow_len * math.sin(head_ang))
        
        set_rew = nodes_w_rewards[nidx1, 2] 
        
        alpha = 0.0
        concave_hull = figure_utils.alpha_shape(points, alpha=alpha)        
        color = mycmapScalarMap.to_rgba(set_rew)
        figure_utils.plot_polygon(concave_hull.buffer(40), fc=color)
elif problem_type == ProblemType.OPN:
    for set_idx in reversed(sorted(sets.keys())):
        points = []
        set_rew = sets_prices[set_idx]
        for nidx1 in sets[set_idx]: 
            node1 = nodes_w_rewards[nidx1, :]
            points.append([node1[0], node1[1]])
            for nidx2 in sets[set_idx]: 
                if(nidx1 != nidx2):
                    node2 = nodes_w_rewards[nidx2, :]
                    # plt.plot([node1[0], node2[0] ], [node1[1], node2[1] ], '-k', lw=0.2)
        
        alpha = 0.0
        concave_hull = figure_utils.alpha_shape(points, alpha=alpha)
            
        color = mycmapScalarMap.to_rgba(set_rew)
        figure_utils.plot_polygon(concave_hull.buffer(25), fc=color)

else:    
    for set_idx in reversed(sorted(sets.keys())):
        points = []
        set_rew = sets_prices[set_idx]
        for nidx1 in sets[set_idx]: 
            node1 = nodes_w_rewards[nidx1, :]
            points.append([node1[0], node1[1]])
            for nidx2 in sets[set_idx]: 
                if(nidx1 != nidx2):
                    node2 = nodes_w_rewards[nidx2, :]
                    # plt.plot([node1[0], node2[0] ], [node1[1], node2[1] ], '-k', lw=0.2)
        
        alpha = 0.0
        concave_hull = figure_utils.alpha_shape(points, alpha=alpha)
            
        color = mycmapScalarMap.to_rgba(set_rew)
        figure_utils.plot_polygon(concave_hull.buffer(25), fc=color)
       

for node_idx in range(1, len(result_target_ids)):
    
    if problem_type == ProblemType.DOP:
        step_size = 20
        turning_radius = op.dubins_radius
        node = result_cluster_ids[node_idx]
        node_prew = result_cluster_ids[node_idx - 1]
        q_start = [nodes_w_rewards[node, 0], nodes_w_rewards[node, 1], result_head_angs[node_idx]]
        q_end = [nodes_w_rewards[node_prew][0], nodes_w_rewards[node_prew][1], result_head_angs[node_idx - 1]]
        path = dubins.shortest_path(q_start, q_end, turning_radius)
        qs, _ = path.sample_many(step_size)
        # length_dub += math.ceil(path.path_length())
        xses = [item[0] for item in qs]
        yses = [item[1] for item in qs]
        print(node_prew, '->', node, ",", q_start, '->', q_end)
        plt.plot(xses, yses, '-g', lw=1.6)
        
    elif problem_type == ProblemType.OPN:
        node = result_target_ids[node_idx]
        node_prew = result_target_ids[node_idx - 1]
        node_pos = [nodes[node][0], nodes[node][1]]
        node_pos_prew = [nodes[node_prew][0], nodes[node_prew][1]]
        print(node_prew, '->', node, ",", node_pos_prew, '->', node_pos)
        plt.plot([node_pos_prew[0], node_pos[0] ], [node_pos_prew[1], node_pos[1] ], '-g', lw=1.6)

    else:
        node = result_target_ids[node_idx]
        node_prew = result_target_ids[node_idx - 1]
        node_pos = [nodes[node][0], nodes[node][1]]
        node_pos_prew = [nodes[node_prew][0], nodes[node_prew][1]]
        print(node_prew, '->', node, ",", node_pos_prew, '->', node_pos)
        plt.plot([node_pos_prew[0], node_pos[0] ], [node_pos_prew[1], node_pos[1] ], '-g', lw=1.6)

ax = plt.gca()
ax.axis('equal')
figure_utils.no_axis(ax)

cbar_position = [0.20, 0.05, 0.6, 0.03]
cbar_ax = fig.add_axes(cbar_position)
cb = plt.colorbar(sc, cax=cbar_ax, orientation='horizontal')
cb.ax.tick_params(labelsize=tick_font_size)
cb.set_label('profit', labelpad=-65.0, y=0.8, fontsize=legend_font_size)

# offset = 0.08
fig.subplots_adjust(left=-0.035, right=1.035 , top=1.07 , bottom=0.0)

plt.savefig(SAVE_TO_FIGURE, dpi=300)
if SHOW_FIGURE:
    plt.show()  

