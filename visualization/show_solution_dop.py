#!/usr/bin/env python3

import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.axes_grid1 import make_axes_locatable
import sys
import dubins
import math

from matplotlib.pyplot import arrow

sys.path.append("utils")
import figure_utils
import orienteering_utils

figure_utils.configure_latex_fonts()

RESULT_FILE = "../sources/results/results.log"

SAVE_TO_FIGURE = "solution_dop.png"

legend_font_size = 24
tick_font_size = 20
NUM_POINTS_TO_GEN = 16
SCATTER_SIZE = 80
figsize = (5, 6)

SHOW_FIGURE = True

#op = orienteering_utils.OrienteeringProblemDefinition()
#op.load_problem_file(ORIGINAL_PROBLEM_FILE)
#original_nodes = op.get_nodes()


#print("original_nodes",original_nodes)
data_vns_sop = orienteering_utils.parse_op_log(RESULT_FILE)

print("using the last results")
record = data_vns_sop[-1]

PROBLEM_FILE = record['PROBLEM_FILE']
sop = orienteering_utils.SetOrienteeringProblemDefinition()
sop.load_problem_file(PROBLEM_FILE)
sets_rewards = sop.get_sets_prices()
sets = sop.get_sets()
original_nodes= sop.get_set_centers()



#print("record",record)

result_target_ids = record['RESULT_TARGET_IDS']
result_cluster_ids = record['RESULT_CLUSTER_IDS']
result_node_inside_cluster = []
result_head_angs = []
result_cluster_ids[0] = 0
result_cluster_ids[-1] = 1
result_rewards = record['REWARDS']
result_length = record['LENGTH']
print("problem loaded")
print("result_target_ids:", result_target_ids)
print("result_cluster_ids:", result_cluster_ids)
print("result_rewards", result_rewards)
print("sets_rewards", sets_rewards)
print("sets", sets)
print("result_length",result_length)

turning_radius = 50
step_size = 20
SAMPLING_HEADING = 8

calc_reward = 0
for clust_idx in range(len(result_cluster_ids)):
    clust = result_cluster_ids[clust_idx]
    node = result_target_ids[clust_idx]
    
    node_inside_cluster = node - sets[clust][0]
    result_node_inside_cluster.append(node_inside_cluster)
    head_ang = math.pi+ (2 * math.pi * node_inside_cluster) / SAMPLING_HEADING
    result_head_angs.append(head_ang)

    calc_reward += sets_rewards[clust]
    if node not in sets[clust]:
        print("what the hell, it is not good")

#print("calc_reward", calc_reward)

mycmap = plt.cm.get_cmap('RdYlBu_r')

circle_radiuses = np.ones([len(original_nodes), 1])
circle_radiuses1 = np.multiply(2.0, circle_radiuses)

# print(op.nodes)
nodes_w_rewards = np.zeros((len(original_nodes), 3))
for nidx in range(len(original_nodes)):
    nodes_w_rewards[nidx, 0] = original_nodes[nidx][0] 
    nodes_w_rewards[nidx, 1] = original_nodes[nidx][1] 
    nodes_w_rewards[nidx, 2] = sets_rewards[nidx]
    
    
#print("nodes_w_rewards",nodes_w_rewards) 


minrew = min(nodes_w_rewards[:, 2])
maxrew = max(nodes_w_rewards[:, 2])

cNorm = mpl.colors.Normalize(vmin=minrew, vmax=maxrew + 0.1*(maxrew-minrew))
#cNorm = mpl.colors.Normalize(vmin=min(nodes_w_rewards[:, 2]), vmax=max(nodes_w_rewards[:, 2]))
mycmapScalarMap = mpl.cm.ScalarMappable(norm=cNorm, cmap=mycmap)

result_node_inside_cluster

fig = plt.figure(num=None, figsize=figsize, dpi=80, facecolor='w', edgecolor='k')
circles = figure_utils.circles(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1], circle_radiuses1, c=nodes_w_rewards[:, 2] , alpha=0.05, edgecolor='black', linewidth=0.9, linestyle=':')
sc = plt.scatter(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1], c=nodes_w_rewards[:, 2], cmap=mycmap , alpha=1.0, s=1, facecolor='black', lw=0.5)
plt.plot(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1], 'ok', ms=4.0)

#print(nodes_w_rewards[:, 2])


    
for nidx1 in range(len(nodes_w_rewards)): 
    points = []
    node1 = nodes_w_rewards[nidx1, :]
    points.append([node1[0], node1[1]])
    
    for hind in range(SAMPLING_HEADING):
        head_ang = math.pi+ (2 * math.pi * hind) / SAMPLING_HEADING
        arrow_len = 30
        arrow(node1[0], node1[1], arrow_len*math.cos(head_ang), arrow_len*math.sin(head_ang))
    
    
    set_rew = nodes_w_rewards[nidx1,2] 
    
    alpha = 0.0
    concave_hull = figure_utils.alpha_shape(points, alpha=alpha)        
    color = mycmapScalarMap.to_rgba(set_rew)
    figure_utils.plot_polygon(concave_hull.buffer(40), fc=color)
           
#print(result_cluster_ids)
length_dub = 0
for clust_idx in range(1, len(result_cluster_ids)):
    # print(node_idx-1,node_idx)
    
    node = result_cluster_ids[clust_idx]
    node_prew = result_cluster_ids[clust_idx - 1]
    
    q_start = [nodes_w_rewards[node, 0], nodes_w_rewards[node, 1], result_head_angs[clust_idx]]
    q_end = [nodes_w_rewards[node_prew][0], nodes_w_rewards[node_prew][1], result_head_angs[clust_idx - 1]]

    path = dubins.shortest_path(q_start, q_end, turning_radius)
    # length = dubins.path_length(q_start, q_end, DUBINS_RADIUS)
    # path = dubins.shortest_path(q_start, q_end, turning_radius)
    qs, _ = path.sample_many( step_size)
    length_dub += math.ceil(path.path_length())
    # configurations, _ = path.sample_many(step_size)
    #print(qs)
    
    print(node_prew, '->', node, ",", q_start, '->', q_end)
    xses = [item[0] for item in qs]
    yses = [item[1] for item in qs]
    plt.plot(xses, yses, '-g', lw=1.6)

print("length_dub",length_dub)
if length_dub !=result_length:
    print("very bad man!!!! ")
    sys.exit(1)

# plt.plot(sampled_path1[:,0],sampled_path1[:,1],'-k',lw = 2.0)
ax = plt.gca()
ax.axis('equal')
figure_utils.no_axis(ax)

cbar_position = [0.20, 0.05, 0.6, 0.03]
cbar_ax = fig.add_axes(cbar_position)
cb = plt.colorbar(sc, cax=cbar_ax, orientation='horizontal')
cb.ax.tick_params(labelsize=tick_font_size)
cb.set_label('profit', labelpad=-57.0, y=0.8, fontsize=legend_font_size)


offset = 0.08
#fig.subplots_adjust(left=-0.2, right=1.2 , top=1.1 , bottom=0.1)

plt.savefig(SAVE_TO_FIGURE,dpi=300)
if SHOW_FIGURE:
    plt.show()  

