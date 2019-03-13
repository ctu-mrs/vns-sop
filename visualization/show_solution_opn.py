#!/usr/bin/env python3

import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.axes_grid1 import make_axes_locatable
import sys

sys.path.append("utils")
import figure_utils
import orienteering_utils

figure_utils.configure_latex_fonts()

RESULT_FILE = "../sources/results/results.log"
SAVE_TO_FIGURE = "solution_opn.png"

legend_font_size = 24
tick_font_size = 20
NUM_POINTS_TO_GEN = 16
SCATTER_SIZE = 80
figsize = (5,6)

SHOW_FIGURE = True



data_vns_sop = orienteering_utils.parse_op_log(RESULT_FILE)

print("using the last results")
record = data_vns_sop[-1]
#print("record",record)

PROBLEM_FILE = record['PROBLEM_FILE']
op = orienteering_utils.SetOrienteeringProblemDefinition()
op.load_problem_file(PROBLEM_FILE)
sets_prices = op.get_sets_prices()
sets = op.get_sets()

result_target_ids = record['RESULT_TARGET_IDS']
result_cluster_ids = record['RESULT_CLUSTER_IDS']
result_cluster_ids[0] = 0
result_cluster_ids[-1] = 1
result_rewards = record['REWARDS']
print("problem loaded")
print("result_target_ids:", result_target_ids)
print("result_cluster_ids:", result_cluster_ids)
print("result_rewards", result_rewards)
print("sets_prices", sets_prices)
print("sets", sets)

calc_reward = 0
for clust_idx in range(len(result_cluster_ids)):
    clust = result_cluster_ids[clust_idx]
    node = result_target_ids[clust_idx]
    # print("clust",clust)
    # print("node",node)
    # print("sets",sets[clust])
    calc_reward += sets_prices[clust]
    if node not in sets[clust]:
        print("what the hell, it is not good")

print("calc_reward", calc_reward)

mycmap = plt.cm.get_cmap('RdYlBu_r')

circle_radiuses = np.ones([len(op.nodes), 1])
circle_radiuses1 = np.multiply(2.0, circle_radiuses)

nodes_w_rewards = np.zeros((len(op.nodes), 3))
for nidx in op.nodes:
    nodes_w_rewards[nidx, 0] = op.nodes[nidx][0]
    nodes_w_rewards[nidx, 1] = op.nodes[nidx][1]
    
    for set_idx in sets:
        if nidx in sets[set_idx]:
            nodes_w_rewards[nidx, 2] = sets_prices[set_idx]
            break

minrew = min(nodes_w_rewards[:, 2])
maxrew = max(nodes_w_rewards[:, 2])

cNorm = mpl.colors.Normalize(vmin=minrew, vmax=maxrew + 0.1*(maxrew-minrew))
#cNorm  = mpl.colors.Normalize(vmin=min(nodes_w_rewards[:, 2]), vmax=max(nodes_w_rewards[:, 2]))
mycmapScalarMap = mpl.cm.ScalarMappable(norm=cNorm, cmap=mycmap)

fig = plt.figure(num=None, figsize=figsize, dpi=80, facecolor='w', edgecolor='k')
circles = figure_utils.circles(nodes_w_rewards[:,0],nodes_w_rewards[:,1],circle_radiuses1, c=nodes_w_rewards[:,2] , alpha=0.05, edgecolor='black',linewidth=0.9,linestyle=':')
sc = plt.scatter(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1], c=nodes_w_rewards[:, 2], cmap=mycmap , alpha=1.0, s=1, facecolor='black', lw=0.5)
plt.plot(nodes_w_rewards[:,0],nodes_w_rewards[:,1],'ok',ms = 4.0)

print(nodes_w_rewards[:, 2])

for set_idx in reversed(sorted(sets.keys())):
    points = []
    set_rew = sets_prices[set_idx]
    for nidx1 in sets[set_idx]: 
        node1 = nodes_w_rewards[nidx1, :]
        points.append([node1[0],node1[1]])
        for nidx2 in sets[set_idx]: 
            if(nidx1 != nidx2):
                node2 = nodes_w_rewards[nidx2, :]
                #plt.plot([node1[0], node2[0] ], [node1[1], node2[1] ], '-k', lw=0.2)
    
    alpha = 0.0
    concave_hull = figure_utils.alpha_shape(points, alpha=alpha)
        
    color = mycmapScalarMap.to_rgba(set_rew)
    figure_utils.plot_polygon(concave_hull.buffer(25),fc=color)
    
           
print(result_target_ids)
for node_idx in range(1,len(result_target_ids)):
    #print(node_idx-1,node_idx)
    node = result_target_ids[node_idx]
    node_prew = result_target_ids[node_idx-1]
    node_pos = [op.nodes[node][0], op.nodes[node][1]]
    node_pos_prew = [op.nodes[node_prew][0], op.nodes[node_prew][1]]
    print(node_prew,'->',node,",",node_pos_prew,'->',node_pos)
    plt.plot([node_pos_prew[0], node_pos[0] ], [node_pos_prew[1], node_pos[1] ], '-g', lw=1.6)

# plt.plot(sampled_path1[:,0],sampled_path1[:,1],'-k',lw = 2.0)
ax = plt.gca()
ax.axis('equal')

cbar_position = [0.20, 0.05, 0.6, 0.03]
cbar_ax = fig.add_axes(cbar_position)
cb = plt.colorbar(sc, cax=cbar_ax, orientation='horizontal')
cb.ax.tick_params(labelsize=tick_font_size)
cb.set_label('profit', labelpad=-57.0, y=0.8, fontsize=legend_font_size)

figure_utils.no_axis(ax)
offset = 0.08
fig.subplots_adjust(left=-0.2, right=1.2 , top=1.02 , bottom=0.1)

plt.savefig(SAVE_TO_FIGURE,dpi=300)
if SHOW_FIGURE:
    plt.show()  

