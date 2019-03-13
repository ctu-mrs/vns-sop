import numpy as np
import sys
import csv
import os
import math
from scipy.spatial import ConvexHull
from scipy.spatial import Delaunay
import shapely.geometry as geometry
from shapely.ops import cascaded_union, polygonize
import matplotlib.pyplot as plt
from descartes import PolygonPatch


class OrienteeringProblemDefinition():

    def __init__(self):
        self.budget = 0
        self.nodes = np.array([])
        self.num_vehicles = 0
        self.start_index = 0
        self.end_index = 1
    
    def load_problem_file(self, problem_file):
        try:
            with open(problem_file, "r")  as file:
                first_line_readed = False
                for line in file:
                    if first_line_readed:
                        node_line = line.split()
                        point_to_add = np.array([float(node_line[0]), float(node_line[1]), float(node_line[2])])
                        if np.size(self.nodes, 0) < 1:  
                            self.nodes = np.hstack((self.nodes, point_to_add))
                        else:
                            self.nodes = np.vstack((self.nodes, point_to_add))
                        
                    else:
                        budget_line = line.split()
                        first_line_readed = True
                        self.budget = float(budget_line[0])
                        self.num_vehicles = int(budget_line[1])
                        
        except Exception as e:
            print("can not parse op problem file")
            raise
        
    def get_nodes(self):
        return self.nodes
    
    def get_budget(self):
        return self.budget
    
    def get_num_vehicles(self):
        return self.num_vehicles
    
    def get_start_index(self):
        return self.start_index 

    def get_end_index(self):
        return self.end_index

    
class SetOrienteeringProblemDefinition():

    EDGE_WEIGHT_TYPE_EXPLICIT = 1
    EDGE_WEIGHT_TYPE_CEIL_2D = 2
    
    def __init__(self):
        self.budget = 0
        self.dimension = 0
        self.nodes = {}
        self.sets = {}
        self.sets_prices = {}
        self.set_centers = {}
        self.distance_matrix = None
        self.num_vehicles = 0
        self.start_index = 0
        self.end_index = 1
        self.dubins_radius = 0
        self.neighborhood_radius = 0
        self.edge_weight_type = SetOrienteeringProblemDefinition.EDGE_WEIGHT_TYPE_CEIL_2D
    
    def get_nodes(self):
        return self.nodes    
    
    def get_sets(self):
        return self.sets
    
    def get_sets_prices(self):
        return self.sets_prices
    
    def get_budget(self):
        return self.budget
    
    def get_num_vehicles(self):
        return self.num_vehicles
    
    def get_start_index(self):
        return self.start_index 

    def get_end_index(self):
        return self.end_index
        
    def get_set_centers(self):
        return self.set_centers
    
    def get_distance_matrix(self):
        return self.distance_matrix
    
    def get_edge_weight_type(self):
        return self.edge_weight_type
        
    def load_problem_file(self, problem_file):    
        NODE_COORD_SECTION = "NODE_COORD_SECTION"
        GTSP_SET_SECTION = "GTSP_SET_SECTION"
        CONSTRAINING_SET_SECTION = "CONSTRAINING_SET_SECTION"
        GTSP_SET_CENTER_COORD_SECTION = "GTSP_SET_CENTER_COORD_SECTION"
        EDGE_WEIGHT_SECTION = "EDGE_WEIGHT_SECTION"
        EDGE_WEIGHT_TYPE = "EDGE_WEIGHT_TYPE"
        EDGE_WEIGHT_TYPE_EXPLICIT = "EXPLICIT"
        EDGE_WEIGHT_TYPE_CEIL_2D = "CEIL_2D"
        DIMENSION = "DIMENSION"
        START_SET = "START_SET"
        END_SET = "END_SET"
        TMAX = "TMAX"
        EOF = "EOF"
        DUBINS_RADIUS = "DUBINS_RADIUS"
        NEIGHBORHOOD_RADIUS = "NEIGHBORHOOD_RADIUS"
        
        coords_section = False
        set_section = False
        constraining_set_section = False  # used in SOP datasets
        gtsp_set_center_coords = False  # used in SOP DOP and OPN datasets
        edge_weight_section = False
        self.dimension = 0
        self.nodes = {}
        self.sets = {}
        self.sets_prices = {}
        self.set_centers = {}
        edge_weight_section_from = 0
        edge_weight_section_to = 0
        print("reading prolem file", problem_file)
        calc_using_cords = 0
        with open(problem_file, 'r') as f:
            for line in f:
                
                if EDGE_WEIGHT_TYPE in line:
                    strs = line.split()
                    #print(strs)
                    if strs[1] == EDGE_WEIGHT_TYPE_EXPLICIT:
                        self.edge_weight_type = SetOrienteeringProblemDefinition.EDGE_WEIGHT_TYPE_EXPLICIT
                        #print("set explicit")
                    elif strs[1] == EDGE_WEIGHT_TYPE_CEIL_2D:
                        self.edge_weight_type = SetOrienteeringProblemDefinition.EDGE_WEIGHT_TYPE_CEIL_2D
                        #print("set ceil 2d")
                    else:
                        print("EDGE_WEIGHT_TYPE is unknown", strs)
                        sys.exit(1)
                
                if TMAX in line:
                    strs = line.split()
                    if strs[1].isdigit():
                        self.budget = int(strs[1])
                    else:
                        print("TMAX is not digit", strs)
                        sys.exit(1)
                
                if DUBINS_RADIUS in line:
                    strs = line.split()
                    if strs[1].isdigit():
                        self.dubins_radius = int(strs[1])
                        
                if NEIGHBORHOOD_RADIUS in line:
                    strs = line.split()
                    if strs[1].isdigit():
                        self.neighborhood_radius = int(strs[1])                    
                
                if DIMENSION in line:
                    dim_strs = line.split()
                    if dim_strs[1].isdigit():
                        self.dimension = int(dim_strs[1])
                    else:
                        print("dimension is not digit", dim_strs)
                        sys.exit(1)
                        
                if START_SET in line:
                    strs = line.split()
                    if strs[1].isdigit():
                        self.start_index = int(strs[1])
                    else:
                        print("START_SET is not digit", strs)
                        sys.exit(1)
                        
                if END_SET in line:
                    strs = line.split()
                    if strs[1].isdigit():
                        self.end_index = int(strs[1])
                    else:
                        print("END_SET is not digit", strs)
                        sys.exit(1)
                
                if NODE_COORD_SECTION in line:
                    coords_section = True
                    set_section = False
                    constraining_set_section = False
                    gtsp_set_center_coords = False
                    edge_weight_section = False
                    continue
                
                if GTSP_SET_SECTION in line:
                    coords_section = False
                    set_section = True
                    constraining_set_section = False
                    gtsp_set_center_coords = False
                    edge_weight_section = False
                    continue
                
                if CONSTRAINING_SET_SECTION in line:
                    coords_section = False
                    set_section = False
                    constraining_set_section = True
                    gtsp_set_center_coords = False
                    edge_weight_section = False
                    continue
                
                if GTSP_SET_CENTER_COORD_SECTION in line:
                    coords_section = False
                    set_section = False
                    constraining_set_section = False
                    gtsp_set_center_coords = True
                    edge_weight_section = False
                    continue
                
                if EDGE_WEIGHT_SECTION in line:
                    coords_section = False
                    set_section = False
                    constraining_set_section = False
                    gtsp_set_center_coords = False
                    edge_weight_section = True
                    continue
                
                if EOF in line:
                    continue
                
                if coords_section:
                    coord_parts = line.split()
                    id_coord = int(coord_parts[0]) - 1
                    #print(coord_parts)
                    point_to_add = [float(coord_parts[1]), float(coord_parts[2])] 
                    self.nodes[id_coord] = [float(coord_parts[1]), float(coord_parts[2])] 
                  
                if set_section:
                    set_parts = line.split()
                    self.sets[int(set_parts[0])] = []
                    self.sets_prices[int(set_parts[0])] = int(set_parts[1])
                    for i in range(2, len(set_parts)):
                        id_coord = int(set_parts[i]) - 1
                        self.sets[int(set_parts[0])].append(id_coord)
                    # print(set_parts)
                    
                if edge_weight_section:
                    if self.distance_matrix is None:
                        self.distance_matrix = [ [0] * self.dimension for _ in range(self.dimension)]
                    
                    # print("edge_weight_section")
                    set_parts = line.split()
                    for i in range(len(set_parts)):
                        # print("add from ", edge_weight_section_from, "to", edge_weight_section_to)
                        self.distance_matrix[edge_weight_section_from][edge_weight_section_to] = float(set_parts[i])
                        edge_weight_section_to += 1
                    
                    if edge_weight_section_to >= self.dimension:
                        edge_weight_section_from += 1
                        edge_weight_section_to = 0
                    
                if constraining_set_section:
                    # set_id set_prize id-vertex-list
                    set_parts = line.split()
                    self.sets[int(set_parts[0])] = []
                    self.sets_prices[int(set_parts[0])] = int(set_parts[1])
                    for i in range(2, len(set_parts)):
                        id_coord = int(set_parts[i]) - 1
                        self.sets[int(set_parts[0])].append(id_coord)
                        
                if gtsp_set_center_coords:
                    set_parts = line.split()
                    self.set_centers[int(set_parts[0])] = [float(set_parts[1]), float(set_parts[2])]


def parse_individual_value(value):
    converted = False
    if not converted:
        try: 
            value = int(value)
            converted = True
        except ValueError:
            pass
    if not converted:
        try:
            value = float(value)
            converted = True
        except ValueError:
            pass
    return value


def parse_op_log(log_file):
    print("loading", log_file)
    log_lines = []
    delimiterVariable = ';';
    delimiterValue = ':';
    delimiterSubValue = ',';
    # not yet implemented
    delimiterSubSubValue = '|';
    if not os.path.exists(log_file):
        raise Exception("file does not exists " + log_file) 
        return log_lines
    with open(log_file, "r")  as file:
        for line in file:
            var_values = line.split(delimiterVariable)
            variables = []
            values = []
            for single_var_value in var_values:
                if delimiterValue in single_var_value: 
                    splited_var_value = single_var_value.split(delimiterValue)
                    # print("splited_var_value",splited_var_value)
                    variable = splited_var_value[0]
                    value = splited_var_value[1]
                   
                    if delimiterSubValue in value or delimiterSubSubValue in value:
                        # contains delimiterSubValue
                        splited_sub_value = value.split(delimiterSubValue)
                        subvalue_array = []
                        for subvalue in splited_sub_value:
                                
                            if delimiterSubSubValue in subvalue:
                                # contains delimiterSubSubValue
                                splited_sub_sub_value = subvalue.split(delimiterSubSubValue)
                                sub_subvalue_array = []
                                for sub_subvalue in splited_sub_sub_value:
                                    sub_subvalue = parse_individual_value(sub_subvalue)
                                    sub_subvalue_array.append(sub_subvalue)
                                subvalue_array.append(sub_subvalue_array)
                            else:
                                # does not contains delimiterSubSubValue
                                subvalue = parse_individual_value(subvalue)
                                subvalue_array.append(subvalue)
                        value = subvalue_array
                    else:
                        # does not contains delimiterSubValue
                        value = parse_individual_value(value)    
                    # convert normal value ints
                    
                    variables.append(variable)
                    values.append(value)
            # print("variables",variables)
            # print("values",values)
            single_log = dict(zip(variables, values))
            log_lines.append(single_log)
    # sys.exit()
    print("loaded")
    return log_lines


def load_sampled_path(sampled_path_file):
    sampled_path = None
    with open(sampled_path_file, 'rb') as csvfile:
        scv_reader = csv.reader(csvfile)
        for row in scv_reader:
            to_add = np.array([float(row[0]), float(row[1]), float(row[2])])
            if(sampled_path is None):
                sampled_path = to_add
            else:
                sampled_path = np.vstack((sampled_path, to_add))
    # print(sampled_path)
    return sampled_path


def getUniqueVeluesOfKey(data_to_process, key):
    unique_value_set = set([])
    for idx in range(len(data_to_process)):
        unique_value_set.add(data_to_process[idx][key])
    return list(unique_value_set)


def getDataWithKeyValue(data_to_process, key, value):
    data_subset = []
    for idx in range(len(data_to_process)):
        if data_to_process[idx][key] == value:
            data_subset.append(data_to_process[idx])
    return data_subset


def getDataWithKeyValueMap(data_to_process, mapp):
    data_subset = []
    for idx in range(len(data_to_process)):
        has_all_attrs = True
        for key in mapp:
            if data_to_process[idx][key] != mapp[key]:
                has_all_attrs = False
                break
        if has_all_attrs:
            data_subset.append(data_to_process[idx])
    return data_subset


def getMaxValueInData(data_to_process, key):
    assert(len(data_to_process) > 0 and data_to_process[0].get(key) is not None)
    max = data_to_process[0][key]
    for idx in range(1, len(data_to_process)):
        if data_to_process[idx][key] > max:
            max = data_to_process[idx][key]
    return max


def getMinValueInData(data_to_process, key):
    assert(len(data_to_process) > 0 and data_to_process[0].get(key) is not None)
    min = data_to_process[0][key]
    for idx in range(1, len(data_to_process)):
        if data_to_process[idx][key] < min:
            min = data_to_process[idx][key]
    return min


def getAverageValueInData(data_to_process, key):
    assert(len(data_to_process) > 0 and data_to_process[0].get(key) is not None)
    sum = 0
    num_tested = 0
    for idx in range(len(data_to_process)):
        num_tested += 1
        sum += data_to_process[idx][key]
    return float(sum) / float(num_tested)


def find_minimal_distance_to(sets_sequence, sets_dic, node_dict):
    # distance record has {actual node : (previous node, distance from start)}
    # print("sets_sequence",sets_sequence)
    start_node = sets_dic[sets_sequence[0]][0]
    distances = [{start_node:(-1, 0)}]
    print("find_minimal_distance_to")
    for i in range(1, len(sets_sequence)):
        prev_set_id = sets_sequence[i - 1]
        previous_distances = distances[i - 1]
        act_set_id = sets_sequence[i]
        set_nodes = sets_dic[act_set_id]
        # print("prev_set_id",prev_set_id)
        # print("act_set_id",act_set_id)
        distances_to_actual_set = {}
        for act_node_id in set_nodes:
            act_node = node_dict[act_node_id]
            # print("previous_distances",previous_distances,"act_node",act_node_id)
            for prev_node_id in previous_distances:
                dist_to_prev = previous_distances[prev_node_id][1]
                prev_node = node_dict[prev_node_id]
                dist_act = math.ceil(math.sqrt((act_node[0] - prev_node[0]) ** 2 + (act_node[1] - prev_node[1]) ** 2))
                new_dist = dist_to_prev + dist_act
                if distances_to_actual_set.get(act_node_id) is None:
                    distances_to_actual_set[act_node_id] = (prev_node_id, new_dist)
                else:
                    if new_dist < distances_to_actual_set[act_node_id][1]:
                        distances_to_actual_set[act_node_id] = (prev_node_id, new_dist)
        # print("distances_to_actual_set",distances_to_actual_set)  
        distances.append(distances_to_actual_set)
    # print(distances)
    min_distance_end = sys.float_info.max
    for prev_node_id in distances[-1]:
        if distances[-1][prev_node_id][1] < min_distance_end:
            min_distance_end = distances[-1][prev_node_id][1]
    print("min_distance_end", min_distance_end)
    return distances


def get_path_from_distance(distances_to):
    min_to_dist = sys.float_info.max
    min_to_idx = 0
    for dist_rec_key in distances_to[-1]:
        dist_rec = distances_to[-1][dist_rec_key]
        if dist_rec[1] < min_to_dist:
            min_to_dist = dist_rec[1]
            min_to_idx = dist_rec_key
    
    # print("min_to_dist",min_to_dist)
    # print("min_to_idx",min_to_idx)
    
    path_nodes = [min_to_idx]
    looking_idx = -1
    while min_to_idx >= 0:
        for actual_node_id in distances_to[looking_idx]:
            if actual_node_id == min_to_idx:
                dist_rec = distances_to[looking_idx][actual_node_id]
                min_to_idx = dist_rec[0]
                if min_to_idx >= 0:
                    path_nodes.append(min_to_idx)
                looking_idx -= 1
                break
    path_nodes = list(reversed(path_nodes))
    return path_nodes


def plot_polygon(polygon, fc='#999999', ec='#000000'):
    x_min, y_min, x_max, y_max = polygon.bounds
    ax = plt.gca()
    margin = .3
    ax.set_xlim([x_min - margin, x_max + margin])
    ax.set_ylim([y_min - margin, y_max + margin])
    patch = PolygonPatch(polygon, fc=fc, lw=0.1,
                         ec=ec, fill=True,
                         zorder=-1)
    ax.add_patch(patch)
    #return fig


def alpha_shape(points, alpha):
    # print("booo")
    # print(points)
    if len(points) < 4:
        # print("less")
        return geometry.MultiPoint(list(points)).convex_hull
    # print("do")
    
    def add_edge(edges, edge_points, coords, i, j):
        """
        Add a line between the i-th and j-th points,
        if not in the list already
        """
        if (i, j) in edges or (j, i) in edges:
            # already added
            return
        edges.add((i, j))
        edge_points.append(coords[ [i, j] ])
    
    # print("bo")
    coords = np.array([point for point in points])
    # print("coords",coords)
    tri = Delaunay(coords)
    edges = set()
    edge_points = []
    # loop over triangles:
    # ia, ib, ic = indices of corner points of the
    # triangle
    for ia, ib, ic in tri.vertices:
        pa = coords[ia]
        pb = coords[ib]
        pc = coords[ic]
        # print(ia,ib,ic)
        # Lengths of sides of triangle
        a = math.sqrt((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2)
        b = math.sqrt((pb[0] - pc[0]) ** 2 + (pb[1] - pc[1]) ** 2)
        c = math.sqrt((pc[0] - pa[0]) ** 2 + (pc[1] - pa[1]) ** 2)
        # Semiperimeter of triangle
        s = (a + b + c) / 2.0
        # print("s",s)
        # Area of triangle by Heron's formula
        area = math.sqrt(s * (s - a) * (s - b) * (s - c))
        # print("area",area)
        circum_r = a * b * c / (4.0 * area)
        # print("circum_r",circum_r)
        
        # print("alpha",alpha)
        # Here's the radius filter.
        # print circum_r
        if alpha == 0 or circum_r < 1.0 / alpha:
            # print("add edge")
            add_edge(edges, edge_points, coords, ia, ib)
            add_edge(edges, edge_points, coords, ib, ic)
            add_edge(edges, edge_points, coords, ic, ia)
    # print("booo2")
    m = geometry.MultiLineString(edge_points)
    triangles = list(polygonize(m))
    return cascaded_union(triangles)
