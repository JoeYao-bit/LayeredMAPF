import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt

def loadDataFromfile(file_path):
    data_list = list()
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()#[1:]
            for line in lines:
                #print(line)
                # 处理每一行数据
                #print(line.strip())
                splited_line = line.split()
                
                head_split = splited_line[0].split('_')
                
                new_data = LineData()
                
                new_data.method = splited_line[0]
                new_data.agent_count = int(splited_line[1])
                new_data.time_cost = float(splited_line[2])
                new_data.total_cost = float(splited_line[3])
                new_data.max_single_cost = float(splited_line[4])
                new_data.success = int(splited_line[5])
                new_data.max_memory_usage = float(splited_line[6])

                if head_split[0] == 'LAYERED':
                    new_data.cluster_decomposition_time_cost = float(splited_line[7])
                    new_data.sort_level_time_cost = float(splited_line[8])
                
                data_list.append(new_data)
            #print(new_data.method, ' ', new_data.path_count, ' ', new_data.real_path_count, ' ', new_data.time_cost)
    except Exception as e:            
        print(e)             
    return data_list

class LineData:
    method = ''
    agent_count = 0
    time_cost = 0
    total_cost = 0 
    max_single_cost = 0.
    success = 0
    max_memory_usage = 0.
    # specific time component for layered MAPF
    cluster_decomposition_time_cost = 0.
    sort_level_time_cost = 0.
        

# all data in a txt file
class SingleTestData:
    data_list = list()
    map_name = ''
    
def drawMethodMap(all_data_map, xlabel, ylable, title, is_percentage=False):
    fig=plt.figure(figsize=(5,3.5)) #添加绘图框
    for map_key, map_value in all_data_map.items():
        method_name = map_key
        x = list()
        y = list()
        std_var = list()
        
        sorted_keys = sorted(all_data_map[method_name].keys())
        
        for agent_size_key in sorted_keys:
            x.append(agent_size_key)
            y.append(np.mean(all_data_map[method_name][agent_size_key]))
            std_var.append(np.std(all_data_map[method_name][agent_size_key]))
        
        plt.errorbar(x, y, fmt=map_and_marker[method_name], label=map_key, elinewidth=2, capsize=4)
               
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylable)
    if is_percentage:
        plt.ylim(0, 1)
    plt.legend(loc='best', fontsize = 8, ncol=2)
    #plt.grid()
    plt.tight_layout()
    plt.savefig('../test/pic/layered_MAPF/'+title+"-"+method_name+"-"+ylable+".png", dpi = 400, bbox_inches='tight')   
    #break    
     
    
data_path_dir = '../test/test_data/layered_mapf/'
all_map_name = ["empty-32-32",
                "random-32-32-20-random-1",
                "warehouse-10-20-10-2-1",
                "maze-32-32-2-random-1",
                # "maze-32-32-4-random-1",
                # "den312d-random-1",
                # "Berlin_1_256-random-1",
                # "Paris_1_256-random-1",
                # "den520d-random-1"
                ]
all_single_data = list()

#map_and_marker = {"Berlin_1_256":'D-', "Boston_2_256":'o-', "Denver_2_256":'s-', "London_2_256":'p-', 
#                  "Milan_2_256":'s-',  "Moscow_2_256":'<-', "Paris_0_256":'>-',  "Sydney_1_256":'h-'}

name_of_decomposition = "DECOMPOSITION"

map_and_marker = {"RAW_EECBS":'D-',       "LAYERED_EECBS":'o-',
                  "RAW_LaCAM":'D-',       "LAYERED_LaCAM":'o-',
                  "RAW_PBS":'D-',         "LAYERED_PBS":'o-',
                  "RAW_LaCAM2":'D-',      "LAYERED_LaCAM2":'o-',
                  "RAW_LNS":'D-',         "LAYERED_LNS":'o-',
                  "RAW_AnytimeBCBS":'D-', "LAYERED_AnytimeBCBS":'o-',
                  "RAW_AnytimeEECBS":'D-', "LAYERED_AnytimeEECBS":'o-',
                  "RAW_CBSH2_RTC":'D-',   "LAYERED_CBSH2_RTC":'o-',
                  "RAW_PIBT":'D-',        "LAYERED_PIBT":'o-',
                  "RAW_PIBT2":'D-',       "LAYERED_PIBT2":'o-',
                  "RAW_HCA":'D-',         "LAYERED_HCA":'o-',
                  "RAW_PushAndSwap":'D-', "LAYERED_PushAndSwap":'o-',
                  name_of_decomposition:"D-"
                  }


# 1, load all data
for map_name in all_map_name:
    data_file_path = data_path_dir + map_name + '.txt'
    print('load data from', data_file_path)
    single = SingleTestData() # 不带括号则均指向同一元素
    single.map_name = map_name
    single.data_list = loadDataFromfile(data_file_path)
    all_single_data.append(single)
    
for single_data in all_single_data:
    
    all_method_time_cost_map = dict()
    all_method_time_cost_map[name_of_decomposition] = dict()
    all_method_total_cost_map = dict()
    all_method_max_single_cost_map = dict()
    all_method_success_rate_map = dict()
    all_method_memory_usage_map = dict()
    all_method_cluster_cost_map = dict()
    all_method_level_sort_map = dict()

    for line_data in single_data.data_list:
        
        # if line_data.method != "RAW_EECBS" and line_data.method != "LAYERED_EECBS":
        #     continue
        
        if line_data.method != "RAW_PushAndSwap" and line_data.method != "LAYERED_PushAndSwap":
            continue
               
        if all_method_time_cost_map.get(line_data.method) == None:
            all_method_time_cost_map[line_data.method] = dict()
            all_method_total_cost_map[line_data.method] = dict()
            all_method_max_single_cost_map[line_data.method] = dict()
            all_method_memory_usage_map[line_data.method] = dict()
            all_method_success_rate_map[line_data.method] = dict()
            all_method_cluster_cost_map[line_data.method] = dict()
            all_method_level_sort_map[line_data.method] = dict()
            
        if all_method_time_cost_map[line_data.method].get(line_data.agent_count) == None:
            all_method_time_cost_map[line_data.method][line_data.agent_count] = list()
            all_method_time_cost_map[name_of_decomposition][line_data.agent_count] = list()
            all_method_total_cost_map[line_data.method][line_data.agent_count] = list()
            all_method_max_single_cost_map[line_data.method][line_data.agent_count] = list()    
            all_method_memory_usage_map[line_data.method][line_data.agent_count] = list()    
            all_method_success_rate_map[line_data.method][line_data.agent_count] = list() 
            all_method_cluster_cost_map[line_data.method][line_data.agent_count] = list()    
            all_method_level_sort_map[line_data.method][line_data.agent_count] = list()
        
        all_method_time_cost_map[line_data.method][line_data.agent_count].append(line_data.time_cost)
        all_method_time_cost_map[name_of_decomposition][line_data.agent_count].append(line_data.cluster_decomposition_time_cost + line_data.sort_level_time_cost)
        all_method_total_cost_map[line_data.method][line_data.agent_count].append(line_data.total_cost)
        all_method_max_single_cost_map[line_data.method][line_data.agent_count].append(line_data.max_single_cost)
        all_method_memory_usage_map[line_data.method][line_data.agent_count].append(line_data.max_memory_usage)
        all_method_success_rate_map[line_data.method][line_data.agent_count].append(line_data.success)
        all_method_cluster_cost_map[line_data.method][line_data.agent_count].append(line_data.cluster_decomposition_time_cost)
        all_method_level_sort_map[line_data.method][line_data.agent_count].append(line_data.sort_level_time_cost)
        
    drawMethodMap(all_method_time_cost_map, "Number of agents", "Time cost(ms)", "time_cost/"+single_data.map_name)        
    drawMethodMap(all_method_success_rate_map, "Number of agents", "Success rate", "success_rate/"+single_data.map_name, True)        
    drawMethodMap(all_method_memory_usage_map, "Number of agents", "Memory Usage(MB)", "memory_usage/"+single_data.map_name)        

    #break

#plt.show()