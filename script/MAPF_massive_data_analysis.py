import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker
import os

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
    
def drawMethodMaps(all_data_map, xlable, ylable, title, is_percentage=False):    
    map_and_agent_data = dict()
    for map_key, map_value in all_data_map.items():
        fig=plt.figure(figsize=(5,3.5)) #添加绘图框
        for method_key, method_value in all_data_map[map_key].items():
            x = list()
            y = list()
            std_var = list()
            sorted_keys = sorted(all_data_map[map_key][method_key].keys())
            
            splited_method_name = method_key.split('_')
            for agent_size_key in sorted_keys:
                x.append(agent_size_key)
                y.append(np.mean(all_data_map[map_key][method_key][agent_size_key]))
                std_var.append(np.std(all_data_map[map_key][method_key][agent_size_key]))
            # fmt=method_marker_map[method_key]
            if len(splited_method_name) == 1:
                plt.errorbar(x, y, fmt=map_format_map[map_key]+method_marker_map2[name_of_decomposition], markersize=14, label=map_key+"/"+name_of_decomposition, linewidth=2, elinewidth=4, capsize=4)
            else:
                plt.errorbar(x, y, fmt=map_format_map[map_key]+method_marker_map2[splited_method_name[0]], markersize=14, label=map_key+"/"+splited_method_name[0], linewidth=2, elinewidth=4, capsize=4)
                
        plt.tick_params(axis='both', labelsize=18)
        formater = ticker.ScalarFormatter(useMathText=True) 
        formater.set_scientific(True)
        formater.set_powerlimits((0,0))
        ax = plt.gca()
        ax.yaxis.offsetText.set_fontsize(18)
        ax.yaxis.set_major_formatter(formater)           
        y_range = plt.ylim()
        plt.ylim(0, y_range[1])
        plt.title(ylable)
        if is_percentage:
            plt.ylim(0, 1)
        plt.legend(loc='best', fontsize = 16, ncol=1, handletextpad=.5, framealpha=0.5)
        plt.tight_layout()        
        save_path = '../test/pic/layered_MAPF/'+title
        
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            print("Folder: " + save_path + " created")
            
        save_path = save_path + "/" + map_key
        plt.savefig(save_path, dpi = 400, bbox_inches='tight')   
        plt.close()
        print("save path to " + save_path)
     
    
data_path_dir = '../test/test_data/layered_mapf/'
# all_map_name = ["empty-16-16",
#                 "empty-32-32",
#                 "random-32-32-20",
#                 "warehouse-10-20-10-2-1",
#                 "maze-32-32-2"
#                 "maze-32-32-4",
#                 "den312d",
#                 "den520d",
#                 "Berlin_1_256",
#                 "Paris_1_256",
#                 "ht_chantry",
#                 "lak303d"
#                 ]
all_single_data = list()

#map_and_marker = {"Berlin_1_256":'D-', "Boston_2_256":'o-', "Denver_2_256":'s-', "London_2_256":'p-', 
#                  "Milan_2_256":'s-',  "Moscow_2_256":'<-', "Paris_0_256":'>-',  "Sydney_1_256":'h-'}

name_of_decomposition = "DECOMPOSITION"

method_marker_map = {"RAW_EECBS":'p-',       "LAYERED_EECBS":'p--',
                  "RAW_LaCAM":'P-',       "LAYERED_LaCAM":'P--',
                  "RAW_PBS":'D-',         "LAYERED_PBS":'D--',
                  "RAW_LaCAM2":'X-',      "LAYERED_LaCAM2":'X--',
                  "RAW_LNS":'+-',         "LAYERED_LNS":'+--',
                  "RAW_AnytimeBCBS":'*-', "LAYERED_AnytimeBCBS":'*--',
                  "RAW_AnytimeEECBS":'o-', "LAYERED_AnytimeEECBS":'o--',
                  "RAW_CBSH2_RTC":'v-',   "LAYERED_CBSH2_RTC":'v--',
                  "RAW_PIBT":'^-',        "LAYERED_PIBT":'^--',
                  "RAW_PIBT2":'<-',       "LAYERED_PIBT2":'<--',
                  "RAW_HCA":'>-',         "LAYERED_HCA":'>--',
                  "RAW_PushAndSwap":'H-', "LAYERED_PushAndSwap":'H--',
                  name_of_decomposition:"D-."
                  }

method_marker_map2 = {
    "RAW":'-',
    "LAYERED":'--',
    name_of_decomposition:"-."
}

method_color_map = {
                  "RAW_EECBS":'p-',       "LAYERED_EECBS":'p--',
                  "RAW_LaCAM":'P-',       "LAYERED_LaCAM":'P--',
                  "RAW_PBS":'D-',         "LAYERED_PBS":'D--',
                  "RAW_LaCAM2":'X-',      "LAYERED_LaCAM2":'X--',
                  "RAW_LNS":'+-',         "LAYERED_LNS":'+--',
                  "RAW_AnytimeBCBS":'*-', "LAYERED_AnytimeBCBS":'*--',
                  "RAW_AnytimeEECBS":'o-', "LAYERED_AnytimeEECBS":'o--',
                  "RAW_CBSH2_RTC":'v-',   "LAYERED_CBSH2_RTC":'v--',
                  "RAW_PIBT":'^-',        "LAYERED_PIBT":'^--',
                  "RAW_PIBT2":'<-',       "LAYERED_PIBT2":'<--',
                  "RAW_HCA":'>-',         "LAYERED_HCA":'>--',
                  "RAW_PushAndSwap":'H-', "LAYERED_PushAndSwap":'H--',
                  name_of_decomposition:"D-."
                  }

map_format_map = {
                 "empty-16-16":'o',
                 "empty-32-32":'o',
                 
                 "maze-32-32-2":'*',
                 "maze-32-32-4":'*',
                 "maze-128-128-2":'*',
                 "maze-128-128-10":'*',
                 
                 "den312d":'v',
                 "den520d":'v',
                 
                 "Berlin_1_256":'<',
                 "Paris_1_256":'<',
                 
                 "ht_chantry":'H',
                 "lak303d":'H',
                 
                 "random-32-32-20":'D',
                 "random-64-64-10":'D',  # maximum 8s
                 "random-64-64-20":'D',
                 
                 "room-32-32-4":'X',
                 "room-64-64-8":'X',
                 "room-64-64-16":'X',

                 "warehouse-10-20-10-2-1":'+',
                 "warehouse-20-40-10-2-2":'+',
                 "warehouse-20-40-20-2-1":'+',
                 "warehouse-20-40-20-2-2":'+',

}

# 1, load all data
for map_name_key, map_format_value in map_format_map.items():
    data_file_path = data_path_dir + map_name_key + '.txt'
    print('load data from', data_file_path)
    single = SingleTestData() # 不带括号则均指向同一元素
    single.map_name = map_name_key
    single.data_list = loadDataFromfile(data_file_path)
    all_single_data.append(single)
    

all_method_time_cost_map = dict()
all_method_total_cost_map = dict()
all_method_makespan_map = dict()
all_method_success_rate_map = dict()
all_method_memory_usage_map = dict()
all_method_cluster_cost_map = dict()
all_method_level_sort_map = dict()    
    
for single_data in all_single_data:
    #map_name = single.map_name
    for line_data in single_data.data_list:
        
        if line_data.method != "RAW_EECBS" and line_data.method != "LAYERED_EECBS":
           continue
        
        # if line_data.method != "RAW_PushAndSwap" and line_data.method != "LAYERED_PushAndSwap":
        #     continue
        
        #print("map name1 "+single.map_name)
             
        splited_method_name = line_data.method.split("_")
        method_name = splited_method_name[1]
        if method_name == "AnytimeEECBS":
            continue
        if method_name == "AnytimeBCBS":
            continue
        if method_name == "PIBT":
            continue
        if method_name == "LaCAM":
            continue        
        # if all_method_time_cost_map.get(split_method_name[1]) == None:
        #     all_method_time_cost_map = dict()
        #     all_method_total_cost_map = dict()
        #     all_method_makespan_map = dict()
        #     all_method_memory_usage_map = dict()
        #     all_method_success_rate_map = dict()
        #     all_method_cluster_cost_map = dict()
        #     all_method_level_sort_map = dict()
               
        if all_method_time_cost_map.get(method_name) == None:
            all_method_time_cost_map[method_name] = dict()
        
        if all_method_total_cost_map.get(method_name) == None:                
            all_method_total_cost_map[method_name] = dict()
            
        if all_method_makespan_map.get(method_name) == None:              
            all_method_makespan_map[method_name] = dict()
        
        if all_method_memory_usage_map.get(method_name) == None:    
            all_method_memory_usage_map[method_name] = dict()
        
        if all_method_success_rate_map.get(method_name) == None:    
            all_method_success_rate_map[method_name] = dict()
                   
        if all_method_time_cost_map[method_name].get(single_data.map_name) == None:
            all_method_time_cost_map[method_name][single_data.map_name] = dict()
            all_method_total_cost_map[method_name][single_data.map_name] = dict()
            all_method_makespan_map[method_name][single_data.map_name] = dict()
            all_method_memory_usage_map[method_name][single_data.map_name] = dict()
            all_method_success_rate_map[method_name][single_data.map_name] = dict()

        if all_method_time_cost_map[method_name][single_data.map_name].get(line_data.method) == None:
            all_method_time_cost_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_time_cost_map[method_name][single_data.map_name][name_of_decomposition] = dict()
            all_method_total_cost_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_makespan_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_memory_usage_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_success_rate_map[method_name][single_data.map_name][line_data.method] = dict()
            
        if all_method_time_cost_map[method_name][single_data.map_name][line_data.method].get(line_data.agent_count) == None:
            all_method_time_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()
            all_method_time_cost_map[method_name][single_data.map_name][name_of_decomposition][line_data.agent_count] = list()
            all_method_total_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()
            all_method_makespan_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()    
            all_method_memory_usage_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()    
            all_method_success_rate_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list() 

            
        all_method_time_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.time_cost)
        all_method_time_cost_map[method_name][single_data.map_name][name_of_decomposition][line_data.agent_count].append(line_data.cluster_decomposition_time_cost + line_data.sort_level_time_cost)
        all_method_success_rate_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.success)    
        all_method_memory_usage_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.max_memory_usage)

        
        if line_data.success == 1:
            all_method_total_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.total_cost)
            all_method_makespan_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.max_single_cost)

for method_key, method_value in all_method_time_cost_map.items(): 
    drawMethodMaps(all_method_time_cost_map[method_key], "Number of agents", "Time cost(ms)", "time_cost/"+method_key)           
    drawMethodMaps(all_method_memory_usage_map[method_key], "Number of agents", "Memory usage(ms)", "memory_usage/"+method_key)           
    drawMethodMaps(all_method_total_cost_map[method_key], "Number of agents", "Sum of cost", "sum_of_cost/"+method_key)           
    drawMethodMaps(all_method_makespan_map[method_key], "Number of agents", "Makespan", "makespan/"+method_key)           
    drawMethodMaps(all_method_success_rate_map[method_key], "Number of agents", "Success rate", "success_rate/"+method_key)           
   
    #break;    
    # for method_key, method_value in all_method_time_cost_map.items():
    #     drawMethodMap(all_method_time_cost_map[method_key], "Number of agents", "Time cost(ms)", "time_cost/"+single_data.map_name)        
    #     drawMethodMap(all_method_success_rate_map[method_key], "Number of agents", "Success rate", "success_rate/"+single_data.map_name, True)        
    #     drawMethodMap(all_method_memory_usage_map[method_key], "Number of agents", "Memory Usage(MB)", "memory_usage/"+single_data.map_name)        
    #     drawMethodMap(all_method_total_cost_map[method_key], "Number of agents", "Sum of cost", "sum_of_cost/"+single_data.map_name)   
    #     drawMethodMap(all_method_max_single_cost_map[method_key], "Number of agents", "Makespan", "makespan/"+single_data.map_name)   
    #break
    #drawMethodMap(all_method_time_cost_map, "Number of agents", "Time cost(ms)", "time_cost/"+single_data.map_name)        
    # drawMethodMap(all_method_success_rate_map, "Number of agents", "Success rate", "success_rate/"+single_data.map_name, True)        
    # drawMethodMap(all_method_memory_usage_map, "Number of agents", "Memory Usage(MB)", "memory_usage/"+single_data.map_name)        
    # drawMethodMap(all_method_total_cost_map, "Number of agents", "Sum of cost", "sum_of_cost/"+single_data.map_name)   
    # drawMethodMap(all_method_max_single_cost_map, "Number of agents", "Makespan", "makespan/"+single_data.map_name)   
    
#plt.show()
