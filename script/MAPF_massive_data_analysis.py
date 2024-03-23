import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker

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
        temp_splitted_name = method_name.split('_')
        if len(temp_splitted_name) > 1:
            splitted_name = temp_splitted_name[1]
        x = list()
        y = list()
        std_var = list()
        
        sorted_keys = sorted(all_data_map[method_name].keys())
        
        for agent_size_key in sorted_keys:
            x.append(agent_size_key)
            y.append(np.mean(all_data_map[method_name][agent_size_key]))
            std_var.append(np.std(all_data_map[method_name][agent_size_key]))
        
        plt.errorbar(x, y, std_var, fmt=map_and_marker[method_name], markersize=14, label=map_key, linewidth=4, elinewidth=4, capsize=4)
               
    plt.legend(loc='best')    
    plt.tick_params(axis='both', labelsize=18)
    #plt.ticklabel_format(style='sci', scilimits=(0,0), axis='y')
    
    formater = ticker.ScalarFormatter(useMathText=True) 
    formater.set_scientific(True)
    formater.set_powerlimits((0,0))
    
    ax = plt.gca()
    ax.yaxis.offsetText.set_fontsize(18)
    ax.yaxis.set_major_formatter(formater)           
               
    # plt.title(title)
    # plt.xlabel(xlabel)
    # plt.ylabel(ylable)
    y_range = plt.ylim()
    plt.ylim(0, y_range[1])
    if is_percentage:
        plt.ylim(0, 1)
    plt.legend(loc='best', fontsize = 16, ncol=1, handletextpad=.5, framealpha=0.5)
    #plt.grid()
    plt.tight_layout()
    save_path = '../test/pic/layered_MAPF/'+splitted_name+"/"+title+".png"
    plt.savefig(save_path, dpi = 400, bbox_inches='tight')   
    print("save picture to "+save_path)
    plt.close()
    #break    
     
    
data_path_dir = '../test/test_data/layered_mapf/'
all_map_name = ["empty-16-16"
                # "empty-32-32",
                # "random-32-32-20-random-1",
                # "warehouse-10-20-10-2-1",
                # "maze-32-32-2-random-1",
                # "maze-32-32-4-random-1",
                # "den312d-random-1",
                # "den520d-random-1",
                # "Berlin_1_256-random-1",
                # "Paris_1_256-random-1",
                # "ht_chantry",
                # "lak303d"
                ]
all_single_data = list()

#map_and_marker = {"Berlin_1_256":'D-', "Boston_2_256":'o-', "Denver_2_256":'s-', "London_2_256":'p-', 
#                  "Milan_2_256":'s-',  "Moscow_2_256":'<-', "Paris_0_256":'>-',  "Sydney_1_256":'h-'}

name_of_decomposition = "DECOMPOSITION"

map_and_marker = {"RAW_EECBS":'x-',       "LAYERED_EECBS":'o--',
                  "RAW_LaCAM":'x-',       "LAYERED_LaCAM":'o--',
                  "RAW_PBS":'x-',         "LAYERED_PBS":'o--',
                  "RAW_LaCAM2":'x-',      "LAYERED_LaCAM2":'o--',
                  "RAW_LNS":'x-',         "LAYERED_LNS":'o--',
                  "RAW_AnytimeBCBS":'x-', "LAYERED_AnytimeBCBS":'o--',
                  "RAW_AnytimeEECBS":'x-', "LAYERED_AnytimeEECBS":'o--',
                  "RAW_CBSH2_RTC":'x-',   "LAYERED_CBSH2_RTC":'o--',
                  "RAW_PIBT":'x-',        "LAYERED_PIBT":'o--',
                  "RAW_PIBT2":'x-',       "LAYERED_PIBT2":'o--',
                  "RAW_HCA":'x-',         "LAYERED_HCA":'o--',
                  "RAW_PushAndSwap":'x-', "LAYERED_PushAndSwap":'o--',
                  name_of_decomposition:"^-."
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
    all_method_total_cost_map = dict()
    all_method_max_single_cost_map = dict()
    all_method_success_rate_map = dict()
    all_method_memory_usage_map = dict()
    all_method_cluster_cost_map = dict()
    all_method_level_sort_map = dict()

    for line_data in single_data.data_list:
        
        # if line_data.method != "RAW_EECBS" and line_data.method != "LAYERED_EECBS":
        #     continue
        
        # if line_data.method != "RAW_PushAndSwap" and line_data.method != "LAYERED_PushAndSwap":
        #     continue
               
        split_method_name = line_data.method.split("_")
        if all_method_time_cost_map.get(split_method_name[1]) == None:
            all_method_time_cost_map[split_method_name[1]] = dict()
            all_method_total_cost_map[split_method_name[1]] = dict()
            all_method_max_single_cost_map[split_method_name[1]] = dict()
            all_method_memory_usage_map[split_method_name[1]] = dict()
            all_method_success_rate_map[split_method_name[1]] = dict()
            all_method_cluster_cost_map[split_method_name[1]] = dict()
            all_method_level_sort_map[split_method_name[1]] = dict()
               
        if all_method_time_cost_map[split_method_name[1]].get(line_data.method) == None:
            all_method_time_cost_map[split_method_name[1]][line_data.method] = dict()
            all_method_time_cost_map[split_method_name[1]][name_of_decomposition] = dict()            
            all_method_total_cost_map[split_method_name[1]][line_data.method] = dict()
            all_method_max_single_cost_map[split_method_name[1]][line_data.method] = dict()
            all_method_memory_usage_map[split_method_name[1]][line_data.method] = dict()
            all_method_success_rate_map[split_method_name[1]][line_data.method] = dict()
            all_method_cluster_cost_map[split_method_name[1]][line_data.method] = dict()
            all_method_level_sort_map[split_method_name[1]][line_data.method] = dict()
            
        if all_method_time_cost_map[split_method_name[1]][line_data.method].get(line_data.agent_count) == None:
            all_method_time_cost_map[split_method_name[1]][line_data.method][line_data.agent_count] = list()
            all_method_time_cost_map[split_method_name[1]][name_of_decomposition][line_data.agent_count] = list()
            all_method_total_cost_map[split_method_name[1]][line_data.method][line_data.agent_count] = list()
            all_method_max_single_cost_map[split_method_name[1]][line_data.method][line_data.agent_count] = list()    
            all_method_memory_usage_map[split_method_name[1]][line_data.method][line_data.agent_count] = list()    
            all_method_success_rate_map[split_method_name[1]][line_data.method][line_data.agent_count] = list() 
            all_method_cluster_cost_map[split_method_name[1]][line_data.method][line_data.agent_count] = list()    
            all_method_level_sort_map[split_method_name[1]][line_data.method][line_data.agent_count] = list()
        
        all_method_time_cost_map[split_method_name[1]][line_data.method][line_data.agent_count].append(line_data.time_cost)
        all_method_time_cost_map[split_method_name[1]][name_of_decomposition][line_data.agent_count].append(line_data.cluster_decomposition_time_cost + line_data.sort_level_time_cost)
        all_method_total_cost_map[split_method_name[1]][line_data.method][line_data.agent_count].append(line_data.total_cost)
        all_method_max_single_cost_map[split_method_name[1]][line_data.method][line_data.agent_count].append(line_data.max_single_cost)
        all_method_memory_usage_map[split_method_name[1]][line_data.method][line_data.agent_count].append(line_data.max_memory_usage)
        all_method_success_rate_map[split_method_name[1]][line_data.method][line_data.agent_count].append(line_data.success)
        all_method_cluster_cost_map[split_method_name[1]][line_data.method][line_data.agent_count].append(line_data.cluster_decomposition_time_cost)
        all_method_level_sort_map[split_method_name[1]][line_data.method][line_data.agent_count].append(line_data.sort_level_time_cost)
        
        
    for method_key, method_value in all_method_time_cost_map.items():
        drawMethodMap(all_method_time_cost_map[method_key], "Number of agents", "Time cost(ms)", "time_cost/"+single_data.map_name)        
        drawMethodMap(all_method_success_rate_map[method_key], "Number of agents", "Success rate", "success_rate/"+single_data.map_name, True)        
        drawMethodMap(all_method_memory_usage_map[method_key], "Number of agents", "Memory Usage(MB)", "memory_usage/"+single_data.map_name)        
        drawMethodMap(all_method_total_cost_map[method_key], "Number of agents", "Sum of cost", "sum_of_cost/"+single_data.map_name)   
        drawMethodMap(all_method_max_single_cost_map[method_key], "Number of agents", "Makespan", "makespan/"+single_data.map_name)   
    #break

#plt.show()