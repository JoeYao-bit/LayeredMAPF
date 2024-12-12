import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker
import os

def loadDataFromfile(file_path):
    data_list = list()
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()
            for line in lines:
                splited_line = line.split()
                
                head_split = splited_line[0].split('_')
                
                new_data = LineData()
                
                new_data.method = splited_line[0]
                new_data.agent_count = int(splited_line[1])
                new_data.time_cost = float(splited_line[2])
                new_data.total_cost = int(splited_line[3])
                new_data.max_single_cost = int(splited_line[4])
                new_data.success = int(splited_line[5])
                new_data.max_memory_usage = float(splited_line[6])
                
                if np.isnan(new_data.total_cost):
                    continue
                if np.isnan(new_data.max_single_cost):
                    continue
                
                if new_data.time_cost > 35000:
                    new_data.success = 0
                
                if new_data.time_cost > 30000:
                    new_data.time_cost = 30000
                    

                if head_split[0] == 'LAYERED':
                    new_data.subgraph_init_time_cost = float(splited_line[7])
                    new_data.decomposition_time_cost = float(splited_line[8])
                
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
    max_single_cost = 0
    success = 0
    max_memory_usage = 0.
    # specific time component for layered MAPF
    subgraph_init_time_cost = 0.
    decomposition_time_cost = 0.
        

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
            sorted_keys = sorted(all_data_map[map_key][method_key].keys())
            
            splited_method_name = method_key.split('_')
            for agent_size_key in sorted_keys:
                x.append(agent_size_key)
                if len(all_data_map[map_key][method_key][agent_size_key]) > 0:
                    y.append(np.mean(all_data_map[map_key][method_key][agent_size_key]))
                else:
                    y.append(0)
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
        if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage(MB)":
            ax.set_yscale('log')  
            plt.ylim(1, y_range[1]*10)  
        else:    
            plt.ylim(0, y_range[1])
        plt.title(ylable)
        if is_percentage:
            plt.ylim(0, 1)
                
        plt.tight_layout()
        save_path = '../test/pic/layered_MAPF/'+title
        
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            print("Folder: " + save_path + " created")
            
        save_path = save_path + "/" + map_key
        plt.savefig(save_path, dpi = 400, bbox_inches='tight')   
        plt.close()
        print("save path to " + save_path)


def drawMethodMapAgentSizes(all_data_map, xlable, ylable, title, is_percentage=False):    
    map_and_agent_data = dict()
    fig=plt.figure(figsize=(5,3.5)) #添加绘图框
    for map_key, map_value in all_data_map.items():
        for method_key, method_value in all_data_map[map_key].items():
            x = list()
            y = list()
            sorted_keys = sorted(all_data_map[map_key][method_key].keys())
            
            splited_method_name = method_key.split('_')
            for agent_size_key in sorted_keys:
                x.append(agent_size_key)
                if len(all_data_map[map_key][method_key][agent_size_key]) > 0:
                    y.append(np.mean(all_data_map[map_key][method_key][agent_size_key]))
                else:
                    y.append(0)
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
    if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage(MB)":
        ax.set_yscale('log')
        plt.ylim(1, y_range[1]*10)
    else:
        plt.ylim(0, y_range[1])
    plt.title(ylable)
    if is_percentage:
        plt.ylim(0, 1)
            
    plt.tight_layout()
    save_path = '../test/pic/layered_MAPF/'+title
    
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
        
    save_path = save_path + "/multi_map"
    plt.savefig(save_path, dpi = 400, bbox_inches='tight')
    plt.close()
    print("save path to " + save_path)



def drawSummaryOfMap(all_data_map, xlable, ylable, title, is_percentage=False):
    map_and_agent_data = dict()
    fig=plt.figure(figsize=(5,3.5)) #添加绘图框 
    map_lists = list()
    value_lists_raw = list()
    value_lists_layered = list()
    width = 0.4 
    for map_key, map_value in all_data_map.items():
        map_lists.append(map_key)
        for method_key, method_value in all_data_map[map_key].items():
            value = list()           
            for agent_size_key in all_data_map[map_key][method_key].keys():
                if ylable == "Makespan" or ylable == "Sum of cost":
                    if len(all_data_map[map_key][method_key][agent_size_key]) > 0:
                        # only considering success cases
                        success_cases = list()
                        for case in all_data_map[map_key][method_key][agent_size_key]:
                            success_cases.append(case)
                        value.append(np.mean(success_cases))
                    else:
                        value.append(0)
                else:
                    if len(all_data_map[map_key][method_key][agent_size_key]) > 0:
                        value.append(np.mean(all_data_map[map_key][method_key][agent_size_key]))
                    else:
                        value.append(0)      
            head_split = method_key.split('_')
            if head_split[0] == 'LAYERED':  
                plt.bar(map_format_list.index(map_key)+1+width/2, np.mean(value), width, hatch="//")    
                value_lists_layered.extend(value)
                
            if head_split[0] == 'RAW':
                plt.bar(map_format_list.index(map_key)+1-width/2, np.mean(value), width)    
                value_lists_raw.extend(value)
                
        plt.xticks(rotation=70)         
    
    print(title + "/" + ylable + "layered = " + str(np.mean(value_lists_layered)) + " / raw = " + str(np.mean(value_lists_raw)))
    
    plt.tick_params(axis='both', labelsize=14)
    plt.xticks(np.arange(1, len(map_format_list)+1, 1))
    formater = ticker.ScalarFormatter(useMathText=True) 
    formater.set_scientific(True)
    formater.set_powerlimits((0,0))
    ax = plt.gca()
    ax.yaxis.offsetText.set_fontsize(18)
    ax.yaxis.set_major_formatter(formater)      
    
    y_range = plt.ylim()      
    if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage(MB)":
        ax.set_yscale('log')  
        plt.ylim(1, y_range[1]*10)  
    else:    
        plt.ylim(0, y_range[1])
    plt.title(ylable)
    if is_percentage:
        plt.ylim(0, 1)
            
    #plt.legend(loc='best', fontsize = 16, ncol=1, handletextpad=.5, framealpha=0.5)
    plt.tight_layout()        
    save_path = '../test/pic/layered_MAPF/'+title
    
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
        
    save_path = save_path + "/" + 'summary'
    plt.savefig(save_path, dpi = 400, bbox_inches='tight')   
    plt.close()
    print("save path to " + save_path)     
    
    
def drawSummaryOfMethod(all_data_map, xlable, ylable, title, is_percentage=False):
    map_and_agent_data = dict()
    fig=plt.figure(figsize=(5,4.5))

    width = 0.4 
    formater = ticker.ScalarFormatter(useMathText=True) 
    formater.set_scientific(True)
    formater.set_powerlimits((0,0))
    
    for method_top, method_value in all_data_map.items():
        value_lists_raw = list()
        value_lists_layered = list()
        for map_key, map_value in all_data_map[method_top].items():
            for method_key, method_value in all_data_map[method_top][map_key].items():
                value = list()           
                for agent_size_key in all_data_map[method_top][map_key][method_key].keys():
                    if ylable == "Makespan" or ylable == "Sum of cost":
                        if len(all_data_map[method_top][map_key][method_key][agent_size_key]) > 0:
                            # only considering success cases
                            success_cases = list()
                            for case in all_data_map[method_top][map_key][method_key][agent_size_key]:
                                success_cases.append(case)
                            value.append(np.mean(success_cases))
                        else:
                            value.append(0)
                    else:
                        if len(all_data_map[method_top][map_key][method_key][agent_size_key]) > 0:
                            value.append(np.mean(all_data_map[method_top][map_key][method_key][agent_size_key]))
                        else:
                            value.append(0)    
                head_split = method_key.split('_')
                if head_split[0] == 'LAYERED':  
                    value_lists_layered.append(np.mean(value))  
                if head_split[0] == 'RAW':
                    value_lists_raw.append(np.mean(value)) 
                    
        plt.xticks(rotation=70)    
        common_font_size = 12
        common_rotate_angle = 80
        if ylable == "Success rate":
            if len(value_lists_layered) > 0:
                p1 = plt.bar(drawing_method_set.index(method_top)+width/2, np.mean(value_lists_layered), width, hatch="//") 
                plt.bar_label(p1, label_type='edge', fmt="%.2g", rotation=common_rotate_angle, fontsize=common_font_size)#, fmt=formater) 
            if len(value_lists_raw) > 0:
                p2 = plt.bar(drawing_method_set.index(method_top)-width/2, np.mean(value_lists_raw), width)   
                plt.bar_label(p2, label_type='edge', fmt="%.2g", rotation=common_rotate_angle, fontsize=common_font_size)#, fmt=formater)   
        else:
            if len(value_lists_layered) > 0:
                p1 = plt.bar(drawing_method_set.index(method_top)+width/2, np.mean(value_lists_layered), width, hatch="//") 
                plt.bar_label(p1, label_type='edge', fmt="%.1e", rotation=common_rotate_angle, fontsize=common_font_size)#, fmt=formater) 
            if len(value_lists_raw) > 0:
                p2 = plt.bar(drawing_method_set.index(method_top)-width/2, np.mean(value_lists_raw), width)   
                plt.bar_label(p2, label_type='edge', fmt="%.1e", rotation=common_rotate_angle, fontsize=common_font_size)#, fmt=formater)    
        # break            
    
    plt.tick_params(axis='both', labelsize=14)

    ax = plt.gca()
    ax.yaxis.offsetText.set_fontsize(8)
    ax.yaxis.set_major_formatter(formater)     
    plt.xticks([0,1,2,3,4,5,6], drawing_method_set_2)
    
    y_range = plt.ylim()      
    if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage(MB)":
        ax.set_yscale('log')  
        plt.ylim(y_range[0]/10, y_range[1]*10)  
    else:    
        plt.ylim(0, y_range[1]*1.1)
    #plt.title(ylable)
    if is_percentage:
        plt.ylim(0, 1)
        
    #plt.legend(loc='best', fontsize = 16, ncol=1, handletextpad=.5, framealpha=0.5)
    plt.tight_layout()        
    save_path = '../test/pic/layered_MAPF/'+title
    
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
        
    save_path = save_path + "/" + 'method_summary'+'.png'
    plt.savefig(save_path, dpi = 200, bbox_inches='tight')   
    plt.close()
    print("save path to " + save_path)  
    
data_path_dir = '../test/test_data/layered_mapf/'

all_single_data = list()

name_of_decomposition = "DECOMPOSITION"

method_marker_map = {"RAW_EECBS":'p-',       "LAYERED_EECBS":'p--',
                  "RAW_LaCAM":'P-',       "LAYERED_LaCAM":'P--',
                  "RAW_PBS":'D-',         "LAYERED_PBS":'D--',
                  "RAW_LaCAM":'X-',      "LAYERED_LaCAM":'X--',
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
map_format_list = [
                 "empty-16-16", # 120
                 "empty-32-32", # 400
                 # 2
                 "maze-32-32-2", # 120
                 "maze-32-32-4", # 240
                 "maze-128-128-2", # 700
                 "maze-128-128-10", # 1000
                 # 6
                 "den312d", # 800
                 "den520d", # 900
                 # 8
                 "Berlin_1_256", # 900
                 "Paris_1_256", # 1000
                 # 10
                 "ht_chantry", # 1000
                 "lak303d", # 1000
                 # 12
                 "random-32-32-20",# 240
                 "random-64-64-20",  # 1000

                 # 14
                 "room-32-32-4", # 200
                 "room-64-64-8", # 700
                 "room-64-64-16", # 1000
                 # 17
                 "warehouse-10-20-10-2-1", # 800
                 "warehouse-10-20-10-2-2", # 1000
                 "warehouse-20-40-10-2-1", # 1000
                 "warehouse-20-40-10-2-2", # 1000
                 # 21
                 "Boston_0_256", # 1000
                 "lt_gallowstemplar_n", # 1000
                 "ost003d" # 1000

]

# 0~250: empty-16-16, maze-32-32-2, maze-32-32-4, random-32-32-20
# 0~500: empty-32-32, 
# 0~750: maze-128-128-2, room-64-64-8, warehouse-10-20-10-2-1
# 0~1000: maze-128-128-10, Paris_1_256, ht_chantry, lak303d, room-64-64-16, warehouse-10-20-10-2-1, warehouse-10-20-10-2-2, warehouse-20-40-10-2-1, warehouse-20-40-10-2-2, Boston_0_256, lt_gallowstemplar_n, ost003d


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
                 "random-64-64-20":'D',  # maximum 8s

                 "room-32-32-4":'X',
                 "room-64-64-8":'X',
                 "room-64-64-16":'X',

                 "warehouse-10-20-10-2-1":'+',
                 ####
                 "warehouse-10-20-10-2-2":'+',
                 "warehouse-20-40-10-2-1":'+',
                 "warehouse-20-40-10-2-2":'+',
                 
                 "Boston_0_256":'<',
                 "lt_gallowstemplar_n":'H',
                 "ost003d":'H'

}

# when draw how multiple maps in one figure, draw map that have similar agent size range in one figure 
map_format_map_index = {
                 "empty-16-16":1, # 120
                 "empty-32-32":2, # 400
                 # 2
                 "maze-32-32-2":1, # 120
                 "maze-32-32-4":1, # 240
                 "maze-128-128-2":2, # 700
                 "maze-128-128-10":3, # 1000
                 # 6
                 "den312d":2, # 800
                 "den520d":3, # 900
                 # 8
                 "Berlin_1_256":3, # 900
                 "Paris_1_256":3, # 1000
                 # 10
                 "ht_chantry":3, # 1000
                 "lak303d":3, # 1000
                 # 12
                 "random-32-32-20":1,# 240
                 "random-64-64-20":4,  # 1000

                 # 14
                 "room-32-32-4":1, # 200
                 "room-64-64-8":2, # 700
                 "room-64-64-16":4, # 1000
                 # 17
                 "warehouse-10-20-10-2-1":2, # 800
                 "warehouse-10-20-10-2-2":4, # 1000
                 "warehouse-20-40-10-2-1":4, # 1000
                 "warehouse-20-40-10-2-2":4, # 1000
                 # 21
                 "Boston_0_256":4, # 1000
                 "lt_gallowstemplar_n":4, # 1000
                 "ost003d":4 # 1000

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

drawing_method_set = [
                      "EECBS", # tested ok, advance in time cost, nothing in memory usage
                      "PBS",  # tested ok, advance in time cost, advance in memory usage, full map
                      "PushAndSwap", # some map not ok, advance in time cost, nothing in memory usage, full map
                      "LaCAM",# tested ok, drawback in time cost, drawback in memory usage, full map
                      "HCA", # test ok, advance in memory usage, drawback in time cost, full map
                      "PIBT2", # some map not ok，advance in memory usage, drawback in time cost, full map
                      "LNS", # test ok, advance in memory usage, advance in time cost
                     ]

drawing_method_set_2 = [
                      "EECBS", # tested ok, advance in time cost, nothing in memory usage
                      "PBS",  # tested ok, advance in time cost, advance in memory usage, full map
                      "PAS", # some map not ok, advance in time cost, nothing in memory usage, full map
                      "LaCAM",# tested ok, drawback in time cost, drawback in memory usage, full map
                      "HCA*", # test ok, advance in memory usage, drawback in time cost, full map
                      "PIBT+", # some map not ok，advance in memory usage, drawback in time cost, full map
                      "LNS2", # test ok, advance in memory usage, advance in time cost
                     ]
    
for single_data in all_single_data:
    #map_name = single.map_name
    for line_data in single_data.data_list:
             
        splited_method_name = line_data.method.split("_")
        method_name = splited_method_name[1]
        
        if not method_name in drawing_method_set:
            continue
               
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
        all_method_time_cost_map[method_name][single_data.map_name][name_of_decomposition][line_data.agent_count].append(line_data.subgraph_init_time_cost)
        all_method_success_rate_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.success)    
        all_method_memory_usage_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.max_memory_usage)

        
        if line_data.success == 1:
            all_method_total_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.total_cost)
            all_method_makespan_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.max_single_cost)

for method_key, method_value in all_method_time_cost_map.items(): 
    # draw at each agent size
    # drawMethodMaps(all_method_time_cost_map[method_key], "Number of agents", "Time cost(ms)", "time_cost/"+method_key)           
    # drawMethodMaps(all_method_memory_usage_map[method_key], "Number of agents", "Memory usage(MB)", "memory_usage/"+method_key)           
    # drawMethodMaps(all_method_total_cost_map[method_key], "Number of agents", "Sum of cost", "sum_of_cost/"+method_key)           
    # drawMethodMaps(all_method_makespan_map[method_key], "Number of agents", "Makespan", "makespan/"+method_key)           
    # drawMethodMaps(all_method_success_rate_map[method_key], "Number of agents", "Success rate", "success_rate/"+method_key)        
    drawMethodMapAgentSizes(all_method_time_cost_map[method_key], "Number of agents", "Time cost(ms)", "time_cost/"+method_key)
    
    # draw summary of maps
    drawSummaryOfMap(all_method_time_cost_map[method_key], "Number of agents", "Time cost(ms)", "time_cost/"+method_key)    
    drawSummaryOfMap(all_method_memory_usage_map[method_key], "Number of agents", "Memory usage(MB)", "memory_usage/"+method_key)           
    drawSummaryOfMap(all_method_total_cost_map[method_key], "Number of agents", "Sum of cost", "sum_of_cost/"+method_key)           
    drawSummaryOfMap(all_method_makespan_map[method_key], "Number of agents", "Makespan", "makespan/"+method_key)           
    drawSummaryOfMap(all_method_success_rate_map[method_key], "Number of agents", "Success rate", "success_rate/"+method_key)      
    
#draw summary of methods
drawSummaryOfMethod(all_method_time_cost_map, "Number of agents", "Time cost(ms)", "time_cost")           
drawSummaryOfMethod(all_method_memory_usage_map, "Number of agents", "Memory usage(MB)", "memory_usage")           
drawSummaryOfMethod(all_method_total_cost_map, "Number of agents", "Sum of cost", "sum_of_cost")           
drawSummaryOfMethod(all_method_makespan_map, "Number of agents", "Makespan", "makespan")           
drawSummaryOfMethod(all_method_success_rate_map, "Number of agents", "Success rate", "success_rate")      

