
import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker
import os

from matplotlib.ticker import MaxNLocator

def loadDataFromfile(file_path, map_name):
    data_list = list()
    #print("file=",file_path)
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()
            #print("lines count=", len(lines))
            for line in lines:
                #print("line=", line)
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
                
                # if(new_data.agent_count > map_size_limit[map_name]):
                #     continue
                
                if np.isnan(new_data.total_cost):
                    continue
                if np.isnan(new_data.max_single_cost):
                    continue
                
                #if new_data.time_cost > 35:
                    #new_data.success = 0
                    #continue
                if new_data.time_cost > 60:
                   new_data.time_cost = 60
                    #new_data.success = 0
                    
                # if new_data.agent_count >= 10 and new_data.time_cost <= 2: 
                #     continue
                
                # if head_split[0] == 'ID':
                #     continue

                if head_split[0] == 'BL' or head_split[0] == "BP" or head_split[0] == "BLINIT":
                    new_data.get_subgraph_time_cost = float(splited_line[7])
                    new_data.decomposition_time_cost = float(splited_line[8])
                    if new_data.success:
                        new_data.max_subproblem_size = int(splited_line[9])
                        new_data.num_of_subproblem = int(splited_line[10])
                    else:
                        new_data.max_subproblem_size = new_data.agent_count
                        new_data.num_of_subproblem = 1
                    new_data.loss_of_solvability = float(splited_line[11])
                        
                if head_split[0] == 'ID' :
                    if new_data.success:
                        new_data.max_subproblem_size = int(splited_line[7])
                        new_data.num_of_subproblem =  int(splited_line[8])
                    else:
                        new_data.max_subproblem_size = new_data.agent_count
                        new_data.num_of_subproblem = 1
                
                if head_split[0] == 'RAW' :
                    new_data.max_subproblem_size = new_data.agent_count
                    new_data.num_of_subproblem = 1
                data_list.append(new_data)
                #print("data=", new_data.method, ' ', new_data.agent_count, ' ', new_data.max_subproblem_size, ' ', new_data.num_of_subproblem)

                # if head_split[1] == 'LaCAM' :
                #     print("data=", new_data.method, ' ', new_data.agent_count, ' ', new_data.max_subproblem_size, ' ')

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
    get_subgraph_time_cost = 0.
    decomposition_time_cost = 0.
    max_subproblem_size = 0
    num_of_subproblem = 0
    loss_of_solvability = 0.    

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
            # print("splited_method_name[0]=", splited_method_name[0])
            # print("splited_method_name[1]=", splited_method_name[1])

            if ylable == "max_subproblem_size" or ylable == "num_of_subproblem":
                if splited_method_name[0] == "RAW":
                    continue


            if ylable == "loss_of_solvability":
                if splited_method_name[0] != "BP" and splited_method_name[0] != "BL" and splited_method_name[0] != "BLINIT":
                    continue

            if splited_method_name[0] == "ID" and splited_method_name[1] == "LaCAM":
                continue

            for agent_size_key in sorted_keys:
                x.append(agent_size_key)
                if len(all_data_map[map_key][method_key][agent_size_key]) > 0:
                    y.append(np.mean(all_data_map[map_key][method_key][agent_size_key]))
                else:
                    break 
    
            if len(splited_method_name) == 2:    
                if len(x) != 0 and len(y) != 0:
                    while len(x) > len(y):
                        x.pop()
                    plt.errorbar(x, y, fmt=method_marker_map2[splited_method_name[0]], markersize=14, markerfacecolor='none', label=lable_map[splited_method_name[0]], linewidth=2, elinewidth=4, capsize=4) 

        plt.tick_params(axis='both', labelsize=18)
        formater = ticker.ScalarFormatter(useMathText=True) 
        formater.set_scientific(True)
        formater.set_powerlimits((0,0))
        ax = plt.gca()
        ax.yaxis.offsetText.set_fontsize(18)
        ax.yaxis.set_major_formatter(formater) 
        
        plt.xlabel(xlable, fontsize=18) # 横坐标标题
        plt.legend(loc='best', fontsize=18) # 图例位置设置

        y_range = plt.ylim()      
        plt.ylim(0, y_range[1]*1.1)
    
        if is_percentage:
            plt.ylim(0, 1)
                
        plt.tight_layout()
        save_path = data_path_dir+title
        
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            print("Folder: " + save_path + " created")
            
        save_path = save_path + "/" + map_key
        plt.savefig(save_path, dpi = 100, bbox_inches='tight')   
        plt.close()
        print("save path to " + save_path)

def drawSummaryOfMap(all_data_map, xlable, ylable, title, is_percentage=False):
    map_and_agent_data = dict()
    fig, ax =plt.subplots(figsize=(5,3.5)) #添加绘图框 plt.figure(figsize=(5,3.5)
    map_lists = list()
    value_lists_id = list()
    value_lists_raw = list()
    value_lists_bp = list()
    value_lists_bl = list()
    value_lists_bl_init = list()
    
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
              
            if head_split[0] == 'RAW':  
                value_lists_raw.append(np.mean(value))
                
            if head_split[0] == 'ID':  
                value_lists_id.append(np.mean(value))
                
            if head_split[0] == 'BP':
                value_lists_bp.append(np.mean(value))

            if head_split[0] == 'BL':
                value_lists_bl.append(np.mean(value))

            if head_split[0] == 'BLINIT':
                value_lists_bl_init.append(np.mean(value))    

    if ylable == "loss_of_solvability":
        width = 0.4                   
        x1 = np.arange(len(value_lists_bp))
        x2 = np.arange(len(value_lists_bl))
        x3 = np.arange(len(value_lists_bl_init))

        ax.bar(x1+1-width, value_lists_bp,      width, label="BP", hatch="//")    
        ax.bar(x2+1,       value_lists_bl,      width, label="BL", hatch="-")   
        ax.bar(x3+1+width, value_lists_bl_init, width, label="BI", hatch="o")   

    elif ylable == "max_subproblem_size" or ylable == "num_of_subproblem":
        width = 0.2                   
        x1 = np.arange(len(value_lists_id))
        x2 = np.arange(len(value_lists_bp))
        x3 = np.arange(len(value_lists_bl))
        x4 = np.arange(len(value_lists_bl_init))

        ax.bar(x1+1-3*width/2, value_lists_id,      width, label="ID", hatch="-")    
        ax.bar(x2+1-width/2,   value_lists_bp,      width, label="BP", hatch="//")    
        ax.bar(x3+1+width/2,   value_lists_bl,      width, label="BL", hatch="-")    
        ax.bar(x4+1+3*width/2, value_lists_bl_init, width, label="BI", hatch="o")    

    elif len(head_split) == 2 and head_split[1] == "CBS":
        width = 0.2 
        x1 = np.arange(len(value_lists_raw))
        x2 = np.arange(len(value_lists_id))
        x3 = np.arange(len(value_lists_bp))
        x4 = np.arange(len(value_lists_bl))
        x5 = np.arange(len(value_lists_bl_init))

        ax.bar(x1+1-2*width, value_lists_raw,       width, label="RAW")    
        ax.bar(x2+1-width,   value_lists_id,        width, label="ID", hatch="-")    
        ax.bar(x3+1,         value_lists_bp,        width, label="BP", hatch="//")  
        ax.bar(x4+1+width,   value_lists_bl,        width, label="BL", hatch="//")  
        ax.bar(x5+1+2*width, value_lists_bl_init,   width, label="BI", hatch="o")  

    else: 
        width = 0.2 
        x1 = np.arange(len(value_lists_raw))
        x2 = np.arange(len(value_lists_bp))
        x3 = np.arange(len(value_lists_bl))
        x4 = np.arange(len(value_lists_bl_init))

        ax.bar(x1+1-3*width/2, value_lists_raw,       width, label="RAW")    
        ax.bar(x2+1-width/2,   value_lists_bp,        width, label="BP", hatch="//")  
        ax.bar(x3+1+width/2,   value_lists_bl,        width, label="BL", hatch="//")  
        ax.bar(x4+1+3*width/2, value_lists_bl_init,   width, label="BI", hatch="o")  
                        
    plt.xticks(rotation=70)         
    plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))  # 整数刻度
    print(title + "/" + ylable + " / raw = " + str(np.mean(value_lists_raw)) + " bp = " + str(np.mean(value_lists_bp)) + " id = " + str(np.mean(value_lists_id)) + " bl = " + str(np.mean(value_lists_bl)) )
    
    plt.tick_params(axis='both', labelsize=14)
    #plt.xticks(np.arange(1, len(map_format_list)+1, 1))
    formater = ticker.ScalarFormatter(useMathText=True) 
    formater.set_scientific(True)
    formater.set_powerlimits((0,0))
    ax = plt.gca()
    ax.yaxis.offsetText.set_fontsize(18)
    ax.yaxis.set_major_formatter(formater)      
    
    y_range = plt.ylim()      
    # if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage(MB)":
    #     ax.set_yscale('log')  
    #     plt.ylim(1, y_range[1]*10)  
    # else:    
    #     plt.ylim(0, y_range[1])
    plt.title(ylable)
    if is_percentage:
        plt.ylim(0, 1)
        
    plt.xlabel(xlable, fontsize=14) # 横坐标标题
    #plt.legend(loc='best', fontsize = 16, ncol=1, handletextpad=.5, framealpha=0.5)
    ax.legend(loc='best', fontsize=12) # 图例位置设置
    plt.tight_layout()        
    save_path = data_path_dir+title
    
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
        
    save_path = save_path + "/" + 'summary'
    plt.savefig(save_path, dpi = 100, bbox_inches='tight')   
    plt.close()
    print("save path to " + save_path)     
    
def compareOnlySuccess(all_data_map, ylable, method_1, method_2):
    map_lists = list()
    
    value_list_1 = list()
    value_list_2 = list()
    
    for map_key, map_value in all_data_map.items():
        map_lists.append(map_key)
        for method_key, method_value in all_data_map[map_key].items():
            value = list()           
            for agent_size_key in all_data_map[map_key][method_key].keys():
                if len(all_data_map[map_key][method_key][agent_size_key]) > 0:
                    # print("method_key = ", method_key)
                    splitted_key = method_key.split("_")

                    new_key_1 = method_1 + "_" + splitted_key[1]
                    new_key_2 = method_2 + "_" + splitted_key[1]
                    
                    # print("new 1 = ", new_key_1)
                    # print("new 2 = ", new_key_2)                   
                    
                    if all_data_map[map_key].get(new_key_1) == None:
                        continue
                    if all_data_map[map_key].get(new_key_2) == None:
                        continue
                    
                    # print("len 1 = ", len(all_data_map[map_key][new_key_1][agent_size_key]), "len 2 =", len(all_data_map[map_key][new_key_2][agent_size_key]))             
                    smaller_len = min(len(all_data_map[map_key][new_key_1][agent_size_key]), len(all_data_map[map_key][new_key_2][agent_size_key]))       
                    # assert(len(all_data_map[map_key][new_key_1][agent_size_key]) == len(all_data_map[map_key][new_key_2][agent_size_key]))
                    
                    for i in range(smaller_len):
                        val_1 = all_data_map[map_key][new_key_1][agent_size_key][i]
                        val_2 = all_data_map[map_key][new_key_2][agent_size_key][i] 
                        # only considering both success cases
                        if val_1 != 0 and val_2 != 0:
                            value_list_1.append(val_1)
                            value_list_2.append(val_2)
    
    print(ylable + ": " + new_key_1 + " = " + str(np.mean(value_list_1)) + " / " + new_key_2 + " = " + str(np.mean(value_list_2)))
    
    
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
    # if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage(MB)":
    #     ax.set_yscale('log')  
    #     plt.ylim(y_range[0]/10, y_range[1]*10)  
    # else:    
    #     plt.ylim(0, y_range[1]*1.1)
    #plt.title(ylable)
    if is_percentage:
        plt.ylim(0, 1)
        
    #plt.legend(loc='best', fontsize = 16, ncol=1, handletextpad=.5, framealpha=0.5)
    plt.tight_layout()        
    save_path = data_path_dir+title
    
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
        
    save_path = save_path + "/" + 'method_summary'+'.png'
    plt.savefig(save_path, dpi = 100, bbox_inches='tight')   
    plt.close()
    print("save path to " + save_path)  
    
data_path_dir = '../test/test_data/layered_mapf/'

all_single_data = list()

name_of_get_subgraph = "GETSUBGRAPH"
name_of_decomposition = "DECOMPOSITION"

# method_marker_map = {"RAW_CBS":'p-',       "LAYERED_CBS":'p--',
#                   "RAW_LaCAM":'P-',       "LAYERED_LaCAM":'P--',
#                   "RAW_PBS":'D-',         "LAYERED_PBS":'D--',
#                   "RAW_LaCAM":'X-',      "LAYERED_LaCAM":'X--',
#                   "RAW_LNS":'+-',         "LAYERED_LNS":'+--',
#                   "RAW_AnytimeBCBS":'*-', "LAYERED_AnytimeBCBS":'*--',
#                   "RAW_AnytimeEECBS":'o-', "LAYERED_AnytimeEECBS":'o--',
#                   "RAW_CBSH2_RTC":'v-',   "LAYERED_CBSH2_RTC":'v--',
#                   "RAW_PIBT":'^-',        "LAYERED_PIBT":'^--',
#                   "RAW_PIBT2":'<-',       "LAYERED_PIBT2":'<--',
#                   "RAW_HCA":'>-',         "LAYERED_HCA":'>--',
#                   "RAW_PushAndSwap":'H-', "LAYERED_PushAndSwap":'H--',
#                   name_of_get_subgraph:"D-."
#                   }

method_marker_map2 = {
    "RAW":'o-',
    "BP":'*-',
    "ID":'v-',
    "BL":"x-",
    "BLINIT":"s-",
    name_of_get_subgraph:"o-.",
    name_of_decomposition:"*-."
}

lable_map = {
    "RAW":"RAW",
    "BP":"BP",
    "ID":"ID",
    "BL":"BL",
    "BLINIT":"BI",
}

map_format_map = {
                #  "empty-16-16":'o', # 1
                  "empty-32-32":'o', # 2
                 
                  "maze-32-32-2":'*', # 1
                  "maze-32-32-4":'p', # 1 
                #  "maze-128-128-2":'*', # 2
                #  "maze-128-128-10":'o', # 3
                 
                #  "den312d":'p', # 2
                #  "den520d":'*', # 3
                 
                  "Berlin_1_256":'p', # 3
                #  "Paris_1_256":'s', # 3
                 
                  "ht_chantry":'H', # 3
                  "lak303d":'X', # 3

                  "random-32-32-20":'s', # 1
                #  "random-64-64-20":'o',  # 4

                  "room-32-32-4":'H', # 1
                #  "room-64-64-8":'s', # 2
                #  "room-64-64-16":'*', # 4

                  "warehouse-10-20-10-2-1":'H', # 2
                #  ####
                  "warehouse-10-20-10-2-2":'p', # 4
                  "warehouse-20-40-10-2-1":'s', # 4
                  "warehouse-20-40-10-2-2":'H', # 4
                 
                 "Boston_0_256":'X', # 4 
                  "lt_gallowstemplar_n":'P', # 4
                  "ost003d":'<' # 4

}





# 1, load all data
for map_name_key, map_format_value in map_format_map.items():
    data_file_path = data_path_dir + map_name_key + '_comp.txt'
    print('load data from', data_file_path)
    single = SingleTestData() # 不带括号则均指向同一元素
    single.map_name = map_name_key
    single.data_list = loadDataFromfile(data_file_path, map_name_key)
    all_single_data.append(single)
    

all_method_time_cost_map = dict()
all_method_total_cost_map = dict()
all_method_makespan_map = dict()
all_method_success_rate_map = dict()
all_method_memory_usage_map = dict()
all_method_cluster_cost_map = dict()
all_method_level_sort_map = dict()    

all_method_max_subproblem_map = dict()
all_method_num_of_subproblem_map = dict()    
all_method_loss_of_solvability_map = dict()    

drawing_method_set = [
                      "CBS", # tested ok, advance in time cost, nothing in memory usage
                      "PBS",  # tested ok, advance in time cost, advance in memory usage, full map
                      "PushAndSwap", # some map not ok, advance in time cost, nothing in memory usage, full map
                      "LaCAM",# tested ok, drawback in time cost, drawback in memory usage, full map
                      "HCA", # test ok, advance in memory usage, drawback in time cost, full map
                      "PIBT2", # some map not ok，advance in memory usage, drawback in time cost, full map
                      "LNS", # test ok, advance in memory usage, advance in time cost
                     ]

drawing_method_set_2 = [
                      "CBS", # tested ok, advance in time cost, nothing in memory usage
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
                   
        if all_method_max_subproblem_map.get(method_name) == None:    
            all_method_max_subproblem_map[method_name] = dict()      
            
        if all_method_num_of_subproblem_map.get(method_name) == None:    
            all_method_num_of_subproblem_map[method_name] = dict()   

        if all_method_loss_of_solvability_map.get(method_name) == None:    
            all_method_loss_of_solvability_map[method_name] = dict()                  
                   
        if all_method_time_cost_map[method_name].get(single_data.map_name) == None:
            all_method_time_cost_map[method_name][single_data.map_name] = dict()
            all_method_total_cost_map[method_name][single_data.map_name] = dict()
            all_method_makespan_map[method_name][single_data.map_name] = dict()
            all_method_memory_usage_map[method_name][single_data.map_name] = dict()
            all_method_success_rate_map[method_name][single_data.map_name] = dict()
            all_method_max_subproblem_map[method_name][single_data.map_name] = dict()
            all_method_num_of_subproblem_map[method_name][single_data.map_name] = dict()
            all_method_loss_of_solvability_map[method_name][single_data.map_name] = dict()

        if all_method_time_cost_map[method_name][single_data.map_name].get(line_data.method) == None:
            all_method_time_cost_map[method_name][single_data.map_name][line_data.method] = dict()
            #all_method_time_cost_map[method_name][single_data.map_name][name_of_get_subgraph] = dict()
            #all_method_time_cost_map[method_name][single_data.map_name][name_of_decomposition] = dict()
            all_method_total_cost_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_makespan_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_memory_usage_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_success_rate_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_max_subproblem_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_num_of_subproblem_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_loss_of_solvability_map[method_name][single_data.map_name][line_data.method] = dict()

            
        if all_method_time_cost_map[method_name][single_data.map_name][line_data.method].get(line_data.agent_count) == None:
            all_method_time_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()
            #all_method_time_cost_map[method_name][single_data.map_name][name_of_get_subgraph][line_data.agent_count] = list()
            #all_method_time_cost_map[method_name][single_data.map_name][name_of_decomposition][line_data.agent_count] = list()
            all_method_total_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()
            all_method_makespan_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()    
            all_method_memory_usage_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()    
            all_method_success_rate_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list() 
            all_method_max_subproblem_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list() 
            all_method_num_of_subproblem_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list() 
            all_method_loss_of_solvability_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list() 

            
        all_method_time_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.time_cost)
        # all_method_time_cost_map[method_name][single_data.map_name][name_of_get_subgraph][line_data.agent_count].append(line_data.get_subgraph_time_cost)
        # all_method_time_cost_map[method_name][single_data.map_name][name_of_decomposition][line_data.agent_count].append(line_data.decomposition_time_cost/1e3)
        all_method_success_rate_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.success)    
        all_method_memory_usage_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.max_memory_usage)

        splited_method_name = line_data.method.split('_')

        # if (splited_method_name[0] == "ID" or splited_method_name[0] == "BP" or splited_method_name[0] == "BL") and splited_method_name[1] != "LaCAM":
        all_method_max_subproblem_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.max_subproblem_size)
        all_method_num_of_subproblem_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.num_of_subproblem) 
        all_method_loss_of_solvability_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.loss_of_solvability) 

        if line_data.success == 1:
            all_method_total_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.total_cost)
            all_method_makespan_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.max_single_cost)

for method_key, method_value in all_method_time_cost_map.items(): 
    # draw at each agent size
    drawMethodMaps(all_method_time_cost_map[method_key], "Number of agents", "Time cost(s)", "time_cost/"+method_key)           
    drawMethodMaps(all_method_memory_usage_map[method_key], "Number of agents", "Memory usage(MB)", "memory_usage/"+method_key)           
    drawMethodMaps(all_method_total_cost_map[method_key], "Number of agents", "Sum of cost", "sum_of_cost/"+method_key)           
    drawMethodMaps(all_method_makespan_map[method_key], "Number of agents", "Makespan", "makespan/"+method_key)           
    drawMethodMaps(all_method_success_rate_map[method_key], "Number of agents", "Success rate", "success_rate/"+method_key)        
    
    drawMethodMaps(all_method_max_subproblem_map[method_key], "Number of agents", "max_subproblem_size", "max_subproblem_size/"+method_key)        
    drawMethodMaps(all_method_num_of_subproblem_map[method_key], "Number of agents", "num_of_subproblem", "num_of_subproblem/"+method_key)        
    drawMethodMaps(all_method_loss_of_solvability_map[method_key], "Loss of solvability(%)", "loss_of_solvability", "loss_of_solvability/"+method_key)        

    # # draw summary of maps
    drawSummaryOfMap(all_method_time_cost_map[method_key], "Map index", "Time cost(s)", "time_cost/"+method_key)    
    drawSummaryOfMap(all_method_memory_usage_map[method_key], "Map index", "Memory usage(MB)", "memory_usage/"+method_key)           
    # drawSummaryOfMap(all_method_total_cost_map[method_key], "Map index", "Sum of cost", "sum_of_cost/"+method_key)           
    # drawSummaryOfMap(all_method_makespan_map[method_key], "Map index", "Makespan", "makespan/"+method_key)           
    drawSummaryOfMap(all_method_success_rate_map[method_key], "Map index", "Success rate", "success_rate/"+method_key)      
    
    drawSummaryOfMap(all_method_max_subproblem_map[method_key], "Map index", "Max subproblem", "max_subproblem_size/"+method_key)           
    drawSummaryOfMap(all_method_num_of_subproblem_map[method_key], "Map index", "Number of subproblems", "num_of_subproblem/"+method_key)
    
    # compute makespan and soc comparison when both success      
    compareOnlySuccess(all_method_makespan_map[method_key], "Makespan", "RAW", "ID")
    compareOnlySuccess(all_method_makespan_map[method_key], "Makespan", "RAW", "BP")
    compareOnlySuccess(all_method_makespan_map[method_key], "Makespan", "RAW", "BL")
    compareOnlySuccess(all_method_makespan_map[method_key], "Makespan", "ID", "BP")
    compareOnlySuccess(all_method_makespan_map[method_key], "Makespan", "ID", "BL")
    compareOnlySuccess(all_method_makespan_map[method_key], "Makespan", "BP", "BL")

    compareOnlySuccess(all_method_total_cost_map[method_key], "Sum of cost", "RAW", "ID")
    compareOnlySuccess(all_method_total_cost_map[method_key], "Sum of cost", "RAW", "BP")
    compareOnlySuccess(all_method_total_cost_map[method_key], "Sum of cost", "RAW", "BL")
    compareOnlySuccess(all_method_total_cost_map[method_key], "Sum of cost", "ID", "BP")
    compareOnlySuccess(all_method_total_cost_map[method_key], "Sum of cost", "ID", "BL")
    compareOnlySuccess(all_method_total_cost_map[method_key], "Sum of cost", "BP", "BL")

#draw summary of methods
# drawSummaryOfMethod(all_method_time_cost_map, "Number of agents", "Time cost(ms)", "time_cost")           
# drawSummaryOfMethod(all_method_memory_usage_map, "Number of agents", "Memory usage(MB)", "memory_usage")           
# drawSummaryOfMethod(all_method_total_cost_map, "Number of agents", "Sum of cost", "sum_of_cost")           
# drawSummaryOfMethod(all_method_makespan_map, "Number of agents", "Makespan", "makespan")           
# drawSummaryOfMethod(all_method_success_rate_map, "Number of agents", "Success rate", "success_rate")      
# drawSummaryOfMethod(all_method_max_subproblem_map, "Number of agents", "Max subproblem", "max_subproblem_size")        
# drawSummaryOfMethod(all_method_num_of_subproblem_map, "Number of agents", "Number of subproblems", "num_of_subproblem")        

def removeMethodDataFromFile(file_path, temp_method_name):
    filtered_lines = list()
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()
            for line in lines:
                splited_line = line.split()
                if splited_line[0] != temp_method_name:
                    filtered_lines.append(line)
        f.close()
            #print(new_data.method, ' ', new_data.path_count, ' ', new_data.real_path_count, ' ', new_data.time_cost)
    except Exception as e:            
        print(e)  
        
    try:
        with open(file_path, 'w') as f:
            f.writelines(filtered_lines)    
        f.close()    
    except Exception as e:            
        print(e)             
    
def removeMethodDataFromFiles(map_format_map_index_local, method_name_local):
    for map_name_key, map_format_value in map_format_map_index_local.items():
        data_file_path = data_path_dir + map_name_key + '_comp.txt'
        print('remove data of ', method_name_local, ' from', data_file_path)
        removeMethodDataFromFile(data_file_path, method_name_local)

