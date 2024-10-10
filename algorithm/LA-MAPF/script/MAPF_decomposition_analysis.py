import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker

#import seaborn as sns

def loadDataFromfile(file_path):
    data_list = list()
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()#[1:]
            for line in lines:
                #print(line)
                #print(line.strip())
                splited_line = line.split()
                
                
                new_data = LineData()
                
                new_data.time_cost         = float(splited_line[0])
                new_data.max_cluster       = float(splited_line[1])
                new_data.total_size        = float(splited_line[2])
                new_data.success           = float(splited_line[3])
                new_data.level             = int  (splited_line[4])
                new_data.memory_usage      = float(splited_line[5])
                new_data.num_of_subproblem = float(splited_line[6])
                
                new_data.init_time_cost    = float(splited_line[7])
                new_data.level_1_time_cost = float(splited_line[8])
                new_data.level_2_time_cost = float(splited_line[9])
                new_data.level_3_time_cost = float(splited_line[10])
                new_data.level_4_time_cost = float(splited_line[11])

                data_list.append(new_data)
    except Exception as e:            
        print(e)       
    return data_list

class LineData:
    time_cost         = 0.
    max_cluster       = 0.
    total_size        = 0.
    success           = 1.
    level             = 0
    memory_usage      = 0.
    num_of_subproblem = 0
    
    init_time_cost    = 0.
    level_1_time_cost = 0.
    level_2_time_cost = 0.
    level_3_time_cost = 0.
    level_4_time_cost = 0.    

# all data in a txt file
class SingleTestData:
    data_list = list()
    map_name = ''
    
    
def drawMethodMap(single_map_data, value_type):
    fig = plt.figure(figsize=(5,3.5)) 
    map_name = single_map_data.map_name
    
    all_raw_data = [dict(), dict(), dict(), dict()]
    all_init_time_cost = dict() # only use when draw time cost of initialize cconnectivity graph

    for data in single_map_data.data_list:
        total_size        = data.total_size
        time_cost         = data.time_cost
        success           = data.success
        max_cluster       = data.max_cluster
        memory_usage      = data.memory_usage
        num_of_subproblem = data.num_of_subproblem
        
        if all_raw_data[0].get(total_size) == None:
            all_raw_data[0][total_size] = list()
            all_raw_data[1][total_size] = list()
            all_raw_data[2][total_size] = list()            
            all_raw_data[3][total_size] = list()            

        assert(data.level>= 1 and data.level <= 4)
        
        if value_type == "decomposition_rate":
            all_raw_data[data.level-1][total_size].append(max_cluster / total_size)    
        elif value_type == "time_cost":
            all_raw_data[data.level-1][total_size].append(time_cost)    
            if all_init_time_cost.get(total_size)  == None:
                all_init_time_cost[total_size] = list()   
            all_init_time_cost[total_size].append(data.init_time_cost)
        elif value_type == "memory_usage":
            all_raw_data[data.level-1][total_size].append(memory_usage)        
        elif value_type == "num_of_subproblem":
            all_raw_data[data.level-1][total_size].append(num_of_subproblem)     
    label_buffer = "1"        
    for i in range(0,4):    
        
        dict_x_y = dict()
        dict_x_error = dict()

        if value_type == "decomposition_rate":
            for data_key, data_val in all_raw_data[i].items():
                dict_x_y[data_key] = np.mean(data_val)
                dict_x_error[data_key] = np.std(data_val)
        else:
            for data_key, data_val in all_raw_data[i].items():
                dict_x_y[data_key] = np.mean(data_val)
                dict_x_error[data_key] = np.std(data_val)

        # yerr=std_val
        sorted_x = sorted (dict_x_y)
        sorted_y = list()
        sorted_error = list()
        for temp_x in sorted_x:
            sorted_y.append(dict_x_y[temp_x])
            sorted_error.append(dict_x_error[temp_x])
        plt.errorbar(sorted_x, sorted_y, label=label_buffer, markersize=14, fmt=step_fmt[i], linewidth= 4, elinewidth=4, capsize=4)
        label_buffer = label_buffer + ", " + str(i+2)          

    # add time cost about construct connectivity graph
    if value_type == "time_cost":
        #print(all_init_time_cost)
        for data_key, data_val in all_init_time_cost.items():
            dict_x_y[data_key] = np.mean(data_val)
            dict_x_error[data_key] = np.std(data_val)
        sorted_x = sorted (dict_x_y)
        sorted_y = list()
        sorted_error = list()
        for temp_x in sorted_x:
            sorted_y.append(dict_x_y[temp_x])
            sorted_error.append(dict_x_error[temp_x])
        plt.errorbar(sorted_x, sorted_y, label="init_cost", markersize=14, fmt=step_fmt[4], linewidth= 4, elinewidth=4, capsize=4)
        
    plt.legend(loc='best')    
    plt.tick_params(axis='both', labelsize=18)
    #plt.ticklabel_format(style='sci', scilimits=(0,0), axis='y')
    
    formater = ticker.ScalarFormatter(useMathText=True) 
    formater.set_scientific(True)
    formater.set_powerlimits((0,0))

    ax = plt.gca()
    if value_type == "time_cost":
        ax.set_yscale('log')  
    ax.yaxis.offsetText.set_fontsize(18)
    ax.yaxis.set_major_formatter(formater)
    y_range = plt.ylim()
    plt.ylim(0, y_range[1])
    if value_type == "decomposition_rate":
        plt.ylim(0, 1)

    plt.legend(loc='best', fontsize = 16, ncol=1, handletextpad=.5, framealpha=0.5)
    # plt.title(title_dict[value_type])

    #plt.grid()
    plt.tight_layout()
    save_path = '../../../test/test_data/large_agent_instance/decomposition/'+value_type+'/'+map_name+"-"+value_type+'.png'
    plt.savefig(save_path, dpi = 200, bbox_inches='tight')   
    plt.close()
    print("save picture to "+save_path)

    
step_fmt = ["x-","X-.","*--","2--","r-"] # ["x-","o-.","^--","s--"]       
title_dict = {"time_cost":"Time cost(ms)",
              "decomposition_rate":"Decomposition rate",
              "memory_usage":"Memory usage(MB)",
              "num_of_subproblem":"Number of subproblem"
              }
    
data_path_dir = '../../../test/test_data/large_agent_instance/'
all_map_name = [
                "empty-48-48",
                
                # "maze-32-32-2",
                # "maze-32-32-4",
                "maze-128-128-10",
                # "maze-128-128-2",
                
                # "den312d",
                "den520d",
                
                "Berlin_1_256",
                "Paris_1_256",
                
                # "ht_chantry",
                # "lak303d",
                
                # "random-64-64-10",
                # "random-64-64-20",
                # "random-32-32-20",
                
                # "room-64-64-16",
                # "room-64-64-8",
                # "room-32-32-4",
                
                # "warehouse-10-20-10-2-1",
                # "warehouse-10-20-10-2-2",
                # "warehouse-20-40-10-2-1",
                # "warehouse-20-40-10-2-2",
                
                # "Boston_0_256",
                # "lt_gallowstemplar_n",
                "ost003d"
                ]

all_single_data = list()

for map_name in all_map_name:
    data_file_path = data_path_dir + map_name + '_la_dec.txt'
    print('load data from', data_file_path)
    single = SingleTestData()
    single.map_name = map_name
    single.data_list = loadDataFromfile(data_file_path)
    if len(single.data_list) == 0:
        continue
    all_single_data.append(single)
    
for single_map_data in all_single_data:
    drawMethodMap(single_map_data, "time_cost")
    
for single_map_data in all_single_data:
    drawMethodMap(single_map_data, "decomposition_rate")
    
for single_map_data in all_single_data:
    drawMethodMap(single_map_data, "memory_usage")    
    
for single_map_data in all_single_data:
    drawMethodMap(single_map_data, "num_of_subproblem")        
    
#plt.show()    
