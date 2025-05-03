import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker
import os
from PIL import Image
import matplotlib.image as mpimg
import math

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
                new_data.success = float(splited_line[5])
                new_data.max_memory_usage = float(splited_line[6])
                
                if np.isnan(new_data.total_cost):
                    continue
                if np.isnan(new_data.max_single_cost):
                    continue
                
                if new_data.time_cost > 31000:
                    new_data.success = 0
                
                if new_data.time_cost > 30000:
                    new_data.time_cost = 30000

                if len(splited_line) >= 9 and (head_split[0] == 'LAYERED' or 'ID'):
                    new_data.max_sub_problem_size = float(splited_line[7])
                    new_data.total_number_of_subproblem = float(splited_line[8])
                    # if not new_data.success:
                    #     new_data.max_sub_problem_size = float(splited_line[7])
                    #     new_data.total_number_of_subproblem = float(splited_line[8])
                
                if pre_fix != '':
                    if (head_split[0] == 'LAYERED' or 'ID') and new_data.success == 0:
                        new_data.max_sub_problem_size = new_data.agent_count 
                        new_data.total_number_of_subproblem = 1
                    elif head_split[0] == 'RAW':
                        new_data.max_sub_problem_size = new_data.agent_count 
                        new_data.total_number_of_subproblem = 1
                    if head_split[0] == 'RAW':
                        continue
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
    
    # used in compare layered mapf and independence detection
    max_sub_problem_size = 0
    total_number_of_subproblem = 0
        

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
                plt.errorbar(x, y, fmt=map_format_map[map_key]+method_marker_map2[name_of_decomposition], markersize=10, label=map_key+"/"+name_of_decomposition, linewidth=2, elinewidth=4, capsize=4)
            else:
                plt.errorbar(x, y, fmt=map_format_map[map_key]+method_marker_map2[splited_method_name[0]], markersize=10, label=map_key+"/"+splited_method_name[0], linewidth=2, elinewidth=4, capsize=4)
                
        plt.tick_params(axis='both', labelsize=18)
        formater = ticker.ScalarFormatter(useMathText=True) 
        formater.set_scientific(True)
        formater.set_powerlimits((0,0))
        ax = plt.gca()
        ax.yaxis.offsetText.set_fontsize(18)
        ax.yaxis.set_major_formatter(formater) 
        
        y_range = plt.ylim()      
        if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage":
            ax.set_yscale('log')  
            plt.ylim(1, y_range[1]*10)  
        else:    
            plt.ylim(0, y_range[1])
        plt.title(ylable)
        plt.xlabel(xlable)
        if is_percentage:
            plt.ylim(0, 1)
                
        plt.tight_layout()
        save_path = '../test/pic/layered_MAPF/'+title
        if pre_fix != '':
            save_path = save_path + '_id'
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            print("Folder: " + save_path + " created")
            
        save_path = save_path + "/" + map_key
        plt.savefig(save_path, dpi = 50, bbox_inches='tight')   
        plt.close()
        print("save path to " + save_path)



def drawMethodMapAgentSizes(all_data_map, xlable, ylable, title, is_percentage=False):    
    map_and_agent_data = dict()
    for i in range(1, 5):
        fig=plt.figure(figsize=(5,3.5)) #添加绘图框
        agent_size_and_data_map = dict()
        for map_key, map_value in all_data_map.items():
            if map_format_map_index[map_key] != i: 
                continue
            for method_key, method_value in all_data_map[map_key].items():
                # if method_key not in drawing_method_set:
                #     continue
                x = list()
                y = list()
                sorted_keys = sorted(all_data_map[map_key][method_key].keys())
                if method_key not in agent_size_and_data_map:
                    agent_size_and_data_map[method_key] = dict()
                splited_method_name = method_key.split('_')
                if ylable == 'MaxSubproblemSize' or ylable == 'NumberOfSubProblem':
                    if splited_method_name[0] == 'RAW':
                        # print('continue when ylable = ',ylable,', splited_method_name[0] = ',splited_method_name[0])
                        continue
                for agent_size_key in sorted_keys:
                    x.append(agent_size_key)
                    if agent_size_key not in agent_size_and_data_map[method_key]:
                        agent_size_and_data_map[method_key][agent_size_key] = list()
                    if len(all_data_map[map_key][method_key][agent_size_key]) > 0:
                        y.append(np.mean(all_data_map[map_key][method_key][agent_size_key]))
                        agent_size_and_data_map[method_key][agent_size_key].append(y[-1])
                    else:
                        y.append(0)
                        agent_size_and_data_map[method_key][agent_size_key].append(0)
                        
                # print(splited_method_name)        
                # if len(splited_method_name) == 1:
                #     plt.errorbar(x, y, fmt=map_format_map[map_key]+method_marker_map2[name_of_decomposition], markersize=14, label=map_key+"/"+name_of_decomposition, linewidth=2, elinewidth=4, capsize=4)
                # else:
                #     plt.errorbar(x, y, fmt=map_format_map[map_key]+method_marker_map2[splited_method_name[0]], markersize=14, label=map_key+"/"+splited_method_name[0], linewidth=2, elinewidth=4, capsize=4)
                
                # markerfacecolor='none' marker空心化
                if len(splited_method_name) != 1:
                    temp_width = 2
                    if pre_fix != '' and splited_method_name[0] == 'ID':
                        temp_width = 5
                    if pre_fix == '' and splited_method_name[0] == 'RAW':
                        temp_width = 5    
                    plt.errorbar(x, y, fmt=map_format_map[map_key]+method_marker_map2[splited_method_name[0]], markersize=10, label=map_key+"/"+splited_method_name[0], linewidth=temp_width, elinewidth=4, capsize=4, markerfacecolor='none')
                
        plt.tick_params(axis='both', labelsize=18)
        formater = ticker.ScalarFormatter(useMathText=True)
        formater.set_scientific(True)
        formater.set_powerlimits((0,0))
        ax = plt.gca()
        ax.yaxis.offsetText.set_fontsize(18)
        ax.yaxis.set_major_formatter(formater)
        
        y_range = plt.ylim()
        if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage":
            ax.set_yscale('log')
            plt.ylim(1, y_range[1]*10)
        else:
            plt.ylim(0, y_range[1])
        plt.title(ylable, fontsize=16) # 图片标题
        plt.xlabel(xlable, fontsize=16) # 横坐标标题
        # plt.legend(loc='best', fontsize = 16, ncol=1, handletextpad=.5, framealpha=0.5) # 图例位置设置
        if is_percentage:
            plt.ylim(0, 1)
                
        plt.tight_layout()
        save_path = '../test/pic/layered_MAPF/'+title
        
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            print("Folder: " + save_path + " created")
        if pre_fix == '':    
            save_path = save_path + "/multi_map_"+str(i)
        else:
            save_path = save_path + "/multi_map_"+str(i)+'_id'
        plt.savefig(save_path, dpi = 50, bbox_inches='tight')
        # canvas.paste(plt, (0,0))

        # 2, then create a new image
        # adjust the figure size as necessary

        figsize = (3, 3.5)
        fig_leg = plt.figure(figsize=figsize)
        ax_leg = fig_leg.add_subplot(111)
        # add the legend from the previous axes
        ax_leg.legend(*ax.get_legend_handles_labels(), bbox_to_anchor=(-.15,1.15), ncol = 1, loc='upper left',frameon=False)
        # hide the axes frame and the x/y labels
        ax_leg.axis('off')
        legend_path = '../test/pic/layered_MAPF/'+title+'/'+str(i)+'_legend'
        if pre_fix != '':
            legend_path = legend_path + '_id'
        fig_leg.savefig(legend_path+'.png',dpi = 50)#, bbox_inches='tight')
        plt.close('all')
        print("save path to " + save_path)
        
        # 3, create an image to summary all map in one 
        fig=plt.figure(figsize=(5,3.5)) #添加绘图框
        for method_key, method_value in agent_size_and_data_map.items():
            sorted_keys = sorted(agent_size_and_data_map[method_key].keys())
            x = list()
            y = list()
            splited_method_name = method_key.split('_')
            for agent_size_key in sorted_keys:
                x.append(agent_size_key)
                if len(agent_size_and_data_map[method_key][agent_size_key]) > 0:
                    y.append(np.mean(agent_size_and_data_map[method_key][agent_size_key]))
                else:
                    y.append(0)
                    
                if len(splited_method_name) != 1:
                    plt.errorbar(x, y, fmt=map_format_map[map_key]+method_marker_map2[splited_method_name[0]], markersize=10, label=map_key+"/"+splited_method_name[0], linewidth=2, elinewidth=4, capsize=4, markerfacecolor='none')     
                       
                plt.tick_params(axis='both', labelsize=18)
                
        formater = ticker.ScalarFormatter(useMathText=True)
        formater.set_scientific(True)
        formater.set_powerlimits((0,0))
        ax = plt.gca()
        ax.yaxis.offsetText.set_fontsize(18)
        ax.yaxis.set_major_formatter(formater)
        
        y_range = plt.ylim()
        if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage":
            ax.set_yscale('log')
            plt.ylim(1, y_range[1]*10)
        else:
            plt.ylim(0, y_range[1])
        plt.title(ylable, fontsize=16) # 图片标题
        plt.xlabel(xlable, fontsize=16) # 横坐标标题
        # plt.legend(loc='best', fontsize = 16, ncol=1, handletextpad=.5, framealpha=0.5) # 图例位置设置
        if is_percentage:
            plt.ylim(0, 1)
                
        plt.tight_layout()
        save_path = '../test/pic/layered_MAPF/'+title
        
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            print("Folder: " + save_path + " created")
        if pre_fix == '':
            save_path = save_path + "/multi_map_"+str(i)+'_sum'
        else:
            save_path = save_path + "/multi_map_"+str(i)+'_sum_id'        
        plt.savefig(save_path, dpi = 50, bbox_inches='tight')

def drawSummaryOfMap(all_data_map, xlable, ylable, title, is_percentage=False):
    map_and_agent_data = dict()
    fig=plt.figure(figsize=(5,3.5)) #添加绘图框 
    map_lists = list()
    value_lists_raw = list()
    value_lists_layered = list()
    value_lists_id = list()

    width = 0.4 
    for map_key, map_value in all_data_map.items():
        map_lists.append(map_key)
        for method_key, method_value in all_data_map[map_key].items():
            # if method_key not in drawing_method_set:
            #     continue
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
            
            if pre_fix == '':
                if head_split[0] == 'LAYERED':
                    plt.bar(map_format_list.index(map_key)+1-width/2, np.mean(value), width)    
                    value_lists_layered.extend(value)
                if head_split[0] == 'RAW':  
                    plt.bar(map_format_list.index(map_key)+1+width/2, np.mean(value), width, hatch="//")    
                    value_lists_raw.extend(value)    
                    
            else:
                    
                if head_split[0] == 'LAYERED':  
                    plt.bar(map_format_list.index(map_key)+1-width/2, np.mean(value), width)    
                    value_lists_layered.extend(value)    
                
                if head_split[0] == 'ID':  
                    plt.bar(map_format_list.index(map_key)+1+width/2, np.mean(value), width, hatch="//")    
                    value_lists_id.extend(value)
                    
        plt.xticks(rotation=70)         
    
    print(title + "/" + ylable + "layered = " + str(np.mean(value_lists_layered)) + " / raw = " + str(np.mean(value_lists_raw)) \
        + " / id = " + str(np.mean(value_lists_id)))
    
    plt.tick_params(axis='both', labelsize=14)
    plt.xticks(np.arange(1, len(map_format_list)+1, 1))
    formater = ticker.ScalarFormatter(useMathText=True) 
    formater.set_scientific(True)
    formater.set_powerlimits((0,0))
    ax = plt.gca()
    ax.yaxis.offsetText.set_fontsize(18)
    ax.yaxis.set_major_formatter(formater)      
    
    y_range = plt.ylim()      
    if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage":
        ax.set_yscale('log')  
        plt.ylim(1, y_range[1]*10)  
    else:    
        plt.ylim(0, y_range[1])
    plt.title(ylable, fontsize=16)
    plt.xlabel('Index of map', fontsize=16)
    if is_percentage:
        plt.ylim(0, 1)
            
    # plt.legend(loc='best', fontsize = 16, ncol=1, handletextpad=.5, framealpha=0.5)
    plt.tight_layout()        
    save_path = '../test/pic/layered_MAPF/'+title
    
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
    if pre_fix == '':    
        save_path = save_path + "/" + 'summary'
    else:
        save_path = save_path + "/" + 'summary_id'
    plt.savefig(save_path, dpi = 50, bbox_inches='tight')   
    plt.close()
    print("save path to " + save_path)     
    
    
def drawSummaryOfMethod(all_data_map, xlable, ylable, title, is_percentage=False):
    if pre_fix != '':
        return
    map_and_agent_data = dict()
    fig=plt.figure(figsize=(5,4.5))

    width = 0.4 
    if pre_fix != '':
        width = 0.25
    formater = ticker.ScalarFormatter(useMathText=True) 
    formater.set_scientific(True)
    formater.set_powerlimits((0,0))
    
    value_lists_raw_sum = list()
    value_lists_layered_sum = list()
    value_lists_id_sum = list()
    
    value_lists_raw_x = list()
    value_lists_layered_x = list()
    value_lists_id_x = list()
    
    for method_top, method_value in all_data_map.items():
        value_lists_raw = list()
        value_lists_layered = list()
        value_lists_id = list()
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
                if head_split[0] == 'ID':  
                    value_lists_id.append(np.mean(value))  
                                       
        plt.xticks(rotation=70)    
        common_font_size = 12
        common_rotate_angle = 80
        if pre_fix == '':
            if len(value_lists_raw) > 0:
                #p2 = plt.bar(drawing_method_set.index(method_top)-width/2, np.mean(value_lists_raw), width, label="RAW")   
                #plt.bar_label(p2, label_type='edge', fmt="%.2g", rotation=common_rotate_angle, fontsize=common_font_size)#, fmt=formater)   
                value_lists_raw_sum.append(np.mean(value_lists_raw))
                value_lists_raw_x.append(drawing_method_set.index(method_top)-width/2)
            if len(value_lists_layered) > 0:
                #p1 = plt.bar(drawing_method_set.index(method_top)+width/2, np.mean(value_lists_layered), width, hatch="//", label="LAYERED") 
                #plt.bar_label(p1, label_type='edge', fmt="%.2g", rotation=common_rotate_angle, fontsize=common_font_size)#, fmt=formater) 
                value_lists_layered_sum.append(np.mean(value_lists_layered))
                value_lists_layered_x.append(drawing_method_set.index(method_top)+width/2)
  
        else:
            if len(value_lists_raw) > 0:
                #p2 = plt.bar(drawing_method_set.index(method_top)-width, np.mean(value_lists_raw), width, label="RAW")   
                #plt.bar_label(p2, label_type='edge', fmt="%.2g", rotation=common_rotate_angle, fontsize=common_font_size)#, fmt=formater) 
                value_lists_raw_sum.append(np.mean(value_lists_raw)) 
                value_lists_raw_x.append(drawing_method_set.index(method_top)-width)                    
            if len(value_lists_layered) > 0:
                #p1 = plt.bar(drawing_method_set.index(method_top), np.mean(value_lists_layered), width, hatch="//", label="LAYERED") 
                #plt.bar_label(p1, label_type='edge', fmt="%.2g", rotation=common_rotate_angle, fontsize=common_font_size)#, fmt=formater) 
                value_lists_layered_sum.append(np.mean(value_lists_layered))
                value_lists_layered_x.append(drawing_method_set.index(method_top))                    

            if len(value_lists_id) > 0:
                #p1 = plt.bar(drawing_method_set.index(method_top)+width, np.mean(value_lists_id), width, hatch="o", label="ID") 
                #plt.bar_label(p1, label_type='edge', fmt="%.2g", rotation=common_rotate_angle, fontsize=common_font_size)#, fmt=formater)     
                value_lists_id_sum.append(np.mean(value_lists_id))
                value_lists_id_x.append(drawing_method_set.index(method_top)+width)                    

             
    if pre_fix == '':
        p1 = plt.bar(value_lists_raw_x,     value_lists_raw_sum,     width, label="RAW")   
        p2 = plt.bar(value_lists_layered_x, value_lists_layered_sum, width, label="LAYERED", hatch="//")   

    else:
        p1 = plt.bar(value_lists_raw_x,     value_lists_raw_sum,     width, label="RAW")   
        p2 = plt.bar(value_lists_layered_x, value_lists_layered_sum, width, label="LAYERED", hatch="//")   
        p3 = plt.bar(value_lists_id_x,      value_lists_id_sum,      width, label="ID",      hatch="o")   
 
    
    plt.tick_params(axis='both', labelsize=18)

    ax = plt.gca()
    ax.yaxis.set_major_formatter(formater)     
    plt.xticks([0,1,2,3,4,5,6], drawing_method_set_2)
    
    y_range = plt.ylim()      
    if ylable == "Sum of cost" or ylable == "Makespan" or ylable == "Memory usage":
        ax.set_yscale('log')  
        plt.ylim(y_range[0]/10, y_range[1]*10)  
    else:    
        plt.ylim(0, y_range[1]*1.1)
    plt.title(ylable, fontsize = 18)
    if is_percentage:
        plt.ylim(0, 1)
        
    offset_text = ax.yaxis.get_offset_text()
    offset_text.set_fontsize(18)    
        
    plt.legend(loc='best', fontsize = 16)#, ncol=1, handletextpad=.5, framealpha=0.5)
    plt.tight_layout()        
    save_path = '../test/pic/layered_MAPF/'+title
    
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print("Folder: " + save_path + " created")
    if pre_fix == '':    
        save_path = save_path + "/" + 'method_summary'+'.png'
    else:
        save_path = save_path + "/" + 'method_summary'+'_id.png'
    plt.savefig(save_path, dpi = 200, bbox_inches='tight')   
    plt.close()
    print("save path to " + save_path)  


def removeMethodDataFromFile(file_path, temp_method_name):
    filtered_lines = list()
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()
            for line in lines:
                splited_line = line.split()
                # print('old_line=',line)
                head_split = splited_line[0].split('_')
                if head_split[1] != temp_method_name:
                    # print('new_line=',line)
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
        data_file_path = data_path_dir + map_name_key + '.txt'
        print('remove data of ', method_name_local, ' from', data_file_path)
        removeMethodDataFromFile(data_file_path, method_name_local)
    


    
#pre_fix = 'ID/' # use for compare with independence detection
pre_fix = '' # use for compare with raw mapf

data_path_dir = '../test/test_data/layered_mapf/'+pre_fix

all_single_data = list()

name_of_decomposition = "DECOMPOSITION"

# method_marker_map = {"RAW_EECBS":'p-',       "LAYERED_EECBS":'p--',
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
#                   name_of_decomposition:"D-."
#                   }

method_marker_map2 = {
    "RAW":':',
    "LAYERED":'-',
    "ID":':',
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


# o, *, p, 3, H, X, P, < 
# map in the same image must have different marker
map_format_map = {
                 "empty-16-16":'o', # 1
                 "empty-32-32":'o', # 2
                 
                 "maze-32-32-2":'*', # 1
                 "maze-32-32-4":'p', # 1 
                 "maze-128-128-2":'*', # 2
                 "maze-128-128-10":'o', # 3
                 
                 "den312d":'p', # 2
                 "den520d":'*', # 3
                 
                 "Berlin_1_256":'p', # 3
                 "Paris_1_256":'s', # 3
                 
                 "ht_chantry":'H', # 3
                 "lak303d":'X', # 3

                 "random-32-32-20":'s', # 1
                 "random-64-64-20":'o',  # 4

                 "room-32-32-4":'H', # 1
                 "room-64-64-8":'s', # 2
                 "room-64-64-16":'*', # 4

                 "warehouse-10-20-10-2-1":'H', # 2
                 ####
                 "warehouse-10-20-10-2-2":'p', # 4
                 "warehouse-20-40-10-2-1":'s', # 4
                 "warehouse-20-40-10-2-2":'H', # 4
                 
                 "Boston_0_256":'X', # 4 
                 "lt_gallowstemplar_n":'P', # 4
                 "ost003d":'<' # 4

}

# when draw how multiple maps in one figure, draw map that have similar agent size range in one figure 
map_format_map_index = {
                 "empty-16-16":1, # 120
                 "room-32-32-4":1, # 200
                 "maze-32-32-2":1, # 120
                 "maze-32-32-4":1, # 240
                 "random-32-32-20":1,# 240

                  "empty-32-32":2, # 400
                 "maze-128-128-2":2, # 700
                 "den312d":2, # 800
                 "room-64-64-8":2, # 700
                 "warehouse-10-20-10-2-1":2, # 800

                 "maze-128-128-10":3, # 1000
                 "den520d":3, # 900
                 "Berlin_1_256":3, # 900
                 "Paris_1_256":3, # 1000
                 "ht_chantry":3, # 1000
                 "lak303d":3, # 1000
                 
                 "random-64-64-20":4,  # 1000
                 "room-64-64-16":4, # 1000
                 "warehouse-10-20-10-2-2":4, # 1000
                 "warehouse-20-40-10-2-1":4, # 1000
                 "warehouse-20-40-10-2-2":4, # 1000
                 "Boston_0_256":4, # 1000
                 "lt_gallowstemplar_n":4, # 1000
                 "ost003d":4 # 1000

}


 
# 1, load all data
for map_name_key, map_format_value in map_format_map_index.items():
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

# only used in comparison  
all_method_max_subproblem_map = dict()    
all_method_subproblem_size_map = dict()    

drawing_method_set = [
                      "EECBS", # tested ok, advance in time cost, nothing in memory usage
                      "PBS",  # tested ok, advance in time cost, advance in memory usage, full map
                      "PushAndSwap", # some map not ok, advance in time cost, nothing in memory usage, full map
                      "LaCAM",# tested ok, drawback in time cost, drawback in memory usage, full map
                      "HCA", # test ok, advance in memory usage, drawback in time cost, full map
                      "PIBT2", # some map not ok，advance in memory usage, drawback in time cost, full map
                      "LNS", # test ok, advance in memory usage, advance in time cost
                       "CBS"
                     ]

drawing_method_set_2 = [ 
                      "EECBS", # tested ok, advance in time cost, nothing in memory usage
                      "PBS",  # tested ok, advance in time cost, advance in memory usage, full map
                      "PAS", # some map not ok, advance in time cost, nothing in memory usage, full map
                      "LaCAM",# tested ok, drawback in time cost, drawback in memory usage, full map
                      "HCA*", # test ok, advance in memory usage, drawback in time cost, full map
                      "PIBT+", # some map not ok，advance in memory usage, drawback in time cost, full map
                      "LNS2", # test ok, advance in memory usage, advance in time cost
                      #"CBS"
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
            
        if all_method_subproblem_size_map.get(method_name) == None:    
            all_method_subproblem_size_map[method_name] = dict()                
                   
        if all_method_time_cost_map[method_name].get(single_data.map_name) == None:
            all_method_time_cost_map[method_name][single_data.map_name] = dict()
            all_method_total_cost_map[method_name][single_data.map_name] = dict()
            all_method_makespan_map[method_name][single_data.map_name] = dict()
            all_method_memory_usage_map[method_name][single_data.map_name] = dict()
            all_method_success_rate_map[method_name][single_data.map_name] = dict()
            all_method_max_subproblem_map[method_name][single_data.map_name] = dict()
            all_method_subproblem_size_map[method_name][single_data.map_name] = dict()


        if all_method_time_cost_map[method_name][single_data.map_name].get(line_data.method) == None:
            all_method_time_cost_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_time_cost_map[method_name][single_data.map_name][name_of_decomposition] = dict()
            all_method_total_cost_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_makespan_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_memory_usage_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_success_rate_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_max_subproblem_map[method_name][single_data.map_name][line_data.method] = dict()
            all_method_subproblem_size_map[method_name][single_data.map_name][line_data.method] = dict()            
            
        if all_method_time_cost_map[method_name][single_data.map_name][line_data.method].get(line_data.agent_count) == None:
            all_method_time_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()
            all_method_time_cost_map[method_name][single_data.map_name][name_of_decomposition][line_data.agent_count] = list()
            all_method_total_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()
            all_method_makespan_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()    
            all_method_memory_usage_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()    
            all_method_success_rate_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list() 
            all_method_max_subproblem_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list()    
            all_method_subproblem_size_map[method_name][single_data.map_name][line_data.method][line_data.agent_count] = list() 
            
        all_method_time_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.time_cost)
        all_method_time_cost_map[method_name][single_data.map_name][name_of_decomposition][line_data.agent_count].append(line_data.subgraph_init_time_cost)
        all_method_success_rate_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.success)    
        all_method_memory_usage_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.max_memory_usage)

        
        if line_data.success == 1:
            all_method_total_cost_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.total_cost)
            all_method_makespan_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.max_single_cost)
        
        if pre_fix != '':    
            all_method_max_subproblem_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.max_sub_problem_size)    
            all_method_subproblem_size_map[method_name][single_data.map_name][line_data.method][line_data.agent_count].append(line_data.total_number_of_subproblem)

for method_key, method_value in all_method_time_cost_map.items(): 
    # if method_key != method_name:
    #     continue
    
    # print(method_key+'/'+method_name)
    # draw at each agent size
    # drawMethodMaps(all_method_time_cost_map[method_key], "Number of agents", "Time cost(ms)", "time_cost/"+method_key)           
    # drawMethodMaps(all_method_memory_usage_map[method_key], "Number of agents", "Memory usage", "memory_usage/"+method_key)           
    # drawMethodMaps(all_method_total_cost_map[method_key], "Number of agents", "Sum of cost", "sum_of_cost/"+method_key)           
    # drawMethodMaps(all_method_makespan_map[method_key], "Number of agents", "Makespan", "makespan/"+method_key)           
    # drawMethodMaps(all_method_success_rate_map[method_key], "Number of agents", "Success rate", "success_rate/"+method_key)        
    
    # drawMethodMapAgentSizes(all_method_time_cost_map[method_key], "Number of agents", "Time cost(ms)", "time_cost/"+method_key)
    # drawMethodMapAgentSizes(all_method_memory_usage_map[method_key], "Number of agents", "Memory usage", "memory_usage/"+method_key)           
    # drawMethodMapAgentSizes(all_method_total_cost_map[method_key], "Number of agents", "Sum of cost", "sum_of_cost/"+method_key)           
    # drawMethodMapAgentSizes(all_method_makespan_map[method_key], "Number of agents", "Makespan", "makespan/"+method_key)           
    # drawMethodMapAgentSizes(all_method_success_rate_map[method_key], "Number of agents", "Success rate", "success_rate/"+method_key)    
    # drawMethodMapAgentSizes(all_method_max_subproblem_map[method_key], "Number of agents", "MaxSubproblemSize", "max_subproblem/"+method_key)    
    # drawMethodMapAgentSizes(all_method_subproblem_size_map[method_key], "Number of agents", "NumberOfSubProblem", "sub_problem_size/"+method_key)    

    # draw summary of maps
    drawSummaryOfMap(all_method_time_cost_map[method_key], "Number of agents", "Time cost(ms)", "time_cost/"+method_key)    
    drawSummaryOfMap(all_method_memory_usage_map[method_key], "Number of agents", "Memory usage", "memory_usage/"+method_key)           
    drawSummaryOfMap(all_method_total_cost_map[method_key], "Number of agents", "Sum of cost", "sum_of_cost/"+method_key)           
    drawSummaryOfMap(all_method_makespan_map[method_key], "Number of agents", "Makespan", "makespan/"+method_key)           
    drawSummaryOfMap(all_method_success_rate_map[method_key], "Number of agents", "Success rate", "success_rate/"+method_key)      
    drawSummaryOfMap(all_method_max_subproblem_map[method_key], "Number of agents", "MaxSubproblemSize", "max_subproblem/"+method_key)      
    drawSummaryOfMap(all_method_subproblem_size_map[method_key], "Number of agents", "NumberOfSubProblem", "sub_problem_size/"+method_key)    

#draw summary of methods
drawSummaryOfMethod(all_method_time_cost_map, "Number of agents", "Time cost(ms)", "time_cost")           
drawSummaryOfMethod(all_method_memory_usage_map, "Number of agents", "Memory usage", "memory_usage")           
drawSummaryOfMethod(all_method_total_cost_map, "Number of agents", "Sum of cost", "sum_of_cost")           
drawSummaryOfMethod(all_method_makespan_map, "Number of agents", "Makespan", "makespan")           
drawSummaryOfMethod(all_method_success_rate_map, "Number of agents", "Success rate", "success_rate")      
drawSummaryOfMethod(all_method_max_subproblem_map, "Number of agents", "MaxSubproblemSize", "max_subproblem")      
drawSummaryOfMethod(all_method_subproblem_size_map, "Number of agents", "NumberOfSubProblem", "sub_problem_size")      

# TODO: create a big canvas that have all figures, simplify visualization
# draw all figure of a method under all map (condensed into four fig, each fig have approx 6 maps)
# five types of data, 
#print('haha')


def display_images_in_grid(image_filess, method_name, visualize=False):
    """
    读取指定文件夹中的多张图片，并以网格形式显示在一张图上。
    
    参数:
        image_folder (str): 图片所在的文件夹路径。
        rows (int): 网格的行数。
        cols (int): 网格的列数。
    """
    # 获取文件夹中的所有图片
    #image_files = [f for f in os.listdir(image_folder) if f.endswith(('png', 'jpg', 'jpeg'))]
    
    # 检查图片数量
    for image_files in image_filess:
        if len(image_files) == 0:
            print("未找到图片，请检查文件夹路径和图片格式。")
            return
    
    rows = len(image_filess)
    
    # print("rows", rows)
    
    cols = len(image_filess[0])
    
    print('rows, cols = ',rows, cols) # rows, cols =  1 6

    
    # 限制图片数量以匹配网格
    image_files = image_files[:rows * cols]
    
    # 创建子图
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 5, rows * 3.5))
    axes = axes.flatten()  # 将多维数组转换为一维，方便迭代
    
    for i, ax in enumerate(axes):
        # if i < len(image_files):
        #     print('i/cols and icols',i/cols,i%cols)
        #     print('image_filess[int(i/cols)][i_cols]',image_filess[int(i/cols)][i%cols])
        #     #img_path = os.path.join(image_folder, image_files[i])
        #     if not os.path.exists(image_filess[int(i/cols)][i%cols]):
        #         continue
        #     img = mpimg.imread(image_filess[int(i/cols)][i%cols])
        #     ax.imshow(img)
        #     ax.axis('off')  # 隐藏坐标轴
        #     #ax.set_title(f'Image {i+1}')
        #     ax.set_title(method_name)
        # else:
        #     ax.axis('off')  # 多余的子图隐藏
        # print('i = ', i)
        # print('i/cols and icols',i/cols,i%cols)
        # print('image_filess[int(i/cols)][i_cols]',image_filess[int(math.floor(i/cols))][i%cols])
        #img_path = os.path.join(image_folder, image_files[i])
        if not os.path.exists(image_filess[int(i/cols)][i%cols]):
            print('not exist: ', image_filess[int(i/cols)][i%cols])
            continue
        img = mpimg.imread(image_filess[int(i/cols)][i%cols])
        ax.imshow(img)
        ax.axis('off')  # 隐藏坐标轴
        #ax.set_title(f'Image {i+1}')
        ax.set_title(method_name)
    
    plt.tight_layout()
    save_path = '../test/pic/layered_MAPF/'+method_name+'_summary'
    if pre_fix != '':
        save_path = save_path + '_id'
    print('save summary to ', save_path)    
    plt.savefig(save_path, dpi = 50, bbox_inches='tight')   
    if visualize:
        plt.show()
    
image_folder = '../test/pic/layered_MAPF/' 
data_type_names = ['time_cost', 'success_rate', 'sum_of_cost', 'makespan', 'memory_usage']
# method_name = 'EECBS'

#removeMethodDataFromFiles(map_format_map_index, 'HCA')

# for method_name in drawing_method_set:
#     all_image_filess = []
#     for i in range(1, 5):
#         all_image_files = []
#         for type_name in data_type_names:
#             load_path = image_folder + type_name +'/'+ method_name +'/'+ 'multi_map_'+str(i)
#             if pre_fix != '':
#                 load_path = load_path + '_id'
#             print('load from ', load_path)    
#             all_image_files.append(load_path+'.png')
#         if pre_fix != '':
#             load_path = image_folder + 'max_subproblem' +'/'+ method_name +'/multi_map_'+ str(i)+'_id'
#             print('load from id ', load_path)    
#             all_image_files.append(load_path+'.png')
#             load_path = image_folder + 'sub_problem_size' +'/'+ method_name +'/multi_map_'+ str(i)+'_id'
#             print('load from id ', load_path)    
#             all_image_files.append(load_path+'.png')

#         all_image_files.append(image_folder + type_name +'/'+ method_name +'/'+ str(i)+'_legend.png')
#         all_image_filess.append(all_image_files)
#     display_images_in_grid(all_image_filess, method_name)



# map_format_map_index
# 1, empty-16-16: EECBS, PBS, LNS, HCA
# 1, maze-32-32-4: EECBS, PBS, LNS, HCA
# 1, room-32-32-4: EECBS, PBS，LNS, HCA
# 1, maze-32-32-2: EECBS, PBS, LNS, HCA
# 1, random-32-32-20: EECBS, PBS, LNS, HCA

# 2, empty-32-32: EECBS, PBS, LNS, HCA
# 2, maze-128-128-2: EECBS, PBS, LNS, HCA
# 2, den312d: EECBS, PBS, LNS, HCA(ing)
# 2, room-64-64-8: 
# 2, warehouse-10-20-10-2-1: 

# 3, maze-128-128-10: 
# 3, den520d:
# 3, Berlin_1_256: 
# 3, Paris_1_256: 
# 3, ht_chantry:
# 3, lak303d: 
    
# 4, random-64-64-20: 
# 4, room-64-64-16: 
# 4, warehouse-10-20-10-2-2: 
# 4, warehouse-20-40-10-2-1: 
# 4, warehouse-20-40-10-2-2:
# 4, Boston_0_256: 
# 4, lt_gallowstemplar_n: 
# 4, ost003d: 
