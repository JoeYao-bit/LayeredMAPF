import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker
from scipy.interpolate import griddata

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
                
                new_data.level_1           = float(splited_line[7])
                new_data.level_2           = float(splited_line[8])
                new_data.level_3           = float(splited_line[9])
                
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
    level_1           = 0.
    level_2           = 0.
    level_3           = 0.
        

# all data in a txt file
class SingleTestData:
    data_list = list()
    map_name = ''


short_name = ['IC', 'BC', 'LS']    
    
def drawMethodMap(single_map_data, value_type):
    fig = plt.figure(figsize=(5,3.5)) #添加绘图框
    map_name = single_map_data.map_name
    
    all_raw_data = [dict(), dict(), dict()]


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
        
        assert(data.level>= 1 and data.level <= 3)
        
        if value_type == "decomposition_rate":
            all_raw_data[data.level-1][total_size].append(max_cluster / total_size)    
        elif value_type == "time_cost":
            all_raw_data[data.level-1][total_size].append(time_cost)    
        elif value_type == "memory_usage":
            all_raw_data[data.level-1][total_size].append(memory_usage)        
        elif value_type == "num_of_subproblem":
            all_raw_data[data.level-1][total_size].append(num_of_subproblem)     
    label_buffer = '' # short_name[0] #"1"        
    for i in range(0,3):    
        x = list()
        y = list()    
        std_val = list()
        if value_type == "decomposition_rate":
            for data_key, data_val in all_raw_data[i].items():
                x.append(data_key)        
                y.append(np.mean(data_val))
                std_val.append(np.std(data_val))
        else:
            for data_key, data_val in all_raw_data[i].items():
                x.append(data_key)        
                y.append(np.mean(data_val))
                std_val.append(np.std(data_val))
        # yerr=std_val
        label_buffer = label_buffer + ", " + short_name[i] #+ str(i+2)          
        plt.errorbar(x, y, label=label_buffer, markersize=14, fmt=step_fmt[i], linewidth= 4, elinewidth=4, capsize=4)

    plt.legend(loc='best')    
    plt.tick_params(axis='both', labelsize=18)
    #plt.ticklabel_format(style='sci', scilimits=(0,0), axis='y')
    
    formater = ticker.ScalarFormatter(useMathText=True) 
    formater.set_scientific(True)
    formater.set_powerlimits((0,0))
    
    ax = plt.gca()
    ax.yaxis.offsetText.set_fontsize(18)
    ax.yaxis.set_major_formatter(formater)
    y_range = plt.ylim()
    plt.ylim(0, y_range[1])
    if value_type == "decomposition_rate":
        plt.ylim(0, 1)

    plt.legend(loc='best', fontsize = 18, ncol=1, handletextpad=.5, framealpha=0.5)
    plt.xlabel('Num of agents', fontsize = 20)
    #plt.grid()
    plt.tight_layout()
    save_path = '../test/pic/decomposition/'+value_type+'/'+map_name+"-"+value_type+'.png'
    plt.savefig(save_path, dpi = 200, bbox_inches='tight')   
    plt.close()
    print("save picture to "+save_path)


# draw how max subproblem size change as agent size, and agent density changes
def drawMaxSubAgent3D(all_map_data):
    x = list()
    y = list()
    z = list()
    for single_map_data in all_single_data:
        map_name = single_map_data.map_name
        for line_data in single_map_data.data_list:
            if line_data.level != 3:
                continue
            num_of_agent = line_data.total_size
            density_of_agent = float(num_of_agent)/map_scale_and_free_grid_size[map_name][2]
            max_sub_problem = line_data.max_cluster
            
            # print("num_of_agent/density_of_agent/max_sub_problem = ",num_of_agent,' ', density_of_agent, ' ', max_sub_problem)
            
            x.append(num_of_agent)
            y.append(density_of_agent)
            z.append(max_sub_problem)
            
    # 散点图
    fig = plt.figure(figsize=(5, 4))
    ax = plt.axes(projection='3d')
    ax.set_xlabel('Number of Agents')
    ax.set_ylabel('Agent Density')
    ax.set_zlabel('Max Subproblem Size')
    
    # s：marker标记的大小
    # c: 颜色  可为单个，可为序列
    # depthshade: 是否为散点标记着色以呈现深度外观。对 scatter() 的每次调用都将独立执行其深度着色。
    # marker：样式
    scatter = ax.scatter(x,y,z,     
        c=z,              # 颜色基于z值
        cmap='viridis',   # 颜色映射方案
        s=30,             # 点的大小（可选）
        alpha=0.8,        # 透明度（可选）
        #edgecolor='k',     # 点边缘颜色（可选）
        marker="."   
    )
    # 添加颜色条
    cbar = plt.colorbar(scatter, ax=ax, location='left', pad=0.01, shrink=0.6)
    cbar.set_label('Max Subproblem Size')

    # plt.show()
    #plt.tight_layout()
    save_path = '../test/pic/decomposition/summary.png'
    plt.savefig(save_path, dpi = 200)#, bbox_inches='tight')   
    plt.close()
    print("save picture to "+save_path)
    
    # # 将点云投影到XY平面，并生成网格
    # grid_x, grid_y = np.mgrid[min(x):max(x):50j, min(y):max(y):50j]

    # # 提取每个网格内的最大Z值（上界点）
    # points = np.vstack((x, y)).T
    # grid_z_max = griddata(points, z, (grid_x, grid_y), method='nearest')

    # # 使用插值平滑曲面（可选：method='linear'或'cubic'）
    # grid_z_smooth = griddata(points, z, (grid_x, grid_y), method='cubic')

    # # 创建3D图像
    # fig = plt.figure(figsize=(10, 6))
    # ax = fig.add_subplot(111, projection='3d')

    # # 绘制原始点云
    # ax.scatter(x, y, z, c='blue', s=10, label='原始点云', alpha=0.6)

    # # 绘制上界曲面（仅显示有效区域）
    # if not np.isnan(grid_z_smooth).all():
    #     ax.plot_surface(grid_x, grid_y, grid_z_smooth, 
    #                     cmap='viridis', alpha=0.7, 
    #                     label='拟合上界')

    # #ax.legend()
    # ax.set_xlabel('X轴')
    # ax.set_ylabel('Y轴')
    # ax.set_zlabel('Z轴')
    # plt.title('3D点云上界拟合')
    # plt.show()

    
step_fmt = ["x-","o-.","^--"]    
    
data_path_dir = '../test/test_data/decomposition/'
all_map_name = [
                "empty-16-16",
                "empty-32-32",
                
                "maze-32-32-2",
                "maze-32-32-4",
                "maze-128-128-10",
                "maze-128-128-2",
                
                "den312d",
                "den520d",
                
                "Berlin_1_256",
                "Paris_1_256",
                
                "ht_chantry",
                "lak303d",
                
                #"random-64-64-10",
                "random-64-64-20",
                "random-32-32-20",
                
                "room-64-64-16",
                "room-64-64-8",
                "room-32-32-4",
                
                "warehouse-10-20-10-2-1",
                "warehouse-10-20-10-2-2",
                "warehouse-20-40-10-2-1",
                "warehouse-20-40-10-2-2",
                
                "Boston_0_256",
                "lt_gallowstemplar_n",
                "ost003d"
                ]

map_scale_and_free_grid_size = {
                 "empty-16-16": [16, 16, 256], # 120
                 "room-32-32-4": [32, 32, 682], # 200
                 "maze-32-32-2": [32, 32, 666], # 120
                 "maze-32-32-4": [32, 32, 790], # 240
                 "random-32-32-20": [32, 32, 819],# 240

                  "empty-32-32": [32, 32, 1024], # 400
                 "maze-128-128-2": [128, 128, 10858], # 700
                 "den312d": [65, 85, 2445], # 800
                 "room-64-64-8": [64, 64, 3678], # 700
                 "warehouse-10-20-10-2-1": [161, 63, 5699], # 800

                 "maze-128-128-10":[128, 128, 14818], # 1000
                 "den520d": [256, 257, 28178], # 900
                 "Berlin_1_256": [256, 256, 47540], # 900
                 "Paris_1_256": [256, 256, 47240], # 1000
                 "ht_chantry": [162, 141, 7461], # 1000
                 "lak303d": [194, 194, 14784], # 1000
                 
                 "random-64-64-20": [64, 64, 3687],  # 1000
                 "room-64-64-16": [64, 64, 3646], # 1000
                 "warehouse-10-20-10-2-2": [170, 84, 9776], # 1000
                 "warehouse-20-40-10-2-1": [321, 123, 22599], # 1000
                 "warehouse-20-40-10-2-2": [340, 164, 38756], # 1000
                 "Boston_0_256": [256, 256, 47768], # 1000
                 "lt_gallowstemplar_n": [251, 180, 10021], # 1000
                 "ost003d": [194, 194, 13214] # 1000

}

all_single_data = list()

for map_name in all_map_name:
    data_file_path = data_path_dir + map_name + '_de.txt'
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

# drawMaxSubAgent3D(all_single_data)    
#plt.show()    