import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

# 替换成你找到的实际路径
font_path = '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc'

try:
    font = FontProperties(fname=font_path)
    plt.figure()
    plt.text(0.5, 0.5, '机器人数量', fontproperties=font, fontsize=24, ha='center')
    plt.title('标题测试', fontproperties=font)
    plt.xlabel('哈哈', fontproperties=font)
    print("字体加载成功！")
    save_path = '/home/yaozhuo/code/LayeredMAPF/test/pic/test_cn.png'
    plt.savefig(save_path, dpi = 200, bbox_inches='tight')   
    plt.show()

except Exception as e:
    print(f"字体加载失败：{e}")
