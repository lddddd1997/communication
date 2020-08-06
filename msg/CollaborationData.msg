std_msgs/Header header

int32 uav_id                 #无人机ID

int32 task_target_id         #目标编号
int32 task_flag              #标志位（0为编队或搜索，1为执行任务）
int32 task_type              #任务类型

float32[3] pos_uav           #无人机位置信息 x y z
float32 yaw_uav              #无人机偏航

int32 target_id              #目标ID
float32[3] pos_target        #目标信息 x y v 
float32 yaw_target           #目标偏航
int32 num                    #车身数字
int32 num_side               #数字在哪一侧
int32 target_find            #发现目标标志
int32 target_find_uav_id     #发现者
int32 num_change             #数字改变

int32[9] assign_array        #分配情况（3*3矩阵，每个目标分配的无人机情况）