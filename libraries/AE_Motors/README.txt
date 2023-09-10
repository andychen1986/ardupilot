***未进行设置严重影响功能实现
**未进行设置无法进行对应控制
*一般辅助功能

AE_Motors共有四个输出通道
分别为k_boom = 111,k_forearm = 112,k_bucket = 113,k_rotation = 114**
将指定severo通道设置为上面四个值即可使用
设置四个通道为excavator

AEMOT_OUT_MIN设置输出最小值 并以最小值为基准对输出进行缩放*

加入软限幅通过设置CLD1_BM_MX，CLD2_FM_MX，CLD3_BK_MX来限制大臂、小臂、铲斗最大杆臂长度***
通过设置CLD1_BM_MN，CLD2_FM_MN，CLD3_BK_MN来限制大臂、小臂、铲斗最小杆臂长度
参数于初始化时获取，需要在更改参数后重启飞控

加入两个输出通道
分别为k_cutting_header = 115,k_support_leg = 116**
将指定severo通道设置为上面四个值即可使用
设置k_boom，k_rotation，k_cutting_header,k_support_leg为TBM
加入AE_Motors两种输出组合的区分，分别为excavator与TBM由参数表AE_type确定***
参数于初始化时获取，需要在更改参数后重启飞控