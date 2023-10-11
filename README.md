# ArduPilot_Develop Project

[Rover4.2源代码](https://gitee.com/lisq58/ardupilot_develop/tree/Rover-4.2.0/)

本文档为底层代码开发的说明与帮助

## Git使用指南 ##
[git下载地址](https://git-scm.com/downloads)  
***克隆仓库***：git clone -b "分支名称" "远程仓库地址"  
***设置当前git的使用者名称***：git config --global user.name "你的名字"  
***设置当前git的使用者邮箱***：git config --global user.email "你的邮箱"   
***查看git日志***:git log  
***查看当前分支状态***：git status   
***提交本次修改至暂存区***：git add 文件名称   
***提交本次修改至仓库***：git commit -m “本次修改的备注"   
***将当前修改合并到上一次提交并修改上次提交的备注***：git commit --amend   
***将当前修改存入临时区域以方便切换分支或其他操作***：git stash   
***将存入临时区域中的最近一次修改恢复***：git stash pop   
***查看当前所有创建的分知名***:git branch   
***切换分支***：git checkout 分支名称   
***新建当前分支副本并切换至新分支***：git checkout -b 分支名称   
***显示所有已创建远端仓库的名称及其网址***：git remote -v   
***新建远端仓库***：git remote add 远端仓库名称 远端仓库https(仅限gitee，若需要拉取git代码应使用ssh[参考](https://blog.csdn.net/ljk126wy/article/details/87881923))   
***拉取远端仓库指定分支同时为该分支创建本地分支***：git fetch 远端仓库名称 远端仓库指定分支:新建本地分支名   
***添加指定提交的代码***：git cherry-pick 该提交的hash值   
***将分支回溯到指定提交版本并舍弃所有该提交之后的更改***：git reset --hard 指定提交的hash值   
***将分支回溯到指定提交版本保留所有该提交之后的更改***：git reset --soft 指定提交的hash值   

## 仓库使用说明 ##
通过missionplanner_param文件中的参数表使用各个版本的代码    
[添加新的项目或任务](https://cvwz2djf69d.feishu.cn/share/base/form/shrcnOKT2Q66Vw8STW1jTLXB29e)   
[项目进度更新](https://cvwz2djf69d.feishu.cn/base/B5cpbx0nKakEaRs8d4pc0EIenAd?table=tblhPtvXAZk9vOFC&view=vewXxBNTOK)   
## 更新者代码网址 ##

- [陈导](https://github.com/andychen1986/ardupilot):
  - ***主要开发模块***: AE_RobotArmWP,AE_RobotArmInfo,AE_RobotArmWP,AE_SlewingEncoder

- [李绍谦](https://gitee.com/lisq58/my_ardupilot):
  - ***主要开发模块***: AE_Motors
- [周湛豪](https://gitee.com/chengweidaniu/ardupilot):
  - ***主要开发模块***：Excavation_mode
- [骆任贤](https://gitee.com/Cyanluorx/Excavator.git):
  - ***主要开发模块***: AE_Control_Arm,AE_Mission_Arm,AE_Navigation_Arm,AE_RobotArmWP,AE_WPNav_Arm,SIM_TBM
- [林翔宇](https://github.com/LinXiangY/ardupilot/tree/TBM_cuttinghead):
  - ***主要开发模块***：AE_RobotArmInfo
