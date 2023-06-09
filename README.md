# robot_projects
室内巡检机器人项目 
# 大纲
## 1. 简介
为了适应老龄化社会的发展，加速家用小型服务型机器人的落地应用，本人将开发一款小型的自主家庭巡检安防机器人。
其核心在于能够在家庭环境内自主巡航的同时监测家庭环境的异常状况(譬如老人跌倒、陌生人入侵、有害气体浓度、
易燃气体浓度、烟雾浓度、火灾情况等等)，并且能作为智慧家居的核心接入各种外设(空调、灯光、窗帘、睡眠检测仪等等)，方便对智慧家居设备进行
统一管理、调度和控制;

## 2. 项目开发分享大纲
本人将对整个开发过程进行记录以及分享。提供包括电路设计、外壳设计、算法设计以及代码实现部分的文档以及源码，
便于交流以及二次开发。
针对分享过程中的流程和方式在这里先做简单的介绍，后续会根据大纲进行详细的梳理。
整个分享过程将分为10个大的篇章，每个章节会以文字为主并辅以相应的图片甚至是视频资料。
### 2.1 第一篇--前序
第一篇中主要是从整体上对该机器人做一个详细的介绍，让人知道整个项目究竟要做什么。在这一部分中，将从机器人
的整体硬件框架以及软件框架入手，并分模块大致的介绍各个模块的功能和用途。让人产生一个直观的认识，然后在后续
篇章中会对每一个模块进行拆分细致的介绍。

### 2.2 第二篇--机器人下位机驱动电路设计
首先将介绍机器人底层控制板的电路设计方案，以及使用这种方案的好处、并提供相应的pcb和bom表。

### 2.2 第三篇--机器人下位机驱动软件设计（编码器、直流电机以及IMU篇）
由于机器人所需的外部硬件设备较多，因此关于底层硬件驱动设计这块将分为若干个篇章来介绍。首先将介绍轮式
移动机器人中的核心外设--编码器、直流电机以及IMU的硬件电路设计以及软件驱动设计。

### 2.3 第四篇--机器人下位机驱动软件设计（超声波传感器、温湿度传感器、烟雾传感器等各种传感器篇）
机器人由于承担了家庭环境监测的任务，因此需要针对性的加装各种传感器。在这一章中将介绍各种传感器的用途、
驱动方法。

### 2.4 第五篇--机器人上位机软件设计（上下位机间通信程序设计篇）
该机器人的上位机采用地平线公司的X3派开发板，在这一篇中会介绍如何使用串口协议来架起上下位机之间的沟通桥梁。
因为下位机中采集到的所有数据都要送到上位机中进行处理，而上位机需要对下位机发送控制指令。

### 2.5 第六篇--机器人上位机软件设计（激光雷达驱动程序设计篇）
激光雷达作为移动机器人的核心是必不可少的，该机器人采用了一台低成本的单线激光雷达，为了能够采集该雷达的
数据，需要根据产品的开发手册进行相应的程序的编写，在本篇章中将会详细叙述这个过程。

### 2.6 第七篇--机器人上位机算法设计（SLAM算法篇）
本机器人将采用自研的2D激光SLAM算法，该算法让机器人具备构建环境地图并定位的能力。在这一篇章中会详细介绍
SLAM算法的原理以及实现方法。

### 2.7 第八篇--机器人上位机算法设计（路径规划和运动控制算法篇）
在这一部分中将会介绍本机器人上使用的路径规划和运动控制算法原理、仿真和部署方法。

### 2.8 第八篇--机器人上位机算法部署(视觉检测算法的部署和后处理)
由于机器人需要检测陌生人入侵、人员跌倒等异常行为，因此需要在上位机上部署深度学习算法来检测，在这部分中
将介绍如何在X3派上部署视觉检测深度学习算法以及对检测结果的后处理方法。

### 2.9 第九篇--微信小程序开发
为了能在手机上实现远程机器人遥控、实时监测信息查看、接收异常告警信息，需要开发一个微信小程序来展示这些信息。

### 2.10 第十篇--总结和展望
对上述内容的总结，并且展望下一阶段的工作。
