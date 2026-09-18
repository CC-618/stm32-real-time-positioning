#ifndef _FILTER_H_
#define _FILTER_H_
 
#define MAX_SENSOR_NUM 9   //使用滤波时的传感器数量
#define MAX_DATA_NUM 9     //最大采样点数量，即采样窗口长度
#define WINDOW_DATA_NUM 5  //滤波窗口长度
//去除采样窗口内最大最小值的数量，这里去除两个最大和两个最小 
#define REMOVE_MAXMIN_NUM ((MAX_DATA_NUM - WINDOW_DATA_NUM)/2)  
 
//extern double m_dataList[MAX_SENSOR_NUM][MAX_DATA_NUM];
 
//声明定义的函数
double Filter_SlidingWindowAvg(int index, double data);
 
#endif

