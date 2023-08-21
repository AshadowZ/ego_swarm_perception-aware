#include <plan_manage/yaw_thread.h>

void computeYaw(int myArg){
    ros::Rate rate(2); // 以2Hz发点消息说明自己活着
    while(ros::ok())
    {
        // 真的需要开一个线程来算yaw角度吗，如果我在callReboundReplan后面直接加上yaw角的计算
        // 那么满足效果是不是就直接ok了？
        // 或者说我在callReboundReplan规划完轨迹后置个标志位，轨迹的处理和发布全交由这个线程处理。斯，这个看上去更合理一点。
        cout << "yaw_thread is executing!" << endl;
        rate.sleep();
    }
}

