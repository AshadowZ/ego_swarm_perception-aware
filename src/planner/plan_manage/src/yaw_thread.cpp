#include <plan_manage/yaw_thread.h>

void computeYaw(int myArg){
    ros::Rate rate(2); // 以2Hz发点消息说明自己活着
    while(ros::ok())
    {
        // 真的需要开一个线程来算yaw角度吗，如果我在callReboundReplan后面直接加上yaw角的计算。
        // 那么满足效果是不是就直接ok了？
        // 或者说我在callReboundReplan规划完轨迹后置个标志位，轨迹的处理和发布全交由这个线程处理。斯，这个看上去更合理一点。
        cout << "yaw_thread is executing!" << endl;
        rate.sleep();
    }
}

// 只要装填yaw_pts这一项应该就ok了，原来这个yaw_pts是指yaw角B样条轨迹的控制点，我把它当离散的yaw角真值发吧。
// 时间间隔就100ms，由后面进行插值？
void computeYawVel(traj_utils::Bspline& bspline){
    
    // yaw control
    double t_cur;
    double traj_duration_; // 轨迹总时间
    size_t yaw_num; // 离散yaw角的个数
    double yaw = 0;

    // 还是得转换为UniformBspline来计算
    Eigen::MatrixXd pos_pts(3, bspline.pos_pts.size());
    Eigen::VectorXd knots(bspline.knots.size());
    for (size_t i = 0; i < bspline.pos_pts.size(); ++i) // 取出位置控制点
    {
        pos_pts(0, i) = bspline.pos_pts[i].x;
        pos_pts(1, i) = bspline.pos_pts[i].y;
        pos_pts(2, i) = bspline.pos_pts[i].z;
    }
    for (size_t i = 0; i < bspline.knots.size(); ++i) // 取出节点时间信息
    {
        knots(i) = bspline.knots[i];
    }
    UniformBspline pos_traj(pos_pts, bspline.order, 0.1);
    pos_traj.setKnot(knots); // 将轨迹使用UniformBspline类表示
    UniformBspline vel_traj = pos_traj.getDerivative();

    traj_duration_ = pos_traj.getTimeSum();
    
    // 计算yaw角，每100ms计算一个值
    bspline.yaw_dt = 0.1;
    yaw_num = floor(traj_duration_  / bspline.yaw_dt);
    Eigen::Vector3d vel(Eigen::Vector3d::Zero());
    for(size_t i = 0; i < yaw_num; ++i){
        t_cur = i * bspline.yaw_dt;
        vel = vel_traj.evaluateDeBoorT(t_cur);
        yaw = atan2(vel[1], vel[0]); // yaw就是当前速度的切线方向
        bspline.yaw_pts.push_back(yaw);
    }
    cout << "yaw sequence is computed!" << endl;
    cout << "traj_duration: " << traj_duration_ << endl;
    cout << "Number of elements in yaw_pts: " << bspline.yaw_pts.size() << endl;
    cout << "First three elements in yaw_pts: ";
    for (size_t i = 0; i < std::min(bspline.yaw_pts.size(), static_cast<size_t>(3)); ++i) {
        cout << bspline.yaw_pts[i] << " ";
    }
    cout << endl;
}

