#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>

// 添加必要的头文件
#include "common.hpp"

namespace legutils {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

struct JointParam {
  Vector3d p_pj = Vector3d::Zero();
  Vector3d rpy_pj = Vector3d::Zero();
  Vector3d axis = Vector3d(0, 0, 1);
};

struct Chain {
  int n = 0;
  std::vector<Vector3d> p_pj;
  std::vector<Matrix3d> R_pj;
  std::vector<Vector3d> axis_j;
};

struct FKOut {
  std::vector<Matrix3d> R0_joint;
  std::vector<Vector3d> p0_joint;
  std::vector<Matrix3d> R0_link;
  std::vector<Vector3d> p0_link;
  std::vector<Eigen::Matrix<double,6,6>> X_child_parent;
};

struct LegModel {
  std::string leg_name;
  std::vector<std::string> joint_names;
  Chain chain;
  Vector3d foot_offset_in_last_link;
};

// ===== Header-only implementation =====

static inline Matrix3d skew(const Vector3d& v) {
  Matrix3d S;
  S <<    0, -v.z(),  v.y(),
        v.z(),     0, -v.x(),
       -v.y(),  v.x(),    0;
  return S;
}

static inline Matrix3d rpyToR(double r, double p, double y) {
  double cr = std::cos(r), sr = std::sin(r);
  double cp = std::cos(p), sp = std::sin(p);
  double cy = std::cos(y), sy = std::sin(y);
  Matrix3d Rz; Rz << cy,-sy,0, sy,cy,0, 0,0,1;
  Matrix3d Ry; Ry <<  cp,0,sp, 0,1,0, -sp,0,cp;
  Matrix3d Rx; Rx << 1,0,0, 0,cr,-sr, 0,sr,cr;
  return Rz * Ry * Rx;
}

static inline Matrix3d axisAngleToR(const Vector3d& axis_unit, double q) {
  Eigen::AngleAxisd aa(q, axis_unit);
  return aa.toRotationMatrix();
}

struct Spatial {
  static inline Eigen::Matrix<double,6,6> X(const Matrix3d& R_AB, const Vector3d& p_AB) {
    Eigen::Matrix<double,6,6> X; X.setZero();
    X.topLeftCorner<3,3>() = R_AB;
    X.bottomRightCorner<3,3>() = R_AB;
    X.bottomLeftCorner<3,3>() = -R_AB * skew(p_AB);
    return X;
  }
};

inline Chain buildChainFromParams(const std::vector<JointParam>& params) {
  Chain c;
  c.n = static_cast<int>(params.size());
  c.p_pj.resize(c.n);
  c.R_pj.resize(c.n);
  c.axis_j.resize(c.n);
  for (int i = 0; i < c.n; ++i) {
    c.p_pj[i] = params[i].p_pj;
    c.R_pj[i] = rpyToR(params[i].rpy_pj.x(), params[i].rpy_pj.y(), params[i].rpy_pj.z());
    Vector3d a = params[i].axis;
    double an = a.norm();
    if (an == 0) throw std::runtime_error("axis must be non-zero");
    c.axis_j[i] = a / an;
  }
  return c;
}

inline FKOut forwardKinematics(const Chain& c, const VectorXd& q) {
  FKOut out;
  out.R0_joint.resize(c.n);
  out.p0_joint.resize(c.n);
  out.R0_link.resize(c.n);
  out.p0_link.resize(c.n);
  out.X_child_parent.resize(c.n);

  Matrix3d R0_parent = Matrix3d::Identity();
  Vector3d p0_parent = Vector3d::Zero();

  for (int i = 0; i < c.n; ++i) {
    Matrix3d R0_joint = R0_parent * c.R_pj[i];
    Vector3d p0_joint = p0_parent + R0_parent * c.p_pj[i];

    Matrix3d R_jc = axisAngleToR(c.axis_j[i], q[i]);
    Matrix3d R0_child = R0_joint * R_jc;
    Vector3d p0_child = p0_joint;

    out.R0_joint[i] = R0_joint;
    out.p0_joint[i] = p0_joint;
    out.R0_link[i]  = R0_child;
    out.p0_link[i]  = p0_child;

    Matrix3d R_child_parent = R0_child.transpose() * R0_parent;
    Vector3d p_child_parent = R0_child.transpose() * (p0_parent - p0_child);
    out.X_child_parent[i] = Spatial::X(R_child_parent, p_child_parent);

    R0_parent = R0_child;
    p0_parent = p0_child;
  }
  return out;
}

inline MatrixXd geometricJacobian(const Chain& c, const VectorXd& q, const Vector3d& foot_offset = Vector3d::Zero()) {
  FKOut fk = forwardKinematics(c, q);
  int n = c.n;
  MatrixXd J(6, n);
  J.setZero();

  // 计算足端位置
  Vector3d p_foot = fk.p0_link[n-1] + fk.R0_link[n-1] * foot_offset;

  // 计算雅可比矩阵
  for (int i = 0; i < n; ++i) {
    Vector3d axis_world = fk.R0_joint[i] * c.axis_j[i];
    Vector3d r = p_foot - fk.p0_joint[i];
    J.block<3,1>(0, i) = axis_world.cross(r);  // 线速度部分
    J.block<3,1>(3, i) = axis_world;           // 角速度部分
  }
  return J;
}

inline Vector3d footPosition(const Chain& c, const VectorXd& q, const Vector3d& foot_offset = Vector3d::Zero()) {
  FKOut fk = forwardKinematics(c, q);
  return fk.p0_link[c.n-1] + fk.R0_link[c.n-1] * foot_offset;
}

inline Vector3d footVelocity(const Chain& c, const VectorXd& q, const VectorXd& dq, const Vector3d& foot_offset = Vector3d::Zero()) {
  MatrixXd J = geometricJacobian(c, q, foot_offset);
  return J.block<3,3>(0, 0) * dq;  // 只取线速度部分
}

// ===== Contact detection (sigmoid-based with sliding window) =====

// 基于sigmoid分类器的接触检测算法
// 总得分 = Σ(权重 × 特征得分) + 偏置
// 接触概率 = sigmoid(总得分)，接触状态 = (概率 >= 决策阈值)
struct ContactSigmoidParams {
  // 阈值参数
  double knee_torque_threshold_Nm = 10.0;    // [Nm] 膝关节扭矩阈值
  double foot_height_target_m = -0.4;        // [m] 足端目标高度
  double foot_height_tolerance_m = 0.05;     // [m] 足端高度容差
  double knee_torque_change_scale_Nm = 5.0;  // [Nm] 扭矩变化检测尺度
  double foot_vx_contact_threshold_mps = 0.1; // [m/s] 足端x方向速度阈值
  double foot_vy_contact_threshold_mps = 0.1; // [m/s] 足端y方向速度阈值

  // 权重参数
  double weight_torque = 10.0;               // 扭矩特征权重
  double weight_speed = 1.0;                 // 速度特征权重
  double weight_height = 1.5;                // 高度特征权重
  double weight_torque_change = 0.1;         // 扭矩变化特征权重
  double weight_foot_vx = 2.0;               // 足端x方向速度特征权重
  double weight_foot_vy = 2.0;               // 足端y方向速度特征权重

  // 分类器参数
  double bias = 0.0;                         // 偏置项
  double decision_threshold = 0.5;           // 决策阈值

  // 其他参数
  int sliding_window_size = 6;               // 滑动窗口大小
  double state_hold_time_s = 0.1;            // 状态保持时间 [s]
};

inline double _sigmoid(double x) {
  if (x >= 0.0) {
    double ex = std::exp(-x);
    return 1.0 / (1.0 + ex);
  } else {
    double ex = std::exp(x);
    return ex / (1.0 + ex);
  }
}

inline double _saturate(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Sliding window for storing torque history
class TorqueSlidingWindow {
public:
  TorqueSlidingWindow() : window_size_(6), torques_(6, 0.0), index_(0), count_(0) {}
  explicit TorqueSlidingWindow(int size) : window_size_(size), torques_(size, 0.0), index_(0), count_(0) {}

  void add(double torque) {
    torques_[index_] = torque;
    index_ = (index_ + 1) % window_size_;
    if (count_ < window_size_) count_++;
  }

  double getMin() const {
    if (count_ == 0) return 0.0;
    double min_val = torques_[0];
    for (int i = 1; i < count_; ++i) {
      if (torques_[i] < min_val) min_val = torques_[i];
    }
    return min_val;
  }

  int getCount() const { return count_; }

private:
  int window_size_;
  std::vector<double> torques_;
  int index_;
  int count_;
};

// 接触检测状态机
// 用于避免接触和离地状态之间的反复横跳
class ContactStateMachine {
public:
  enum class State {
    UNKNOWN,    // 未知状态
    CONTACT,    // 接触状态
    NO_CONTACT  // 非接触状态
  };

  ContactStateMachine() : current_state_(State::UNKNOWN), last_state_change_time_(0.0) {}

  // 更新状态机
  // 输入：
  //   current_time: 当前时间 [s]
  //   raw_contact: 原始接触检测结果
  //   hold_time: 状态保持时间 [s]
  // 返回：经过状态机处理后的接触状态
  bool update(double current_time, bool raw_contact, double hold_time) {
    State target_state = raw_contact ? State::CONTACT : State::NO_CONTACT;
    
    // 如果目标状态与当前状态不同，且已经过了保持时间，则允许状态切换
    if (target_state != current_state_ && 
        (current_time - last_state_change_time_) >= hold_time) {
      current_state_ = target_state;
      last_state_change_time_ = current_time;
    }
    
    return (current_state_ == State::CONTACT);
  }

  // 获取当前状态
  State getState() const { return current_state_; }
  
  // 获取状态保持时间
  double getStateHoldTime() const { return last_state_change_time_; }
  
  // 重置状态机
  void reset() {
    current_state_ = State::UNKNOWN;
    last_state_change_time_ = 0.0;
  }

private:
  State current_state_;
  double last_state_change_time_;
};

// 接触检测得分结构体
struct ContactScores {
  double f_torque;        // 扭矩特征得分 [0,1]
  double f_speed;         // 速度特征得分 [0,1]
  double f_height;        // 高度特征得分 [0,1]
  double f_torque_change; // 扭矩变化特征得分 [0,1]
  double f_foot_vx;       // 足端x方向速度特征得分 [0,1]
  double f_foot_vy;       // 足端y方向速度特征得分 [0,1]
  double total_score;     // 总得分
  double probability;     // 最终概率 [0,1]
  bool raw_contact;       // 原始接触检测结果
  bool filtered_contact;  // 状态机过滤后的结果
  ContactStateMachine::State state_machine_state;  // 状态机状态
};

// 返回接触概率和各项得分
inline ContactScores contactProbabilitySigmoidDetailed(double knee_tau,
                                                       double knee_dq,
                                                       double foot_z_world,
                                                       double foot_vx_world,
                                                       double foot_vy_world,
                                                       TorqueSlidingWindow& torque_window,
                                                       ContactStateMachine& state_machine,
                                                       double current_time,                                                  
                                                       const ContactSigmoidParams& p = ContactSigmoidParams(),
                                                       double foot_vxd_world = 0.3,
                                                       double foot_vyd_world = 0.0) {
  ContactScores scores;
  
  // Add current torque to sliding window
  torque_window.add(std::fabs(knee_tau));

  // 计算特征得分
  scores.f_torque = (knee_tau > 0) ? _sigmoid(knee_tau - p.knee_torque_threshold_Nm) : 0.0;
  scores.f_speed = _sigmoid(-std::fabs(knee_dq));
  scores.f_height = _sigmoid(-std::fabs(foot_z_world - p.foot_height_target_m) / p.foot_height_tolerance_m);

  // 足端速度得分（x方向和y方向均采用目标速度与实际速度的比较方式）
  if (std::fabs(foot_vxd_world) > 1e-6) {
    double vx_diff = (foot_vxd_world > 0) ? 0.9 * foot_vxd_world - foot_vx_world : foot_vx_world - 0.9 * foot_vxd_world;
    scores.f_foot_vx = _sigmoid(vx_diff);
  } else {
    scores.f_foot_vx = 0.0;
  }

  if (std::fabs(foot_vyd_world) > 1e-6) {
    double vy_diff = (foot_vyd_world > 0) ? 0.9 * foot_vyd_world - foot_vy_world : foot_vy_world - 0.9 * foot_vyd_world;
    scores.f_foot_vy = _sigmoid(vy_diff);
  } else {
    scores.f_foot_vy = 0.0;
  }

  // 扭矩变化特征
  scores.f_torque_change = (torque_window.getCount() >= p.sliding_window_size) ? 
    _sigmoid(-(std::fabs(knee_tau) - torque_window.getMin()) / p.knee_torque_change_scale_Nm) : 0.0;

  // 计算总得分和概率
  scores.total_score = p.weight_torque * scores.f_torque + p.weight_speed * scores.f_speed + 
                      p.weight_height * scores.f_height + p.weight_torque_change * scores.f_torque_change +
                      p.weight_foot_vx * scores.f_foot_vx + p.weight_foot_vy * scores.f_foot_vy + p.bias;
  scores.probability = _sigmoid(scores.total_score);
  
  // 状态机处理
  scores.raw_contact = (scores.probability >= p.decision_threshold);
  scores.filtered_contact = state_machine.update(current_time, scores.raw_contact, p.state_hold_time_s);
  scores.state_machine_state = state_machine.getState();
  
  return scores;
}

// 返回接触概率（0~1）
inline double contactProbabilitySigmoid(double knee_tau,
                                        double knee_dq,
                                        double foot_z_world,
                                        double foot_vx_world,
                                        double foot_vy_world,
                                        TorqueSlidingWindow& torque_window,
                                        ContactStateMachine& state_machine,
                                        double current_time,
                                        double foot_vxd_world = 0.0,
                                        double foot_vyd_world = 0.0,
                                        const ContactSigmoidParams& p = ContactSigmoidParams()) {
  return contactProbabilitySigmoidDetailed(knee_tau, knee_dq, foot_z_world, foot_vx_world, foot_vy_world, torque_window, state_machine, current_time, p, foot_vxd_world, foot_vyd_world).probability;
}

// 返回是否接触（概率与阈值比较）
inline bool detectContactSigmoid(double knee_tau,
                                 double knee_dq,
                                 double foot_z_world,
                                 double foot_vx_world,
                                 double foot_vy_world,
                                 TorqueSlidingWindow& torque_window,
                                 ContactStateMachine& state_machine,
                                 double current_time,
                                 double foot_vxd_world = 0.0,
                                 double foot_vyd_world = 0.0,
                                 const ContactSigmoidParams& p = ContactSigmoidParams()) {
  const double prob = contactProbabilitySigmoid(knee_tau, knee_dq, foot_z_world, foot_vx_world, foot_vy_world, torque_window, state_machine, current_time, foot_vxd_world, foot_vyd_world, p);
  return prob >= p.decision_threshold;
}

// ===== Robot model creation =====

inline std::map<std::string, LegModel> createGRQ20Legs() {
  std::map<std::string, LegModel> legs;
  
  // 创建四条腿
  std::vector<std::string> leg_names = {"FR", "FL", "RR", "RL"};
  std::vector<std::vector<std::string>> joint_names = {
    {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"},
    {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"},
    {"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"},
    {"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}
  };
  
  // 为每条腿设置正确的参数
  for (int i = 0; i < 4; ++i) {
    std::vector<JointParam> p(3);
    const std::string& leg = leg_names[i];
    
    // 根据腿的位置设置参数
    double hip_x = (leg == "FR" || leg == "FL") ? 0.279 : -0.279;  // 前腿为正，后腿为负
    double hip_y = (leg == "FL" || leg == "RL") ? 0.067 : -0.067;  // 左腿为正，右腿为负
    double thigh_y = (leg == "FL" || leg == "RL") ? 0.1103 : -0.1103;  // 左腿为正，右腿为负
    
    // 髋关节
    p[0].p_pj = Vector3d(hip_x, hip_y, 0.0);
    p[0].rpy_pj = Vector3d(0, 0, 0);
    p[0].axis = Vector3d(1, 0, 0);  // 绕X轴旋转
    
    // 大腿关节
    p[1].p_pj = Vector3d(0.0, thigh_y, 0.0);
    p[1].rpy_pj = Vector3d(0, 0, 0);
    p[1].axis = Vector3d(0, 1, 0);  // 绕Y轴旋转
    
    // 小腿关节
    p[2].p_pj = Vector3d(0.0, 0.0, -0.26);
    p[2].rpy_pj = Vector3d(0, 0, 0);
    p[2].axis = Vector3d(0, 1, 0);  // 绕Y轴旋转
    
    // 足端偏移
    Vector3d foot_off(0.0, 0.0, -0.26);
    
    LegModel lm;
    lm.leg_name = leg;
    lm.joint_names = joint_names[i];
    lm.chain = buildChainFromParams(p);
    lm.foot_offset_in_last_link = foot_off;
    legs[leg] = lm;
  }
  
  return legs;
}

// ===== Processing function for KinImuMeas =====

// 简洁的处理函数：直接填充已有的足端位置、速度和IMU信息，并进行坐标转换
inline void processing(double aligned_timestamp,                    // 对齐后的时间戳
                      const std::vector<Vector3d>& foot_positions,  // 4条腿的足端位置（机器人本体坐标系）
                      const std::vector<Vector3d>& foot_velocities, // 4条腿的足端速度（机器人本体坐标系）
                      const Vector3d& imu_acc,                      // IMU加速度
                      const Vector3d& imu_gyr,                      // IMU角速度
                      const std::vector<bool>& contacts,            // 4条腿的接触状态
                      galileo_klio::common::KinImuMeas& kin_imu_meas,
                      const Vector3d& extrinsic_T = Vector3d::Zero(),           // IMU到激光雷达的平移向量
                      const Matrix3d& extrinsic_R = Matrix3d::Identity()) {     // IMU到激光雷达的旋转矩阵
  
  // 设置时间戳
  kin_imu_meas.time_stamp_ = aligned_timestamp;
  
  // 设置IMU数据
  for (int i = 0; i < 3; ++i) {
    kin_imu_meas.acc_[i] = imu_acc[i];
    kin_imu_meas.gyr_[i] = imu_gyr[i];
  }
  
  // 坐标转换矩阵：机器人本体坐标系到激光雷达坐标系（绕Y轴旋转90°）
  // 使用Eigen的欧拉角方法构建旋转矩阵
  // 绕Y轴旋转90度（pitch = 90°）
  Matrix3d body_to_lidar_rot = (Eigen::AngleAxisd(0.0, Vector3d::UnitX()) *      // roll = 0°
                                 Eigen::AngleAxisd(M_PI / 2.0, Vector3d::UnitY()) *  // pitch = 90°
                                 Eigen::AngleAxisd(0.0, Vector3d::UnitZ())).toRotationMatrix().transpose() ;  // yaw = 0°
  
  // 激光雷达到IMU的转换矩阵（外参的逆变换）
  Matrix3d lidar_to_imu_rot = extrinsic_R;
  Vector3d lidar_to_imu_t = -lidar_to_imu_rot * extrinsic_T;
  
  // 设置足端位置、速度和接触状态（应用坐标转换）
  for (int leg_idx = 0; leg_idx < 4; ++leg_idx) {
    // 1. 机器人本体坐标系 -> 激光雷达坐标系
    Vector3d foot_pos_lidar = body_to_lidar_rot* foot_positions[leg_idx];
    Vector3d foot_vel_lidar = body_to_lidar_rot* foot_velocities[leg_idx];
    
    // 2. 激光雷达坐标系 -> IMU坐标系
    Vector3d foot_pos_imu = lidar_to_imu_rot * foot_pos_lidar + lidar_to_imu_t;
    Vector3d foot_vel_imu = lidar_to_imu_rot * foot_vel_lidar;
    
    // 存储转换后的数据
    for (int i = 0; i < 3; ++i) {
      kin_imu_meas.foot_pos_[leg_idx][i] = foot_pos_imu[i];
      kin_imu_meas.foot_vel_[leg_idx][i] = foot_vel_imu[i];
    }
    kin_imu_meas.contact_[leg_idx] = contacts[leg_idx];
  }
}

} // namespace legutils
