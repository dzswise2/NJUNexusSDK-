#!/usr/bin/env python3
"""One-shot refactor: insert ArmController::Impl and add impl_-> prefixes."""

from pathlib import Path
import re

CPP = Path(__file__).resolve().parent.parent / "src/robot_controller/src/controllers/arm_controller.cpp"

IMPL_STRUCT = r'''
struct ArmController::Impl {
    static constexpr int EE_FF_DIM = 6;
    enum class EEForceFeedbackState { FREE, HOLDING };

    ControllerParams params_;
    ControllerStatus status_{ControllerStatus::UNINITIALIZED};

    std::unique_ptr<RobotModel> robot_model_;
    std::unique_ptr<pinocchio::Data> force_est_data_;
    std::unique_ptr<GripperController> gripper_controller_;
    std::unique_ptr<PIDController> joint_pid_controller_;

    Eigen::VectorXd gravity_comp_kp_;
    Eigen::VectorXd gravity_comp_kd_;
    Eigen::VectorXd motion_planning_kp_;
    Eigen::VectorXd motion_planning_kd_;
    Eigen::VectorXd damping_control_kp_;
    Eigen::VectorXd damping_control_kd_;

    JointState initial_joint_state_;

    double planning_percent_{-1.0};
    std::vector<double> planning_start_positions_;
    std::vector<double> last_s_curve_target_;
    std::vector<double> last_polynomial_target_;

    std::unique_ptr<SCurveProfile> s_curve_profile_{nullptr};
    std::unique_ptr<PolynomialProfile> polynomial_profile_{nullptr};

    bool spring_center_initialized_{false};
    std::vector<double> spring_center_q_;

    bool enable_cbf_{false};
    OSCBF::OSCBFController::Config cbf_config_;
    OSCBF::OSCBFController::ObstacleConfig default_obstacle_config_;
    std::shared_ptr<OSCBF::CollisionPairManager> collision_pair_manager_;
    std::unique_ptr<OSCBF::OSCBFController> cbf_controller_;
    std::unique_ptr<RobotModel> robot_model_gripper_locked_;

    bool enable_torque_cbf_{false};
    OSCBF::torque_cbf::OSCBFController::Config torque_cbf_config_;
    OSCBF::torque_cbf::OSCBFController::ObstacleConfig torque_cbf_obstacle_config_;
    std::shared_ptr<OSCBF::torque_cbf::TaskSpaceTorqueConstraint> torque_task_space_constraint_;
    std::shared_ptr<OSCBF::torque_cbf::JointLimitTorqueConstraint> torque_joint_limit_constraint_;
    std::shared_ptr<OSCBF::torque_cbf::JointVelocityTorqueConstraint> torque_joint_velocity_constraint_;
    std::shared_ptr<OSCBF::torque_cbf::SelfCollisionTorqueConstraint> torque_self_collision_constraint_;
    std::unique_ptr<OSCBF::torque_cbf::OSCBFController> torque_cbf_controller_;

    Eigen::Matrix4d pose_target_cbf_{Eigen::Matrix4d::Identity()};
    Eigen::VectorXd q_target;
    Eigen::VectorXd v_target;

    bool pose_target_cbf_initialized_{false};
    bool first_call_cbf_{true};

    double last_filtered_gripper_target_angle_{0.0};
    bool gripper_filter_initialized_{false};
    double gripper_filter_alpha_{0.2};

    RedundantArmIkRuntimeState redundant_arm_ik_runtime_;
    PostCbfJointReferenceFilter post_cbf_joint_ref_filter_;

    Eigen::VectorXd friction_coulomb_{Eigen::VectorXd::Zero(0)};
    Eigen::VectorXd filtered_velocity_{Eigen::VectorXd::Zero(0)};
    bool velocity_filter_initialized_{false};
    double velocity_filter_alpha_{0.485};
    double friction_velocity_threshold_{0.01};
    std::vector<std::deque<int>> velocity_sign_history_;

    double current_control_time_sec_{0.0};

    Eigen::VectorXd last_cart_pose_error_{Eigen::VectorXd::Zero(6)};
    Eigen::VectorXd last_pose_ee_updated_{Eigen::VectorXd::Zero(6)};
    Eigen::VectorXd last_desired_pose_{Eigen::VectorXd::Zero(6)};
    Eigen::VectorXd last_ik_target_pose_xyz_rpy_{Eigen::VectorXd::Zero(6)};
    Eigen::VectorXd last_joint_pos_error_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_T_des_xyz_rpy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_q_des_ik_ns_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_fk_q_des_xyz_rpy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_q_target_after_cbf_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_fk_q_target_xyz_rpy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_q_meas_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_joint_pos_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_ee_pos_track_err_pub_;

    int velocity_sign_history_size_{10};
    double m_t{0.0};
    Eigen::Matrix4d desired_pose_init;

    Eigen::VectorXd freedrive_hold_q_;
    bool freedrive_hold_initialized_{false};

    mutable std::array<EEForceFeedbackState, EE_FF_DIM> ee_ff_state_per_axis_{};
    mutable std::array<double, EE_FF_DIM> ee_ff_state_enter_ts_{};
    mutable std::array<double, EE_FF_DIM> ee_ff_hold_candidate_ts_{{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0}};
    mutable std::array<double, EE_FF_DIM> ee_ff_free_candidate_ts_{{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0}};
    mutable double ee_ff_last_eval_ts_{0.0};
    mutable bool ee_ff_have_last_eval_{false};

    Eigen::MatrixXd computeDoubleDiagDampingMatrix(
        const Eigen::MatrixXd& K,
        const Eigen::VectorXd& zeta,
        const Eigen::MatrixXd& Lambda_inv);

    JointState expandJointStateForGripper(const JointState& joint_state) const;

    bool loadFrictionCoulombFromFile(const std::string& file_path, int expected_dof);

    void publishRedundantArmDebug(const Eigen::VectorXd& T_des_xyz_rpy6,
                                  const Eigen::VectorXd& q_des_ik_ns,
                                  const Eigen::VectorXd& fk_q_des_xyz_rpy6,
                                  const Eigen::VectorXd& q_target_after_cbf,
                                  const Eigen::VectorXd& fk_q_target_xyz_rpy6,
                                  const Eigen::VectorXd& q_meas,
                                  const Eigen::VectorXd& joint_pos_error);

    Eigen::VectorXd computeFrictionCompensation(
        const Eigen::VectorXd& current_v,
        int model_dof,
        double position_error_norm = -1.0);

    void updateFilteredVelocity(const Eigen::VectorXd& current_v, int model_dof);
};

'''

MEMBERS = [
    "debug_ee_pos_track_err_pub_",
    "debug_joint_pos_error_pub_",
    "debug_q_meas_pub_",
    "debug_fk_q_target_xyz_rpy_pub_",
    "debug_q_target_after_cbf_pub_",
    "debug_fk_q_des_xyz_rpy_pub_",
    "debug_q_des_ik_ns_pub_",
    "debug_T_des_xyz_rpy_pub_",
    "last_ik_target_pose_xyz_rpy_",
    "last_joint_pos_error_",
    "current_control_time_sec_",
    "velocity_sign_history_",
    "friction_velocity_threshold_",
    "velocity_filter_alpha_",
    "velocity_filter_initialized_",
    "filtered_velocity_",
    "friction_coulomb_",
    "post_cbf_joint_ref_filter_",
    "redundant_arm_ik_runtime_",
    "gripper_filter_alpha_",
    "gripper_filter_initialized_",
    "last_filtered_gripper_target_angle_",
    "torque_self_collision_constraint_",
    "torque_joint_velocity_constraint_",
    "torque_joint_limit_constraint_",
    "torque_task_space_constraint_",
    "torque_cbf_obstacle_config_",
    "robot_model_gripper_locked_",
    "collision_pair_manager_",
    "default_obstacle_config_",
    "polynomial_profile_",
    "planning_start_positions_",
    "last_polynomial_target_",
    "last_s_curve_target_",
    "spring_center_initialized_",
    "spring_center_q_",
    "joint_pid_controller_",
    "force_est_data_",
    "motion_planning_kd_",
    "motion_planning_kp_",
    "damping_control_kd_",
    "damping_control_kp_",
    "gravity_comp_kd_",
    "gravity_comp_kp_",
    "gripper_controller_",
    "cbf_controller_",
    "torque_cbf_controller_",
    "s_curve_profile_",
    "pose_target_cbf_initialized_",
    "first_call_cbf_",
    "desired_pose_init",
    "last_cart_pose_error_",
    "last_pose_ee_updated_",
    "last_desired_pose_",
    "ee_ff_have_last_eval_",
    "ee_ff_last_eval_ts_",
    "ee_ff_free_candidate_ts_",
    "ee_ff_hold_candidate_ts_",
    "ee_ff_state_enter_ts_",
    "ee_ff_state_per_axis_",
    "freedrive_hold_initialized_",
    "freedrive_hold_q_",
    "velocity_sign_history_size_",
    "initial_joint_state_",
    "planning_percent_",
    "robot_model_",
    "cbf_config_",
    "torque_cbf_config_",
    "enable_torque_cbf_",
    "enable_cbf_",
    "pose_target_cbf_",
    "status_",
    "params_",
    "m_t",
    "q_target",
    "v_target",
]


def match_brace(s: str, open_idx: int) -> int:
    depth = 0
    i = open_idx
    while i < len(s):
        c = s[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    raise RuntimeError("unbalanced brace")


def add_impl_prefix(text: str) -> str:
    for m in MEMBERS:
        text = re.sub(r"\b" + re.escape(m) + r"\b", "impl_->" + m, text)
    return text


def transform_modified_regions(text: str) -> str:
    text = add_impl_prefix(text)
    repl_calls = [
        ("computeDoubleDiagDampingMatrix(", "impl_->computeDoubleDiagDampingMatrix("),
        ("expandJointStateForGripper(", "impl_->expandJointStateForGripper("),
        ("loadFrictionCoulombFromFile(", "impl_->loadFrictionCoulombFromFile("),
        ("publishRedundantArmDebug(", "impl_->publishRedundantArmDebug("),
        ("computeFrictionCompensation(", "impl_->computeFrictionCompensation("),
        ("updateFilteredVelocity(", "impl_->updateFilteredVelocity("),
    ]
    for a, b in repl_calls:
        text = text.replace(a, b)
    text = text.replace("impl_->impl_->", "impl_->")
    text = re.sub(
        r"\bfor \(int i = 0; i < impl_->EE_FF_DIM",
        "for (int i = 0; i < Impl::EE_FF_DIM",
        text,
    )
    text = text.replace("for (int i = 0; i < EE_FF_DIM;", "for (int i = 0; i < Impl::EE_FF_DIM;")
    text = text.replace("EEForceFeedbackState::", "Impl::EEForceFeedbackState::")
    text = re.sub(r"(?<!Impl::)\bEEForceFeedbackState\b(?!::)", "Impl::EEForceFeedbackState", text)
    text = text.replace("Impl::Impl::", "Impl::")
    return text


def main() -> None:
    s = CPP.read_text(encoding="utf-8")

    marker = "}  // namespace\n\n"
    idx = s.find(marker)
    if idx == -1:
        raise SystemExit("marker not found")
    insert_at = idx + len(marker)
    s = s[:insert_at] + IMPL_STRUCT + "\n" + s[insert_at:]

    cm_start = s.find("MitControlCommand ArmController::computeCartMotionControl(")
    cm_end = s.find("MitControlCommand ArmController::computeCartMotionControlWithCBFinSimRobot(", cm_start)
    block = s[cm_start:cm_end]
    block = block.replace("Eigen::VectorXd q_target;", "Eigen::VectorXd q_cart_ik;")
    block = re.sub(r"\bq_target\b", "q_cart_ik", block)
    s = s[:cm_start] + block + s[cm_end:]

    jsp_start = s.find("MotionPlanningResult ArmController::computeJointSpaceMotionPlanning(")
    jsp_next = s.find("MotionPlanningResult ArmController::computeJointSpaceMotionPlanningInterpolate(", jsp_start)
    jsp_block = s[jsp_start:jsp_next]
    inner_start = jsp_block.find("// 构建目标位置向量")
    if inner_start != -1:
        tail = jsp_block[inner_start:]
        tail = tail.replace(
            "Eigen::VectorXd q_target = Eigen::VectorXd::Zero(model_dof);",
            "Eigen::VectorXd q_jsp_goal = Eigen::VectorXd::Zero(model_dof);",
        )
        tail = re.sub(r"\bq_target\b", "q_jsp_goal", tail)
        jsp_block = jsp_block[:inner_start] + tail
    s = s[:jsp_start] + jsp_block + s[jsp_next:]

    old_ctor = (
        "ArmController::ArmController() \n"
        "    : status_(ControllerStatus::UNINITIALIZED), \n"
        "      planning_percent_(-1.0), \n"
        "      s_curve_profile_(nullptr),\n"
        "      polynomial_profile_(nullptr),\n"
        "      pose_target_cbf_(Eigen::Matrix4d::Identity()),\n"
        "      pose_target_cbf_initialized_(false),\n"
        "      last_filtered_gripper_target_angle_(0.0),\n"
        "      gripper_filter_initialized_(false),\n"
        "      gripper_filter_alpha_(0.2),  // 默认滤波系数 0.2\n"
        "      friction_coulomb_(Eigen::VectorXd::Zero(0)),  // 初始化为空向量，在initialize中设置大小\n"
        "      filtered_velocity_(Eigen::VectorXd::Zero(0)),  // 初始化为空向量，在initialize中设置大小\n"
        "      velocity_filter_initialized_(false),\n"
        "      velocity_filter_alpha_(0.485),  // 默认滤波系数，对应截止频率75Hz（控制频率500Hz时）\n"
        "      friction_velocity_threshold_(0.01),  // 默认速度阈值 0.01 rad/s，避免在速度很小时应用摩擦补偿\n"
        "      velocity_sign_history_size_(10),  // 默认历史队列大小10，将在initialize中从配置读取\n"
        "      last_cart_pose_error_(Eigen::VectorXd::Zero(6)),  // 初始化为6维零向量\n"
        "      last_pose_ee_updated_(Eigen::VectorXd::Zero(6)),  // 初始化为6维零向量（x, y, z, roll, pitch, yaw）\n"
        "      last_desired_pose_(Eigen::VectorXd::Zero(6)),      // 初始化为6维零向量（x, y, z, roll, pitch, yaw）\n"
        "      last_ik_target_pose_xyz_rpy_(Eigen::VectorXd::Zero(6)) {\n"
        "}"
    )
    if old_ctor not in s:
        raise SystemExit("constructor pattern not found")
    s = s.replace(old_ctor, "ArmController::ArmController()\n    : impl_(std::make_unique<Impl>()) {\n}\n", 1)

    repl_sig = [
        ("Eigen::MatrixXd ArmController::computeDoubleDiagDampingMatrix(", "Eigen::MatrixXd ArmController::Impl::computeDoubleDiagDampingMatrix("),
        ("JointState ArmController::expandJointStateForGripper(", "JointState ArmController::Impl::expandJointStateForGripper("),
        ("bool ArmController::loadFrictionCoulombFromFile(", "bool ArmController::Impl::loadFrictionCoulombFromFile("),
        ("void ArmController::publishRedundantArmDebug(", "void ArmController::Impl::publishRedundantArmDebug("),
        ("Eigen::VectorXd ArmController::computeFrictionCompensation(", "Eigen::VectorXd ArmController::Impl::computeFrictionCompensation("),
        ("void ArmController::updateFilteredVelocity(", "void ArmController::Impl::updateFilteredVelocity("),
    ]
    for a, b in repl_sig:
        s = s.replace(a, b)

    struct_begin = s.find("struct ArmController::Impl {")
    if struct_begin == -1:
        raise SystemExit("struct not found")
    open_br = s.find("{", struct_begin)
    close_br = match_brace(s, open_br)
    struct_end = close_br + 1
    while struct_end < len(s) and s[struct_end] in " \t":
        struct_end += 1
    if struct_end < len(s) and s[struct_end] == ";":
        struct_end += 1
    while struct_end < len(s) and s[struct_end] in "\n\r":
        struct_end += 1

    skip = [(struct_begin, struct_end)]
    search_from = struct_end
    while True:
        p = s.find("ArmController::Impl::", search_from)
        if p == -1:
            break
        ob = s.find("{", p)
        if ob == -1:
            break
        ce = match_brace(s, ob)
        skip.append((ob, ce + 1))
        search_from = ce + 1

    skip.sort()
    out = []
    cur = 0
    for a, b in skip:
        if cur < a:
            out.append(transform_modified_regions(s[cur:a]))
        out.append(s[a:b])
        cur = b
    if cur < len(s):
        out.append(transform_modified_regions(s[cur:]))
    s = "".join(out)

    CPP.write_text(s, encoding="utf-8")
    print("OK:", CPP)


if __name__ == "__main__":
    main()
