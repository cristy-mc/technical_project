#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// ================================================================
// KDL
// ================================================================

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

// ================================================================
// Eigen
// ================================================================

#include <Eigen/Dense>

using namespace std::chrono_literals;


class Iiwa2PickKdlNode : public rclcpp::Node
{
public:

  struct ToolPose
  {
    double x;
    double y;
    double z;

    double roll;
    double pitch;
    double yaw;
  };


  Iiwa2PickKdlNode()
  : Node("iiwa2_pick_kdl_node"),
    busy_(false),
    has_kdl_model_(false),
    has_joint_state_(false),
    ready_(false)
  {
    joint_names_ = {
      "iiwa2_joint_a1",
      "iiwa2_joint_a2",
      "iiwa2_joint_a3",
      "iiwa2_joint_a4",
      "iiwa2_joint_a5",
      "iiwa2_joint_a6",
      "iiwa2_joint_a7"
    };

    // Deve corrispondere esattamente a initial_positions nel config di
    // avvio (identica a quella usata da iiwa1_pick_node).
    initial_conf_ = { 0.0, 0.06, 0.0, -0.95, 0.0, 0.52, 0.0 };
    home_conf_ = { 0.625, 0.713, 0.240, -0.487, -0.112, 0.487, 0.0 };

    // a1 = -1.5708 rad = -90 gradi esatti (verificato con ik_test --fk)
    rover_home_conf = { -1.5708, 0.40, 0.0, -0.80, 0.0, 0.80, 0.0 };

    // Coordinate RELATIVE a iiwa2_link_0.
    tool_poses_["scissors"] = { 0.80, -0.20, 0.48, 0.0, 0.0,  1.5708 };
    tool_poses_["scalpel"]  = { 0.80,  0.00, 0.48, 0.0, 0.0,  1.5708 };
    tool_poses_["forceps"]  = { 0.83,  0.20, 0.48, 0.0, 0.0, -1.5708 };

    arm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/iiwa2/iiwa_arm_controller/commands", 10);

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/iiwa2/pick_status", 10);

    attach_pubs_["scalpel"]  = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/picker/scalpel/attach", 10);
    attach_pubs_["scissors"] = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/picker/scissors/attach", 10);
    attach_pubs_["forceps"]  = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/picker/forceps/attach", 10);

    detach_pubs_["scalpel"]  = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/picker/scalpel/detach", 10);
    detach_pubs_["scissors"] = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/picker/scissors/detach", 10);
    detach_pubs_["forceps"]  = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/picker/forceps/detach", 10);

    user_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/object_request", 10,
      std::bind(&Iiwa2PickKdlNode::userRequestCallback, this, std::placeholders::_1));

    vision_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/recognized_object", 10,
      std::bind(&Iiwa2PickKdlNode::visionCallback, this, std::placeholders::_1));

    direct_pick_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/iiwa2/pick_command", 10,
      std::bind(&Iiwa2PickKdlNode::directPickCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/iiwa2/joint_states", 10,
      std::bind(&Iiwa2PickKdlNode::jointStateCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("robot_description", "");
    std::string robot_description;
    this->get_parameter("robot_description", robot_description);

    if (!robot_description.empty())
    {
      initKDL(robot_description);
    }
    else
    {
      const auto qos = rclcpp::QoS(1).transient_local().reliable();

      robot_desc_global_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description", qos,
        std::bind(&Iiwa2PickKdlNode::robotDescriptionCallback, this, std::placeholders::_1));

      robot_desc_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/iiwa2/robot_description", qos,
        std::bind(&Iiwa2PickKdlNode::robotDescriptionCallback, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "In attesa di robot_description...");
    }

    startup_timer_ = this->create_wall_timer(
      3s, std::bind(&Iiwa2PickKdlNode::performInitialDetach, this));

    RCLCPP_INFO(this->get_logger(), "============================================");
    RCLCPP_INFO(this->get_logger(), "IIWA2 PICK KDL NODE AVVIATO");
    RCLCPP_INFO(this->get_logger(), "IK base: iiwa2_link_0");
    RCLCPP_INFO(this->get_logger(), "IK tip : iiwa2_tool0");
    RCLCPP_INFO(this->get_logger(), "============================================");
  }


private:

  void performInitialDetach()
  {
    startup_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Eseguo detach iniziale di tutti gli strumenti...");

    std_msgs::msg::Empty empty_msg;

    for (auto & kv : detach_pubs_)
    {
      for (int i = 0; i < 15; ++i)
      {
        kv.second->publish(empty_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      RCLCPP_INFO(this->get_logger(), "Detach inviato per: %s", kv.first.c_str());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ready_ = true;
    RCLCPP_INFO(this->get_logger(), "Nodo pronto per ricevere comandi di pick.");
    publishStatus("idle");
  }

  std::string normalize(std::string s)
  {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    std::transform(s.begin(), s.end(), s.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
  }

  bool validTool(const std::string & tool)
  {
    return tool == "scalpel" || tool == "scissors" || tool == "forceps";
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    bool all_found = true;
    for (size_t i = 0; i < 7; ++i)
    {
      bool found = false;
      for (size_t j = 0; j < msg->name.size(); ++j)
      {
        if (msg->name[j] == joint_names_[i])
        {
          if (j < msg->position.size())
          {
            current_joint_positions_[i] = msg->position[j];
            found = true;
          }
          break;
        }
      }
      if (!found) all_found = false;
    }
    if (all_found) has_joint_state_ = true;
  }

  bool getCurrentJointState(std::vector<double> & q)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (!has_joint_state_) return false;
    q = current_joint_positions_;
    return q.size() == 7;
  }

  // ----------------------------------------------------------------
  // Ritorna la posizione REALE del braccio se disponibile, altrimenti
  // il fallback nominale. Usata come punto di partenza di ogni
  // moveToConfiguration, cosi' un eventuale scostamento residuo da
  // una fase precedente (timeout, inerzia) non si accumula: la fase
  // successiva riparte da dove il braccio si trova davvero, non da
  // dove avrebbe dovuto trovarsi in teoria.
  // ----------------------------------------------------------------
  std::vector<double> getStartPoint(const std::vector<double> & fallback)
  {
    std::vector<double> actual;
    if (getCurrentJointState(actual))
    {
      return actual;
    }
    return fallback;
  }

  // ------------------------------------------------------------
  // Il controller sembra avere un limite di velocita' reale per
  // giunto di circa 0.05 rad/s, osservato in modo consistente sia
  // su a1 (~0.046-0.054 rad/s) che su a2 (~0.054 rad/s),
  // indipendentemente dalla postura del braccio: non e' inerzia,
  // e' un limite del controller. Invece di indovinare una durata
  // fissa per ogni movimento (e scoprire per tentativi quali
  // richiedono piu' tempo), calcoliamo la durata minima necessaria
  // dal giunto che deve spostarsi di piu', con un margine di
  // sicurezza. min_duration_sec evita movimenti innaturalmente
  // brevi per spostamenti piccoli.
  // ------------------------------------------------------------
  double computeSafeDuration(
    const std::vector<double> & start,
    const std::vector<double> & target,
    double min_duration_sec = 3.0)
  {
    constexpr double MAX_SAFE_JOINT_VEL = 0.05;  // rad/s, osservato sperimentalmente
    constexpr double SAFETY_MARGIN = 1.3;        // 30% di margine

    if (start.size() != target.size()) return min_duration_sec;

    double max_delta = 0.0;
    for (size_t i = 0; i < start.size(); ++i)
    {
      max_delta = std::max(max_delta, std::fabs(target[i] - start[i]));
    }

    const double needed = (max_delta / MAX_SAFE_JOINT_VEL) * SAFETY_MARGIN;
    return std::max(min_duration_sec, needed);
  }

  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!has_kdl_model_) initKDL(msg->data);
  }

  void initKDL(const std::string & xml)
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(xml, tree))
    {
      RCLCPP_ERROR(this->get_logger(), "Errore parsing URDF.");
      return;
    }

    const std::string base_link = "iiwa2_link_0";
    const std::string tip_link = "iiwa2_tool0";

    if (!tree.getChain(base_link, tip_link, kdl_chain_))
    {
      RCLCPP_ERROR(this->get_logger(), "Catena KDL non trovata.");
      return;
    }

    if (kdl_chain_.getNrOfJoints() != 7)
    {
      RCLCPP_ERROR(this->get_logger(), "La catena KDL ha %u joint invece di 7.", kdl_chain_.getNrOfJoints());
      return;
    }

    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(kdl_chain_);
    has_kdl_model_ = true;

    RCLCPP_INFO(this->get_logger(), "KDL inizializzata correttamente.");
  }

  KDL::JntArray vectorToJnt(const std::vector<double> & q)
  {
    KDL::JntArray result(7);
    for (size_t i = 0; i < 7; ++i) result(i) = q[i];
    return result;
  }

  void clampJoints(KDL::JntArray & q)
  {
    const double min_q[7] = {-2.9668, -2.0942, -2.9668, -2.0942, -2.9668, -2.0942, -3.0541};
    const double max_q[7] = { 2.9668,  2.0942,  2.9668,  2.0942,  2.9668,  2.0942,  3.0541};
    for (size_t i = 0; i < 7; ++i) q(i) = std::clamp(q(i), min_q[i], max_q[i]);
  }

  Eigen::Vector3d getPosition(const KDL::JntArray & q)
  {
    KDL::Frame frame;
    if (fk_solver_->JntToCart(q, frame) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Errore FK.");
      return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
  }

  bool solvePositionIK(
    const Eigen::Vector3d & target,
    const std::vector<double> & seed,
    std::vector<double> & solution)
  {
    if (!has_kdl_model_) return false;
    if (seed.size() != 7) return false;

    KDL::JntArray q = vectorToJnt(seed);

    constexpr int MAX_ITER = 5000;
    constexpr double TOLERANCE = 0.003;
    constexpr double LAMBDA = 0.005;
    constexpr double MAX_STEP = 0.05;

    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
      const Eigen::Vector3d current = getPosition(q);
      const Eigen::Vector3d error = target - current;
      const double error_norm = error.norm();

      if (error_norm < TOLERANCE)
      {
        solution.resize(7);
        for (size_t i = 0; i < 7; ++i) solution[i] = q(i);
        RCLCPP_INFO(this->get_logger(), "[IK] Convergente: errore %.6f m", error_norm);
        return true;
      }

      KDL::Jacobian kdl_jacobian(7);
      if (jac_solver_->JntToJac(q, kdl_jacobian) < 0) return false;

      Eigen::Matrix<double, 3, 7> J;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 7; ++c)
          J(r, c) = kdl_jacobian(r, c);

      Eigen::Matrix3d A = J * J.transpose();
      A += LAMBDA * LAMBDA * Eigen::Matrix3d::Identity();

      Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(error);

      double max_dq = 0.0;
      for (int i = 0; i < 7; ++i) max_dq = std::max(max_dq, std::fabs(dq(i)));
      if (max_dq > MAX_STEP) dq *= MAX_STEP / max_dq;

      for (int i = 0; i < 7; ++i) q(i) += dq(i);
      clampJoints(q);
    }

    return false;
  }

  bool solvePositionIKMultiSeed(
    const Eigen::Vector3d & target,
    const std::vector<double> & preferred_seed,
    std::vector<double> & solution)
  {
    std::vector<std::vector<double>> seeds;
    seeds.push_back(preferred_seed);
    seeds.push_back(initial_conf_);
    seeds.push_back(home_conf_);
    seeds.push_back({ 0.625,  0.950, 0.240, -1.150, -0.112, 0.650, 0.0 });
    seeds.push_back({-0.625,  0.950, 0.240, -1.150, -0.112, 0.650, 0.0 });
    seeds.push_back({ 0.625,  1.150, 0.240, -1.450, -0.112, 0.800, 0.0 });

    for (size_t i = 0; i < seeds.size(); ++i)
    {
      if (solvePositionIK(target, seeds[i], solution))
      {
        RCLCPP_INFO(this->get_logger(), "[IK] Soluzione trovata con seed %zu", i + 1);
        return true;
      }
    }
    return false;
  }

  bool calculateConfigurations(
    const std::string & tool,
    const std::vector<double> & current_q,
    std::vector<double> & q_pre,
    std::vector<double> & q_grasp)
  {
    const auto it = tool_poses_.find(tool);
    if (it == tool_poses_.end()) return false;

    const ToolPose & pose = it->second;
    constexpr double PRE_HEIGHT = 0.12;

    Eigen::Vector3d grasp_target(pose.x, pose.y, pose.z);
    Eigen::Vector3d pre_target(pose.x, pose.y, pose.z + PRE_HEIGHT);

    RCLCPP_INFO(this->get_logger(), "TOOL = %s", tool.c_str());
    RCLCPP_INFO(this->get_logger(), "GRASP target = [%.3f %.3f %.3f]",
      grasp_target.x(), grasp_target.y(), grasp_target.z());
    RCLCPP_INFO(this->get_logger(), "PRE target = [%.3f %.3f %.3f]",
      pre_target.x(), pre_target.y(), pre_target.z());

    if (!solvePositionIKMultiSeed(pre_target, current_q, q_pre))
    {
      RCLCPP_ERROR(this->get_logger(), "IK PRE fallita.");
      return false;
    }

    if (!solvePositionIKMultiSeed(grasp_target, q_pre, q_grasp))
    {
      RCLCPP_ERROR(this->get_logger(), "IK GRASP fallita.");
      return false;
    }

    const Eigen::Vector3d fk_pre = getPosition(vectorToJnt(q_pre));
    const Eigen::Vector3d fk_grasp = getPosition(vectorToJnt(q_grasp));

    RCLCPP_INFO(this->get_logger(), "FK PRE   = [%.4f %.4f %.4f]", fk_pre.x(), fk_pre.y(), fk_pre.z());
    RCLCPP_INFO(this->get_logger(), "FK GRASP = [%.4f %.4f %.4f]", fk_grasp.x(), fk_grasp.y(), fk_grasp.z());

    return true;
  }

  bool moveToConfiguration(
    const std::vector<double> & start,
    const std::vector<double> & target,
    double duration_sec)
  {
    if (start.size() != 7 || target.size() != 7) return false;
    if (duration_sec <= 0.0) duration_sec = 1.0;

    constexpr double CONTROL_HZ = 100.0;
    const int steps = std::max(1, static_cast<int>(std::ceil(duration_sec * CONTROL_HZ)));
    const auto step_duration = std::chrono::duration<double>(duration_sec / static_cast<double>(steps));

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(7);

    for (int step = 1; step <= steps; ++step)
    {
      const double alpha = static_cast<double>(step) / static_cast<double>(steps);
      for (size_t i = 0; i < 7; ++i)
        msg.data[i] = start[i] + alpha * (target[i] - start[i]);

      arm_pub_->publish(msg);
      std::this_thread::sleep_for(step_duration);
    }

    msg.data = target;
    for (int i = 0; i < 20; ++i)
    {
      arm_pub_->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return true;
  }

  bool waitForArrival(
    const std::vector<double> & target,
    double tolerance_rad,
    double timeout_sec)
  {
    const auto start_time = this->now();

    while (rclcpp::ok())
    {
      std::vector<double> current;
      if (getCurrentJointState(current))
      {
        double max_err = 0.0;
        for (size_t i = 0; i < 7; ++i)
          max_err = std::max(max_err, std::fabs(current[i] - target[i]));

        if (max_err < tolerance_rad)
        {
          RCLCPP_INFO(this->get_logger(), "[ARRIVO] Confermato. Errore massimo: %.4f rad", max_err);
          return true;
        }
      }

      const double elapsed = (this->now() - start_time).seconds();
      if (elapsed > timeout_sec)
      {
        std::vector<double> current_fail;
        getCurrentJointState(current_fail);

        RCLCPP_ERROR(this->get_logger(),
          "[ARRIVO] TIMEOUT dopo %.1fs. Il braccio NON ha raggiunto il target reale.", timeout_sec);

        for (size_t i = 0; i < 7 && i < current_fail.size(); ++i)
          RCLCPP_ERROR(this->get_logger(), "  a%zu: reale=%.4f  target=%.4f  diff=%.4f",
            i + 1, current_fail[i], target[i], current_fail[i] - target[i]);

        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return false;
  }

  void holdConfiguration(const std::vector<double> & target, double duration_sec)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = target;
    const int steps = static_cast<int>(duration_sec * 100.0);
    for (int i = 0; i < steps && rclcpp::ok(); ++i)
    {
      arm_pub_->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void userRequestCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!ready_)
    {
      RCLCPP_WARN(this->get_logger(), "Nodo non ancora pronto (detach iniziale in corso).");
      return;
    }
    const std::string tool = normalize(msg->data);
    if (!validTool(tool)) return;
    if (busy_) return;
    pending_tool_ = tool;
  }

  void visionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!ready_) return;
    const std::string detected = normalize(msg->data);
    if (busy_) return;
    if (pending_tool_.empty()) return;
    if (detected != pending_tool_) return;

    const std::string tool = pending_tool_;
    pending_tool_.clear();
    busy_ = true;

    std::thread(&Iiwa2PickKdlNode::executePick, this, tool).detach();
  }

  void directPickCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!ready_)
    {
      RCLCPP_WARN(this->get_logger(), "[DIRECT PICK] Nodo non ancora pronto (detach iniziale in corso).");
      return;
    }
    if (busy_) return;

    const std::string tool = normalize(msg->data);
    if (!validTool(tool))
    {
      RCLCPP_WARN(this->get_logger(), "[DIRECT PICK] Tool non valido.");
      return;
    }

    busy_ = true;
    RCLCPP_INFO(this->get_logger(), "[DIRECT PICK] Avvio pick: %s", tool.c_str());

    std::thread(&Iiwa2PickKdlNode::executePick, this, tool).detach();
  }

  void publishStatus(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
  }

  void executePick(const std::string & tool)
  {
    publishStatus("busy");
    bool success = false;

    try
    {
      if (!has_kdl_model_)
      {
        RCLCPP_ERROR(this->get_logger(), "KDL non inizializzata.");
        busy_ = false;
        return;
      }

      std::vector<double> current_q = initial_conf_;
      std::vector<double> q_pre;
      std::vector<double> q_grasp;

      if (!calculateConfigurations(tool, current_q, q_pre, q_grasp))
      {
        busy_ = false;
        return;
      }

      // ---------------------------------------------------------
      // 1. INITIAL -> PRE-PICK
      // ---------------------------------------------------------
      RCLCPP_INFO(this->get_logger(), "[1/5] INITIAL -> PRE-PICK");

      {
        const auto start = getStartPoint(initial_conf_);
        const double dur = computeSafeDuration(start, q_pre);
        if (!moveToConfiguration(start, q_pre, dur))
        {
          busy_ = false;
          return;
        }
      }

      // Non bloccante: se non arriva in tempo, il movimento
      // successivo (piu' lungo) avra' modo di recuperare.
      if (!waitForArrival(q_pre, 0.05, 10.0))
      {
        RCLCPP_WARN(this->get_logger(), "Scostamento su PRE-PICK, procedo comunque.");
      }

      // ---------------------------------------------------------
      // 2. PRE-PICK -> GRASP
      // ---------------------------------------------------------
      RCLCPP_INFO(this->get_logger(), "[2/5] PRE-PICK -> GRASP");

      {
        const auto start = getStartPoint(q_pre);
        const double dur = computeSafeDuration(start, q_grasp);
        if (!moveToConfiguration(start, q_grasp, dur))
        {
          busy_ = false;
          return;
        }
      }

      // BLOCCANTE: e' il controllo critico prima dell'attach.
      if (!waitForArrival(q_grasp, 0.08, 10.0))
      {
        RCLCPP_ERROR(this->get_logger(), "Arrivo a GRASP non confermato. Interrompo il pick PRIMA dell'attach.");
        busy_ = false;
        return;
      }

      // Breve stabilizzazione fisica (1s, non piu' 5s): la vera
      // garanzia e' il controllo waitForArrival subito dopo, che
      // ritorna appena confermato - non serve un'attesa cieca lunga.
      RCLCPP_INFO(this->get_logger(), "Stabilizzo GRASP (1s).");
      holdConfiguration(q_grasp, 1.0);

      if (!waitForArrival(q_grasp, 0.08, 5.0))
      {
        RCLCPP_ERROR(this->get_logger(), "Il braccio e' CEDUTO durante l'hold. Interrompo prima dell'attach.");
        busy_ = false;
        return;
      }

      // ---------------------------------------------------------
      // 3. ATTACH
      // ---------------------------------------------------------
      RCLCPP_INFO(this->get_logger(), "[3/5] ATTACH %s", tool.c_str());

      const auto attach_it = attach_pubs_.find(tool);
      if (attach_it == attach_pubs_.end())
      {
        busy_ = false;
        return;
      }

      std_msgs::msg::Empty empty_msg;
      for (int i = 0; i < 10; ++i)
      {
        attach_it->second->publish(empty_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }

      // Assestamento fisico: il plugin DetachableJoint ha bisogno
      // di un attimo per creare davvero il vincolo rigido dentro
      // Gazebo. Se il braccio inizia a risalire troppo presto,
      // l'oggetto rischia di restare indietro (attach non ancora
      // effettivo).
      RCLCPP_INFO(this->get_logger(), "Assestamento attach (1.5s)...");
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));

      // ---------------------------------------------------------
      // 4. RISALITA DA GRASP A PRE-PICK
      // ---------------------------------------------------------
      RCLCPP_INFO(this->get_logger(), "[4/5] Risalita a PRE-PICK");

      {
        const auto start = getStartPoint(q_grasp);
        const double dur = computeSafeDuration(start, q_pre);
        if (!moveToConfiguration(start, q_pre, dur))
        {
          busy_ = false;
          return;
        }
      }
      waitForArrival(q_pre, 0.15, 10.0);

      // ---------------------------------------------------------
      // 5. TRASFERIMENTO VERSO IL ROVER (Pre-Place alto)
      // ---------------------------------------------------------
      RCLCPP_INFO(this->get_logger(), "[5/6] Trasferimento verso il Rover");

      // 1. Alziamo PRIMA il braccio in sicurezza sopra il tavolo (a1 = 0.0)
      std::vector<double> q_table_high = { 0.0, 0.40, 0.0, -0.80, 0.0, 0.80, 0.0 };
      // 2. POI ruotiamo solo la base verso il rover (a1 = -1.5708)
      std::vector<double> q_rover_high = { -1.5708, 0.40, 0.0, -0.80, 0.0, 0.80, 0.0 };

      {
        const auto start = getStartPoint(q_pre);
        const double dur = computeSafeDuration(start, q_table_high);
        if (!moveToConfiguration(start, q_table_high, dur)) return;
      }
      if (!waitForArrival(q_table_high, 0.15, 10.0))
      {
        RCLCPP_ERROR(this->get_logger(), "Collisione durante il sollevamento sul tavolo.");
        busy_ = false;
        return;
      }

      // ------------------------------------------------------------
      // Il controller sembra avere un limite di velocita' reale per
      // giunto di circa 0.05 rad/s (osservato su a1 e su a2 in
      // istanze diverse, indipendentemente dalla postura): la durata
      // qui sotto e' calcolata da quel limite invece che indovinata.
      // ------------------------------------------------------------
      {
        const auto start = getStartPoint(q_table_high);
        const double dur = computeSafeDuration(start, q_rover_high);
        if (!moveToConfiguration(start, q_rover_high, dur)) return;
      }
      if (!waitForArrival(q_rover_high, 0.08, 15.0))
      {
        RCLCPP_ERROR(this->get_logger(), "Rotazione verso il rover fallita.");
        busy_ = false;
        return;
      }

      // ---------------------------------------------------------
      // 6. DISCESA CONTROLLATA E PLACE
      // ---------------------------------------------------------
      RCLCPP_INFO(this->get_logger(), "[6/6] Discesa controllata sul rover");

      // ------------------------------------------------------------
      // Il punto di PLACE (z=0.00 relativo a iiwa2_link_0) e' gia'
      // abbastanza vicino al vassoio del rover da rilasciare
      // l'oggetto direttamente da li' - confermato empiricamente.
      // Niente discesa ulteriore ne' rilevazione di contatto: e'
      // questo il punto di rilascio.
      // ------------------------------------------------------------
      constexpr double PLACE_Z = 0.03;

      const Eigen::Vector3d place_target(0.0, -0.80, PLACE_Z);

      std::vector<double> q_place;

      if (!solvePositionIKMultiSeed(place_target, rover_home_conf, q_place))
      {
        RCLCPP_ERROR(this->get_logger(), "IK PLACE fallita: target irraggiungibile.");
        busy_ = false;
        return;
      }

      {
        const Eigen::Vector3d fk_place = getPosition(vectorToJnt(q_place));
        RCLCPP_INFO(this->get_logger(), "FK PLACE = [%.4f %.4f %.4f]",
          fk_place.x(), fk_place.y(), fk_place.z());
      }

      {
        const auto start = getStartPoint(q_rover_high);
        const double dur = computeSafeDuration(start, q_place);
        RCLCPP_INFO(this->get_logger(), "Discesa a PLACE: durata calcolata %.1fs", dur);
        if (!moveToConfiguration(start, q_place, dur)) return;
      }
      if (!waitForArrival(q_place, 0.05, 15.0))
      {
        RCLCPP_ERROR(this->get_logger(), "Arrivo a PLACE fallito.");
        busy_ = false;
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Stabilizzazione solida sul vassoio (2.5s)...");
      holdConfiguration(q_place, 2.5);

      // ------------------------------------------------------------
      // SECONDO CONTROLLO, come gia' fatto prima dell'ATTACH: se il
      // braccio e' scivolato/oscillato durante l'hold, non facciamo
      // partire il detach alla cieca - lo stesso pattern che gia'
      // protegge la fase di grasp. Tolleranza leggermente piu'
      // permissiva (0.08 invece di 0.05): un piccolo assestamento
      // dovuto al contatto reale col vassoio e' normale, un drift
      // grande come quello visto nel log (0.37 rad) non lo e' e deve
      // comunque far scattare l'errore.
      // ------------------------------------------------------------
      if (!waitForArrival(q_place, 0.08, 5.0))
      {
        RCLCPP_ERROR(this->get_logger(), "Il braccio si e' spostato durante l'hold sul rover. Interrompo prima del detach.");
        busy_ = false;
        return;
      }

      // ---------------------------------------------------------
      // 7. DETACH (rilascio sul rover)
      // ---------------------------------------------------------
      RCLCPP_INFO(this->get_logger(), "Rilascio oggetto sul rover: %s", tool.c_str());

      const auto detach_it = detach_pubs_.find(tool);
      if (detach_it != detach_pubs_.end())
      {
        std_msgs::msg::Empty empty_msg2;
        for (int i = 0; i < 10; ++i)
        {
          detach_it->second->publish(empty_msg2);
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));


      RCLCPP_INFO(this->get_logger(), "Ritiro dal rover...");
      {
        const auto start = getStartPoint(q_place);
        const double dur = computeSafeDuration(start, rover_home_conf);
        moveToConfiguration(start, rover_home_conf, dur);
      }
      waitForArrival(rover_home_conf, 0.15, 10.0);

      {
        // Stessa postura di initial_conf_ (braccio ripiegato) ma con
        // a1 ancora rivolto verso il rover: nessun urto possibile
        // qui, e' solo un ripiegamento sul posto.
        std::vector<double> q_fold_rover_home = initial_conf_;
        q_fold_rover_home[0] = rover_home_conf[0];  

        RCLCPP_INFO(this->get_logger(), "Ripiego il braccio prima di ruotare verso il tavolo...");
        {
          const auto start = getStartPoint(rover_home_conf);
          const double dur = computeSafeDuration(start, q_fold_rover_home);
          if (!moveToConfiguration(start, q_fold_rover_home, dur))
          {
            RCLCPP_WARN(this->get_logger(), "Ripiegamento pre-ritorno non completato, procedo comunque.");
          }
        }
        if (!waitForArrival(q_fold_rover_home, 0.15, 10.0))
        {
          RCLCPP_WARN(this->get_logger(), "Braccio non perfettamente ripiegato prima della rotazione di ritorno.");
        }

        RCLCPP_INFO(this->get_logger(), "Ritorno a INITIAL, pronto per un nuovo pick.");
        {
          const auto start = getStartPoint(q_fold_rover_home);
          const double dur = computeSafeDuration(start, initial_conf_);
          if (!moveToConfiguration(start, initial_conf_, dur))
          {
            RCLCPP_WARN(this->get_logger(), "Rotazione di ritorno non completata, procedo comunque.");
          }
        }
        if (!waitForArrival(initial_conf_, 0.05, 15.0))
        {
          RCLCPP_WARN(this->get_logger(), "Il braccio non e' tornato esattamente a INITIAL (prossimo pick ripartira' dalla posizione reale).");
        }
      }

      RCLCPP_INFO(this->get_logger(), "PICK & PLACE COMPLETATO: %s", tool.c_str());
      success = true;
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Eccezione durante PICK & PLACE: %s", e.what());
    }

    publishStatus(success ? "done" : "error");
    busy_ = false;
  }


  // MEMBERS
  std::vector<std::string> joint_names_;
  std::vector<double> initial_conf_;
  std::vector<double> home_conf_;
  std::vector<double> rover_home_conf;
  std::string pending_tool_;
  std::atomic_bool busy_;
  bool has_kdl_model_;
  bool has_joint_state_;
  std::atomic_bool ready_;

  std::vector<double> current_joint_positions_{0.0, 0.06, 0.0, -0.95, 0.0, 0.52, 0.0};
  std::mutex joint_state_mutex_;

  std::map<std::string, ToolPose> tool_poses_;

  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> attach_pubs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> detach_pubs_;

  rclcpp::TimerBase::SharedPtr startup_timer_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr user_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vision_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr direct_pick_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_global_sub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Iiwa2PickKdlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
