#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
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

using namespace std::chrono_literals;

// =================================================================
// IIWA1 PICK & PLACE NODE - SOLO POSIZIONI HARDCODED (nessun KDL)
// =================================================================

// ===================================================================
// POSIZIONI (giunti a1..a7, in rad). E' l'UNICO posto da modificare
// per tarare i movimenti.
// ===================================================================

// a1 verso il rover: atan2(0.74, 0.01) = 1.5573 rad (89.2 gradi)
constexpr double A1_ROVER = 1.5573;

// ---- GRASP SUL ROVER: parametri da tarare --------------------------
// Nel log precedente a2 si fermava a ~1.53 (contro il target 1.70):
// c'e' contatto fisico, quindi alzare ancora a2 NON fa scendere
// l'attrezzo. Per scendere un po' di piu' si agisce sul gomito (a4).
//   - a4 piu' negativo -> avambraccio piu' verticale: la pinza scende
//     un po' e arretra un po'
//   - a4 meno negativo -> la pinza va piu' lontano / piu' alta
//   - NON andare oltre circa -1.0: a -1.05 e -1.10 hai avuto timeout.
// Se dopo il test la pinza e' ancora alta prova -0.95; se invece
// "non arriva" in avanti prova il contrario (-0.75).
constexpr double GRASP_A2 = 1.75;
constexpr double GRASP_A4 = -0.55;   // era -0.75
constexpr double GRASP_A6 = 0.45;    // se la pinza risulta inclinata, ritoccare questo

using Joints = std::vector<double>;

const Joints Q_HOME        = { 0.00,     0.06,     0.00, -0.95,     0.00, 0.52,     0.00 };

const Joints Q_ROVER_HIGH  = { A1_ROVER, 0.06,     0.00, -0.95,     0.00, 0.52,     0.00 };
const Joints Q_ROVER_PRE   = { A1_ROVER, 0.30,     0.00, -1.20,     0.00, 0.80,     0.00 };
const Joints Q_ROVER_GRASP = { A1_ROVER, GRASP_A2, 0.00, GRASP_A4,  0.00, GRASP_A6, 0.00 };

const Joints Q_TABLE_HIGH  = { 0.00,     0.06,     0.00, -0.95,     0.00, 0.52,     0.00 };
// const Joints Q_TABLE_PRE   = { 0.00,     0.30,     0.00, -1.15,     0.00, 0.80,     0.00 };
// const Joints Q_TABLE_PLACE = { 0.00,     0.33,     0.00, -1.05,     0.00, 0.70,     0.00 };


const Joints Q_TABLE_PRE   = { 0.00,     0.40,     0.00, -1.00,     0.00, 0.80,     0.00 };
const Joints Q_TABLE_PLACE = { 0.00,     0.45,     0.00, -0.85,     0.00, 0.70,     0.00 };


class Iiwa1PickNode : public rclcpp::Node
{
public:

  Iiwa1PickNode()
  : Node("iiwa1_pick_node"),
    busy_(false),
    has_joint_state_(false),
    ready_(false)
  {
    joint_names_ = {
      "iiwa1_joint_a1", "iiwa1_joint_a2", "iiwa1_joint_a3", "iiwa1_joint_a4",
      "iiwa1_joint_a5", "iiwa1_joint_a6", "iiwa1_joint_a7"
    };

    arm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/iiwa1/iiwa_arm_controller/commands", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/iiwa1/pick_status", 10);

    // Un publisher di attach/detach per ciascuno strumento
    attach_pubs_["scalpel"]  = this->create_publisher<std_msgs::msg::Empty>("/iiwa1/picker/scalpel/attach", 10);
    attach_pubs_["scissors"] = this->create_publisher<std_msgs::msg::Empty>("/iiwa1/picker/scissors/attach", 10);
    attach_pubs_["forceps"]  = this->create_publisher<std_msgs::msg::Empty>("/iiwa1/picker/forceps/attach", 10);

    detach_pubs_["scalpel"]  = this->create_publisher<std_msgs::msg::Empty>("/iiwa1/picker/scalpel/detach", 10);
    detach_pubs_["scissors"] = this->create_publisher<std_msgs::msg::Empty>("/iiwa1/picker/scissors/detach", 10);
    detach_pubs_["forceps"]  = this->create_publisher<std_msgs::msg::Empty>("/iiwa1/picker/forceps/detach", 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/iiwa1/joint_states", 10, std::bind(&Iiwa1PickNode::jointStateCallback, this, std::placeholders::_1));

    trigger_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/iiwa1/start_place_command", 10, std::bind(&Iiwa1PickNode::triggerCallback, this, std::placeholders::_1));

    startup_timer_ = this->create_wall_timer(3s, std::bind(&Iiwa1PickNode::performInitialDetach, this));

    RCLCPP_INFO(this->get_logger(), "============================================");
    RCLCPP_INFO(this->get_logger(), "IIWA1 PICK & PLACE NODE STARTED (hardcoded)");
    RCLCPP_INFO(this->get_logger(), "============================================");
  }

private:

  void performInitialDetach()
  {
    startup_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Executing initial detach...");
    std_msgs::msg::Empty empty_msg;

    for (auto & kv : detach_pubs_)
    {
      for (int i = 0; i < 15; ++i)
      {
        kv.second->publish(empty_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ready_ = true;
    RCLCPP_INFO(this->get_logger(), "IIWA1 ready for pick & place tasks.");
    publishStatus("idle");
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

  // Posizione reale se disponibile, altrimenti il valore di riserva
  std::vector<double> getStartPoint(const std::vector<double> & fallback)
  {
    std::vector<double> actual;
    if (getCurrentJointState(actual)) return actual;
    return fallback;
  }

  // Errore massimo (su tutti i giunti) tra posizione reale e target
  double maxJointError(const std::vector<double> & target)
  {
    std::vector<double> now_q;
    if (!getCurrentJointState(now_q)) return 999.0;
    double err = 0.0;
    for (size_t i = 0; i < 7; ++i) err = std::max(err, std::fabs(now_q[i] - target[i]));
    return err;
  }

  // Durata calcolata dal giunto che si sposta di piu' (velocita' reale ~0.05 rad/s)
  double computeSafeDuration(const std::vector<double> & start, const std::vector<double> & target, double min_duration_sec = 3.0)
  {
    constexpr double MAX_SAFE_JOINT_VEL = 0.05;
    constexpr double SAFETY_MARGIN = 1.3;
    if (start.size() != target.size()) return min_duration_sec;

    double max_delta = 0.0;
    for (size_t i = 0; i < start.size(); ++i)
      max_delta = std::max(max_delta, std::fabs(target[i] - start[i]));

    return std::max(min_duration_sec, (max_delta / MAX_SAFE_JOINT_VEL) * SAFETY_MARGIN);
  }

  bool moveToConfiguration(const std::vector<double> & start, const std::vector<double> & target, double duration_sec)
  {
    if (start.size() != 7 || target.size() != 7) return false;
    if (duration_sec <= 0.0) duration_sec = 1.0;

    constexpr double CONTROL_HZ = 100.0;
    const int steps = std::max(1, static_cast<int>(std::ceil(duration_sec * CONTROL_HZ)));
    const auto step_duration = std::chrono::duration<double>(duration_sec / steps);

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(7);

    for (int step = 1; step <= steps; ++step)
    {
      const double alpha = static_cast<double>(step) / steps;
      for (size_t i = 0; i < 7; ++i) msg.data[i] = start[i] + alpha * (target[i] - start[i]);
      arm_pub_->publish(msg);
      std::this_thread::sleep_for(step_duration);
    }
    msg.data = target;
    for (int i = 0; i < 100; ++i)
    {
      arm_pub_->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return true;
  }

  // Parte dalla posizione reale (o da fallback_start) e va a target con durata calcolata
  bool goTo(const std::vector<double> & fallback_start, const std::vector<double> & target, double min_duration_sec = 3.0)
  {
    const auto start = getStartPoint(fallback_start);
    const double dur = computeSafeDuration(start, target, min_duration_sec);
    return moveToConfiguration(start, target, dur);
  }

  bool waitForArrival(const std::vector<double> & target, double tolerance_rad, double timeout_sec)
  {
    const auto start_time = this->now();
    while (rclcpp::ok())
    {
      std::vector<double> current;
      if (getCurrentJointState(current))
      {
        double max_error = 0.0;
        for (size_t i = 0; i < 7; ++i) max_error = std::max(max_error, std::fabs(current[i] - target[i]));
        if (max_error < tolerance_rad)
        {
          RCLCPP_INFO(this->get_logger(), "[ARRIVAL] Target reached. Maximum error: %.4f rad", max_error);
          return true;
        }
      }
      if ((this->now() - start_time).seconds() > timeout_sec)
      {
        RCLCPP_ERROR(this->get_logger(), "[ARRIVAL] TIMEOUT dopo %.1fs (tolleranza %.3f rad). Il braccio non è in posizione!", timeout_sec, tolerance_rad);
        std::vector<double> current_fail;
        if (getCurrentJointState(current_fail))
        {
          for (size_t i = 0; i < 7 && i < current_fail.size(); ++i)
          {
            RCLCPP_ERROR(this->get_logger(), "   a%zu: reale=%.4f  target=%.4f  diff=%.4f",
                         i + 1, current_fail[i], target[i], current_fail[i] - target[i]);
          }
        }
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  void holdConfiguration(const std::vector<double> & target, double duration_sec)
  {
    if (target.size() != 7 || duration_sec <= 0.0) return;
    std_msgs::msg::Float64MultiArray msg;
    msg.data = target;
    const int steps = std::max(1, static_cast<int>(duration_sec * 100.0));
    for (int i = 0; i < steps && rclcpp::ok(); ++i)
    {
      arm_pub_->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  bool validTool(const std::string & tool)
  {
    return tool == "scalpel" || tool == "scissors" || tool == "forceps";
  }

  std::string normalizeTool(std::string tool)
  {
    while (!tool.empty() && std::isspace(static_cast<unsigned char>(tool.front()))) tool.erase(tool.begin());
    while (!tool.empty() && std::isspace(static_cast<unsigned char>(tool.back()))) tool.pop_back();
    std::transform(tool.begin(), tool.end(), tool.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return tool;
  }

  void triggerCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!ready_) { RCLCPP_WARN(this->get_logger(), "IIWA1 is not ready yet."); return; }
    if (busy_) { RCLCPP_WARN(this->get_logger(), "IIWA1 is already executing a task."); return; }

    const std::string tool = normalizeTool(msg->data);
    if (!validTool(tool)) { RCLCPP_WARN(this->get_logger(), "Unknown tool: %s", tool.c_str()); return; }

    busy_ = true;
    RCLCPP_INFO(this->get_logger(), "Starting pick & place sequence for: %s", tool.c_str());
    std::thread(&Iiwa1PickNode::executeSequence, this, tool).detach();
  }

  void publishStatus(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
  }

  void executeSequence(const std::string & tool)
  {
    publishStatus("busy");
    bool success = false;

    try
    {
      std::vector<double> current_q;
      if (!getCurrentJointState(current_q)) current_q = Q_HOME;

      // ---------------- PHASE 1: ruota verso il rover ----------------
      RCLCPP_INFO(this->get_logger(), "[1/11] Rotating toward the Rover...");
      if (!goTo(current_q, Q_ROVER_HIGH)) { busy_ = false; return; }
      if (!waitForArrival(Q_ROVER_HIGH, 0.12, 15.0)) { busy_ = false; return; }

      // ---------------- PHASE 2: sopra il rover ----------------
      RCLCPP_INFO(this->get_logger(), "[2/11] Moving above the Rover...");
      if (!goTo(Q_ROVER_HIGH, Q_ROVER_PRE)) { busy_ = false; return; }
      if (!waitForArrival(Q_ROVER_PRE, 0.08, 10.0)) { busy_ = false; return; }

      // ---------------- PHASE 3: discesa sul rover ----------------
      RCLCPP_INFO(this->get_logger(), "[3/11] Descending to the Rover...");
      
      // Discesa più rapida (4.5 secondi) visto che arriva in anticipo
      if (!goTo(Q_ROVER_PRE, Q_ROVER_GRASP, 3.0)) { busy_ = false; return; }

      // Timeout di sicurezza ridotto
      if (!waitForArrival(Q_ROVER_GRASP, 0.05, 8.0))
      {
        const double residual = maxJointError(Q_ROVER_GRASP);
        if (residual < 0.35)
        {
          RCLCPP_WARN(this->get_logger(), "Ostacolo fisico incontrato (errore residuo %.3f rad). Accetto il contatto e procedo.", residual);
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Arrivo a ROVER-GRASP fallito gravemente (errore %.3f rad). Interrompo.", residual);
          busy_ = false;
          return;
        }
      }

      // Freeze: si tiene la posizione REALE invece di spingere contro il target nominale
      std::vector<double> freeze_q;
      if (getCurrentJointState(freeze_q))
      {
        RCLCPP_INFO(this->get_logger(), "Freno i motori: congelo la posizione reale (1.0s)...");
        holdConfiguration(freeze_q, 1.0); // <--- RIDOTTO A 1 SECONDO
      }
      else
      {
        holdConfiguration(Q_ROVER_GRASP, 1.0); // <--- RIDOTTO A 1 SECONDO
      }

      // Timeout post-freeze ridotto a 5 secondi (visto che aspettiamo solo 1s)
      if (!waitForArrival(Q_ROVER_GRASP, 0.35, 1.0))
      {
        const double residual = maxJointError(Q_ROVER_GRASP);
        if (residual < 0.40)
        {
          RCLCPP_WARN(this->get_logger(), "Braccio stabile post-freeze (errore residuo %.3f rad), procedo all'attach.", residual);
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Il braccio è scivolato via post-freeze (%.3f rad). Interrompo.", residual);
          busy_ = false;
          return;
        }
      }

      // ---------------- PHASE 4: attach immediato ----------------
      RCLCPP_INFO(this->get_logger(), "[4/11] Attaching tool: %s", tool.c_str());

      const auto attach_it = attach_pubs_.find(tool);
      if (attach_it == attach_pubs_.end())
      {
        RCLCPP_ERROR(this->get_logger(), "Strumento sconosciuto per l'attach!");
        busy_ = false; return;
      }

      std_msgs::msg::Empty empty_msg;
      // Bastano 10 messaggi molto veloci (in 0.2 secondi totali) per dare il comando a Gazebo
      for (int i = 0; i < 10; ++i)
      {
        attach_it->second->publish(empty_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      // NESSUNA PAUSA QUI! Il braccio risale immediatamente.

      // ---------------- PHASE 5: risalita ----------------
      RCLCPP_INFO(this->get_logger(), "[5/11] Ascending from the Rover...");
      if (!goTo(Q_ROVER_GRASP, Q_ROVER_PRE)) { busy_ = false; return; }
      waitForArrival(Q_ROVER_PRE, 0.15, 10.0);

      // ---------------- PHASE 6: postura ripiegata ----------------
      RCLCPP_INFO(this->get_logger(), "[6/11] Retracting to folded posture...");
      if (!goTo(Q_ROVER_PRE, Q_ROVER_HIGH)) { busy_ = false; return; }
      waitForArrival(Q_ROVER_HIGH, 0.15, 10.0);

      // ---------------- PHASE 7: verso il tavolo ----------------
      RCLCPP_INFO(this->get_logger(), "[7/11] Moving above the bedtable...");
      if (!goTo(Q_ROVER_HIGH, Q_TABLE_HIGH)) { busy_ = false; return; }
      waitForArrival(Q_TABLE_HIGH, 0.05, 15.0);

      // ---------------- PHASE 8: discesa sul tavolo ----------------
      RCLCPP_INFO(this->get_logger(), "[8/11] Descending to the bedtable...");
      if (!goTo(Q_TABLE_HIGH, Q_TABLE_PRE)) { busy_ = false; return; }
      
      // Tolleranza alzata a 0.10 per l'inerzia dello strumento pesante
      waitForArrival(Q_TABLE_PRE, 0.10, 10.0);

      if (!goTo(Q_TABLE_PRE, Q_TABLE_PLACE, 6.0)) { busy_ = false; return; }
      
      // Tolleranza alzata a 0.25 perché lo strumento tocca il tavolo!
      if (!waitForArrival(Q_TABLE_PLACE, 0.25, 10.0))
      {
        RCLCPP_WARN(this->get_logger(), "Timeout su PLACE, ma probabile contatto col tavolo. Procedo.");
      }

      // Hold post-contatto
      std::vector<double> freeze_place_q;
      if (getCurrentJointState(freeze_place_q)) {
        holdConfiguration(freeze_place_q, 2.0);
      } else {
        holdConfiguration(Q_TABLE_PLACE, 2.0);
      }

      // Tolleranza di sicurezza durante l'hold alzata a 0.30
      if (!waitForArrival(Q_TABLE_PLACE, 0.30, 5.0))
      {
        RCLCPP_ERROR(this->get_logger(), "Il braccio si è spostato troppo durante l'hold sul tavolo.");
        busy_ = false;
        return;
      }

      // ---------------- PHASE 9: detach ----------------
      RCLCPP_INFO(this->get_logger(), "[9/11] Releasing tool on the bedtable...");
      const auto detach_it = detach_pubs_.find(tool);
      if (detach_it != detach_pubs_.end())
      {
        std_msgs::msg::Empty empty_msg2;
        for (int i = 0; i < 15; ++i)
        {
          detach_it->second->publish(empty_msg2);
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // ---------------- PHASE 10: risalita dal tavolo ----------------
      RCLCPP_INFO(this->get_logger(), "[10/11] Ascending from the bedtable...");
      goTo(Q_TABLE_PLACE, Q_TABLE_HIGH);
      waitForArrival(Q_TABLE_HIGH, 0.15, 10.0);

      // ---------------- PHASE 11: ritorno a HOME ----------------
      RCLCPP_INFO(this->get_logger(), "[11/11] Returning to HOME...");
      goTo(Q_TABLE_HIGH, Q_HOME);
      waitForArrival(Q_HOME, 0.05, 10.0);

      RCLCPP_INFO(this->get_logger(), "============================================");
      RCLCPP_INFO(this->get_logger(), "PICK & PLACE COMPLETED: %s", tool.c_str());
      RCLCPP_INFO(this->get_logger(), "IIWA1 returned to HOME.");
      RCLCPP_INFO(this->get_logger(), "============================================");

      success = true;
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception during iiwa1 PICK & PLACE: %s", e.what());
    }

    publishStatus(success ? "done" : "error");
    busy_ = false;
  }

  std::vector<std::string> joint_names_;
  std::atomic_bool busy_;
  bool has_joint_state_;
  std::atomic_bool ready_;
  std::vector<double> current_joint_positions_ = Q_HOME;
  std::mutex joint_state_mutex_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> attach_pubs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> detach_pubs_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Iiwa1PickNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
