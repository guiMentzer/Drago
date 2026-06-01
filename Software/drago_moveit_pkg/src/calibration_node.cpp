/**
 * joint_calibration_node.cpp
 *
 * Nó de calibração física do braço robótico.
 * 
 * - Publica continuamente em /joint_states (50 Hz) para manter o
 *   robot_state_publisher atualizado.
 * - Envia trajetórias para o action server FollowJointTrajectory,
 *   de forma que o ros2_control mova o robô fisicamente.
 *
 * Comandos disponíveis no terminal:
 *   set  <índice> <ângulo>   — move uma junta para o ângulo (rad)
 *   move <índice> <ângulo>   — alias de set
 *   list                     — mostra o estado atual de todas as juntas
 *   reset                    — envia todas as juntas para 0.0 rad
 *   quit | q                 — encerra o nó
 *
 * Exemplo:
 *   set 0  1.57
 *   set 3 -0.5
 *   list
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle            = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// ─── Configuração ─────────────────────────────────────────────────────────────

// Nome do action server do seu controller — altere se necessário
static const std::string ACTION_SERVER =
  "/joint_trajectory_controller/follow_joint_trajectory";

// Juntas do robô — altere para o seu robô se não for o Panda
static const std::vector<std::string> JOINT_NAMES = {
  "Waist_joint",
  "Shoulder_joint",
  "Elbow_joint",
  "Wrist_1_joint",
  "Wrist_2_joint",
  "Effector_joint",
};

// Tempo de movimento para cada comando (segundos)
// Aumente para movimentos mais lentos e seguros durante calibração
static constexpr double MOVEMENT_DURATION_S = 3.0;

// ─────────────────────────────────────────────────────────────────────────────

class JointCalibrationNode : public rclcpp::Node
{
public:
  JointCalibrationNode()
  : Node("calibration_node"),
    running_(true)
  {
    // Publisher de /joint_states (leitura pelo MoveGroup e RViz)
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    // Action client para o controller (move o robô fisicamente)
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, ACTION_SERVER);

    // Inicializa estado com zeros
    state_.name     = JOINT_NAMES;
    state_.position .assign(state_.name.size(), 0.0);
    state_.velocity .assign(state_.name.size(), 0.0);
    state_.effort   .assign(state_.name.size(), 0.0);

    // Timer de publicação a 50 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() { publishJointState(); });

    // Thread de leitura do terminal
    input_thread_ = std::thread(&JointCalibrationNode::inputLoop, this);

    // Aguarda o action server subir
    RCLCPP_INFO(this->get_logger(),
      "Aguardando action server '%s'...", ACTION_SERVER.c_str());

    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_WARN(this->get_logger(),
        "Action server nao encontrado. Comandos 'set/move' nao moverao o robo fisicamente.\n"
        "Verifique se o joint_trajectory_controller esta rodando:\n"
        "  ros2 action list | grep follow_joint");
    } else {
      RCLCPP_INFO(this->get_logger(), "Action server conectado.");
    }

    printHelp();
  }

  ~JointCalibrationNode()
  {
    running_ = false;
    if (input_thread_.joinable())
      input_thread_.join();
  }

private:
  // ── Publicação de /joint_states ─────────────────────────────────────────────
  void publishJointState()
  {
    state_.header.stamp = this->now();
    joint_state_pub_->publish(state_);
  }

  // ── Envio de trajetória para o action server ────────────────────────────────
  void sendTrajectoryGoal(const std::vector<double>& target_positions)
  {
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN(this->get_logger(),
        "Action server nao disponivel — apenas /joint_states foi atualizado.");
      return;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = state_.name;

    // Ponto de início: posição atual (t = 0)
    trajectory_msgs::msg::JointTrajectoryPoint start;
    start.positions  = state_.position;
    start.velocities .assign(state_.name.size(), 0.0);
    start.time_from_start = rclcpp::Duration::from_seconds(0.0);
    goal.trajectory.points.push_back(start);

    // Ponto de destino: posição alvo
    trajectory_msgs::msg::JointTrajectoryPoint target;
    target.positions  = target_positions;
    target.velocities .assign(state_.name.size(), 0.0);
    target.time_from_start = rclcpp::Duration::from_seconds(MOVEMENT_DURATION_S);
    goal.trajectory.points.push_back(target);

    auto send_goal_options =
      rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandle::SharedPtr& gh) {
        if (!gh)
          RCLCPP_ERROR(this->get_logger(), "Goal rejeitado pelo controller.");
        else
          RCLCPP_INFO(this->get_logger(), "Goal aceito — movendo robo...");
      };

    send_goal_options.result_callback =
      [this](const GoalHandle::WrappedResult& result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),  "Movimento concluido.");         break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Movimento abortado.");          break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(),  "Movimento cancelado.");         break;
          
            RCLCPP_ERROR(this->get_logger(), "Resultado desconhecido.");      break;
        }
      };

    action_client_->async_send_goal(goal, send_goal_options);
  }

  // ── Loop de leitura do terminal ─────────────────────────────────────────────
  void inputLoop()
  {
    std::string line;
    while (running_ && std::getline(std::cin, line)) {
      if (line.empty()) continue;

      std::istringstream ss(line);
      std::string cmd;
      ss >> cmd;

      if (cmd == "set" || cmd == "move") {
        int    index;
        double angle;
        if (!(ss >> index >> angle)) {
          printError("Uso: set <indice> <angulo_rad>");
          continue;
        }
        moveJoint(index, angle);

      } else if (cmd == "list") {
        printState();

      } else if (cmd == "reset") {
        std::vector<double> zeros(state_.name.size(), 0.0);
        RCLCPP_INFO(this->get_logger(), "Enviando todas as juntas para 0.0 rad...");
        state_.position = zeros;
        sendTrajectoryGoal(zeros);

      } else if (cmd == "quit" || cmd == "q") {
        RCLCPP_INFO(this->get_logger(), "Encerrando...");
        running_ = false;
        rclcpp::shutdown();
        break;

      } else {
        printError("Comando desconhecido: '" + cmd + "'");
        printHelp();
      }
    }
  }

  // ── Helpers ─────────────────────────────────────────────────────────────────
  void moveJoint(int index, double angle)
  {
    const int n = static_cast<int>(state_.name.size());
    if (index < 0 || index >= n) {
      printError("Indice " + std::to_string(index) +
                 " invalido. Juntas disponiveis: 0 a " + std::to_string(n - 1));
      return;
    }

    std::vector<double> target = state_.position;
    target[index]              = angle;
    state_.position[index]     = angle;   // atualiza estado local

    RCLCPP_INFO(this->get_logger(),
      "  [%d] %s  ->  %.4f rad  (%.2f deg)",
      index, state_.name[index].c_str(), angle, angle * 180.0 / M_PI);

    sendTrajectoryGoal(target);
  }

  void printState() const
  {
    std::cout << "\n+-- Estado atual das juntas -------------------+\n";
    for (size_t i = 0; i < state_.name.size(); ++i) {
      printf("|  [%zu] %-20s  %7.4f rad  (%7.2f deg)\n",
        i, state_.name[i].c_str(),
        state_.position[i],
        state_.position[i] * 180.0 / M_PI);
    }
    std::cout << "+---------------------------------------------+\n\n";
  }

  void printHelp() const
  {
    std::cout <<
      "\n+== joint_calibration_node ========================+\n"
        "|  set <indice> <angulo_rad>   move junta          |\n"
        "|  list                        mostra estado atual |\n"
        "|  reset                       envia tudo para 0   |\n"
        "|  quit | q                    encerra             |\n"
        "+==================================================+\n\n";
  }

  void printError(const std::string& msg) const
  {
    std::cerr << "[ERRO] " << msg << "\n";
  }

  // ── Membros ─────────────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  joint_state_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr     action_client_;
  rclcpp::TimerBase::SharedPtr                                timer_;
  sensor_msgs::msg::JointState                                state_;
  std::thread                                                 input_thread_;
  std::atomic<bool>                                           running_;
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}