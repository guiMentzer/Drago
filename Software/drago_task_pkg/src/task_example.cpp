#include <rclcpp/rclcpp.hpp>

// MoveIt Task Constructor
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

// Interfaces
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>


using namespace std::chrono_literals;

namespace mtc = moveit::task_constructor;

// ─── Poses (valores capturados do /joint_states) ──────────────────────────────

static const std::map<std::string, double> HOME_POSE = {
  {"Effector_joint",  0.0},
  {"Elbow_joint",     0.0},
  {"Shoulder_joint",  0.0},
  {"Waist_joint",     0.0},
  {"Wrist_1_joint",   0.0},
  {"Wrist_2_joint",   0.0},
};

static const std::map<std::string, double> POSE_1 = {
  {"Effector_joint",   0.07131172137029128},
  {"Elbow_joint",     -0.57679175772765560},
  {"Shoulder_joint",   1.71900964410471700},
  {"Waist_joint",     -0.13226386242699179},
  {"Wrist_1_joint",    0.16000000000000000},
  {"Wrist_2_joint",   -0.87257831264599860},
};

static const std::map<std::string, double> POSE_2 = {
  {"Effector_joint",  -0.04663721164100924},
  {"Elbow_joint",      0.06478393407149197},
  {"Shoulder_joint",   1.67032857361401190},
  {"Waist_joint",     -0.27578970385896740},
  {"Wrist_1_joint",    0.25967893031562345},
  {"Wrist_2_joint",   -1.29995189205262560},
};

static const std::map<std::string, double> POSE_3 = {
  {"Effector_joint",  -0.0},
  {"Elbow_joint",     -0.0},
  {"Shoulder_joint",   0.0},
  {"Waist_joint",     -1.3501416095186876},
  {"Wrist_1_joint",   -0.0},
  {"Wrist_2_joint",   -0.0},
};

static const std::map<std::string, double> POSE_4 = {
  {"Effector_joint",  -1.2615094945042187},
  {"Elbow_joint",     -0.16769555624172808},
  {"Shoulder_joint",   0.7239474853123576},
  {"Waist_joint",     -1.2296439244381498},
  {"Wrist_1_joint",   -1.2931961960031597},
  {"Wrist_2_joint",   -0.6852886807959193},
};

static const std::map<std::string, double> POSE_5 = {
  {"Effector_joint",   0.7068186055547715},
  {"Elbow_joint",     -0.2888488092974806},
  {"Shoulder_joint",   0.789206667131627},
  {"Waist_joint",     -1.5625253764604938},
  {"Wrist_1_joint",    0.8694797001538491},
  {"Wrist_2_joint",   -0.21815089211906577},
};


// ─── Helper ───────────────────────────────────────────────────────────────────
static std::unique_ptr<mtc::stages::MoveTo>
makeMoveToJoints(const std::string& name,
                 const std::string& group,
                 const std::map<std::string, double>& joint_values)
{
  auto solver = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  solver->setMaxVelocityScalingFactor(1);
  solver->setMaxAccelerationScalingFactor(1);

  auto stage = std::make_unique<mtc::stages::MoveTo>(name, solver);
  stage->setGroup(group);
  stage->setGoal(joint_values);
  return stage;
}

static std::unique_ptr<mtc::stages::MoveRelative>
makeMoveRotation(const std::string& name, 
                 const std::string& group, 
                 const std::string& frame,
                       char         axis,   
                       float        angle)
{
  auto cartesian_solver = std::make_shared<mtc::solvers::CartesianPath>();  
  cartesian_solver->setMaxVelocityScalingFactor(1);
  cartesian_solver->setMaxAccelerationScalingFactor(1);
  cartesian_solver->setStepSize(0.010);       // passo de 10 mm
  cartesian_solver->setMinFraction(0.0);      // exige 0% do caminho viável

  auto rotation_stage = std::make_unique<mtc::stages::MoveRelative>(name, cartesian_solver);
  rotation_stage->setGroup(group);
  rotation_stage->setIKFrame(frame);

  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;      // eixo no frame do end-effector

  if(axis=='x') {twist.twist.angular.x = angle;}
  if(axis=='y') {twist.twist.angular.y = angle;}
  if(axis=='z') {twist.twist.angular.z = angle;}

  rotation_stage->setDirection(twist);
  return rotation_stage;
}


// ─── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("task_example", options);
  auto logger = node->get_logger();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // Aguarda move_group e execute_task_solution estarem prontos
  RCLCPP_INFO(logger, "Aguardando move_group ficar pronto (5s)...");
  rclcpp::sleep_for(5s);

  // ── Task ──────────────────────────────────────────────────────────────────
  auto task_name = "Routine example";

  mtc::Task task;
  task.stages()->setName(task_name);
  task.loadRobotModel(node);

  // ⚠️  Ajuste para o nome real do seu grupo de planejamento
  const std::string ARM_GROUP = "Drago";
  const std::string EEF_FRAME = "Effector_link";

  // ── Loop de repetição (Ctrl+C para parar) ────────────────────────────────
  int ciclo = 1;
  while (rclcpp::ok()) {
    RCLCPP_INFO(logger, "--- Ciclo %d ---", ciclo++);

    // Replaneja a task do zero a cada ciclo
    mtc::Task loop_task;
    loop_task.stages()->setName("task_name");
    loop_task.loadRobotModel(node);

    loop_task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    loop_task.add(makeMoveToJoints("Go to Home",   ARM_GROUP, HOME_POSE));
    loop_task.add(makeMoveToJoints("Go to pose 1",  ARM_GROUP, POSE_1));
    loop_task.add(makeMoveToJoints("Go to pose 2",  ARM_GROUP, POSE_2));
    loop_task.add(makeMoveToJoints("Go to pose 3",  ARM_GROUP, POSE_3));
    loop_task.add(makeMoveRotation("Rotação em Z",  ARM_GROUP, EEF_FRAME,'z', M_PI/4));
    loop_task.add(makeMoveRotation("Rotação em Z",  ARM_GROUP, EEF_FRAME, 'z', -M_PI/4));
    loop_task.add(makeMoveToJoints("Go to pose 4",  ARM_GROUP, POSE_4));
    loop_task.add(makeMoveToJoints("Go to pose 5",  ARM_GROUP, POSE_5));
    loop_task.add(makeMoveToJoints("Back to Home", ARM_GROUP, HOME_POSE));

    try {
      loop_task.init();
    } catch (const mtc::InitStageException& e) {
      RCLCPP_ERROR(logger, "task.init() falhou no ciclo %d: %s", ciclo, e.what());
      break;
    }

    if (!loop_task.plan(3)) {
      RCLCPP_ERROR(logger, "Planejamento falhou no ciclo %d", ciclo);
      break;
    }

    auto loop_result = loop_task.execute(*loop_task.solutions().front());
    if (loop_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(logger, "Execucao falhou no ciclo %d (codigo %d)",
                   ciclo, loop_result.val);
      break;
    }

    RCLCPP_INFO(logger, "Ciclo %d concluido. Aguardando 1s...", ciclo - 1);
    rclcpp::sleep_for(1s);
  }

  RCLCPP_INFO(logger, "No encerrado.");
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}