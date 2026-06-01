/**
 * MTC Demo Task — Exemplo de múltiplas capacidades do MoveIt Task Constructor
 *
 * Stages demonstradas:
 *   1. CurrentState          — captura o estado atual do robô
 *   2. MoveTo (junta)        — move para uma pose nomeada (ex: "ready")
 *   3. MoveRelative (cartesiano) — deslocamento linear no espaço cartesiano
 *   4. MoveRelative (rotação)    — rotação em torno de um eixo do end-effector
 *   5. Connect               — conecta duas stages com um planner (PipelinePlanner)
 *   6. ModifyPlanningScene   — anexa/desanexa um objeto de colisão
 *   7. GenerateGraspPose     — gera poses de grasp em torno de um objeto
 *   8. ComputeIK             — resolve a IK para uma pose cartesiana
 */

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace mtc = moveit::task_constructor;

static const std::string PLANNING_GROUP = "Drago";
static const std::string EEF_FRAME      = "Effector_joint";
static const std::string WORLD_FRAME    = "base_link";

// ──────────────────────────────────────────────────────────────────────────────
// Helpers para criar solvers reutilizáveis
// ──────────────────────────────────────────────────────────────────────────────

auto makePipelinePlanner(const rclcpp::Node::SharedPtr& node,
                         const std::string& pipeline = "ompl",
                         const std::string& planner  = "RRTConnectkConfigDefault")
{
  auto p = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  p->setProperty("planning_pipeline", pipeline);
  p->setProperty("planner_id",        planner);
  return p;
}

auto makeCartesianPlanner()
{
  auto p = std::make_shared<mtc::solvers::CartesianPath>();
  p->setMaxVelocityScalingFactor(1);
  p->setMaxAccelerationScalingFactor(1);
  p->setStepSize(0.005);       // passo de 5 mm
  p->setMinFraction(0.9);      // exige 90 % do caminho viável
  return p;
}

// ──────────────────────────────────────────────────────────────────────────────
// Função principal de construção da task
// ──────────────────────────────────────────────────────────────────────────────

mtc::Task buildTask(const rclcpp::Node::SharedPtr& node)
{
  mtc::Task task;
  task.stages()->setName("mtc_demo_task");
  task.loadRobotModel(node);

  // Propriedades compartilhadas por todas as stages
  task.setProperty("group",   PLANNING_GROUP);
  task.setProperty("eef",     "Effector_joint");         // nome do end-effector no SRDF
  task.setProperty("ik_frame", EEF_FRAME);

  auto pipeline   = makePipelinePlanner(node);
  auto cartesian  = makeCartesianPlanner();

  // ── 1. CurrentState ────────────────────────────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("estado_atual");
    task.add(std::move(stage));
  }

  // ── 2. MoveTo — pose nomeada ("ready") ─────────────────────────────────────
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("mover_para_ready", pipeline);
    stage->setGroup(PLANNING_GROUP);
    stage->setGoal("Pick position");   // pose nomeada no SRDF
    task.add(std::move(stage));
  }

  // ── 3. MoveRelative — deslocamento cartesiano em Z (+10 cm) ────────────────
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("deslocamento_z", cartesian);
    stage->setGroup(PLANNING_GROUP);
    stage->setIKFrame(EEF_FRAME);

    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = WORLD_FRAME;
    direction.vector.z = 0.8;   // 10 cm para cima
    stage->setDirection(direction);
    task.add(std::move(stage));
  }

  // ── 4. MoveRelative — rotação em torno do eixo Z do end-effector (45°) ─────
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("rotacao_z_eef", cartesian);
    stage->setGroup(PLANNING_GROUP);
    stage->setIKFrame(EEF_FRAME);

    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = EEF_FRAME;      // eixo no frame do end-effector
    twist.twist.angular.z = M_PI / 16.0;    // 45° em torno de Z
    stage->setDirection(twist);
    task.add(std::move(stage));
  }

  // ── 5. ModifyPlanningScene — adiciona cubo ao ambiente ─────────────────────
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("adicionar_cubo");
    stage->addObject([]() {
      moveit_msgs::msg::CollisionObject obj;
      obj.id               = "cubo_alvo";
      obj.header.frame_id  = WORLD_FRAME;
      obj.primitives.resize(1);
      obj.primitives[0].type               = shape_msgs::msg::SolidPrimitive::BOX;
      obj.primitives[0].dimensions         = {0.05, 0.05, 0.05};  // 5 cm³
      obj.primitive_poses.resize(1);
      obj.primitive_poses[0].position.x    = 0.4;
      obj.primitive_poses[0].position.y    = 0.0;
      obj.primitive_poses[0].position.z    = 0.5;
      obj.primitive_poses[0].orientation.w = 1.0;
      obj.operation = moveit_msgs::msg::CollisionObject::ADD;
      return obj;
    }());
    task.add(std::move(stage));
  }

  // ── 6. GenerateGraspPose + ComputeIK (wrapper container) ───────────────────
  //       Demonstra como gerar poses de grasp e resolver IK em torno de um objeto
  {
    // Container serial: gera a pose → resolve IK → aproxima
    auto grasp_container = std::make_unique<mtc::SerialContainer>("sequencia_grasp");
    task.properties().exposeTo(grasp_container->properties(),
                               {"eef", "group", "ik_frame"});

    // 6a. GenerateGraspPose
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("gerar_grasp");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->setPreGraspPose("open");
      stage->setObject("cubo_alvo");
      stage->setAngleDelta(M_PI / 8.0);   // amostras a cada 22.5°
      stage->setMonitoredStage(task.stages()->findChild("estado_atual"));
      grasp_container->insert(std::move(stage));
    }

    // 6b. ComputeIK ao redor das poses geradas
    {
      auto stage = std::make_unique<mtc::stages::ComputeIK>("ik_grasp",
        std::make_unique<mtc::stages::CurrentState>("ik_seed"));
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      stage->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      stage->setMaxIKSolutions(4);
      stage->setMinSolutionDistance(1.0);
      grasp_container->insert(std::move(stage));
    }

    task.add(std::move(grasp_container));
  }

  // ── 7. MoveRelative — aproximação cartesiana em -Z (pré-grasp) ─────────────
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("aproximar_grasp", cartesian);
    stage->setGroup(PLANNING_GROUP);
    stage->setIKFrame(EEF_FRAME);

    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = EEF_FRAME;
    dir.vector.z = -0.05;   // 5 cm na direção do approaching
    stage->setDirection(dir);
    task.add(std::move(stage));
  }

  // ── 8. MoveTo — retorno à pose "home" via planner de junta ─────────────────
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("retornar_home", pipeline);
    stage->setGroup(PLANNING_GROUP);
    stage->setGoal("Home position");
    task.add(std::move(stage));
  }

  return task;
}

// ──────────────────────────────────────────────────────────────────────────────
// main
// ──────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mtc_demo_node");

  // Spin em thread separada para não bloquear os callbacks do MoveIt
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = std::thread([&executor]() { executor.spin(); });

  try {
    auto task = buildTask(node);

    RCLCPP_INFO(node->get_logger(), "Inicializando task...");
    task.init();

    RCLCPP_INFO(node->get_logger(), "Planejando...");
    if (task.plan(5 /* soluções máximas */)) {
      RCLCPP_INFO(node->get_logger(), "Planejamento bem-sucedido!");
      task.introspection().publishSolution(*task.solutions().front());

      // Para executar de fato, descomente:
      // task.execute(*task.solutions().front());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Falha no planejamento.");
    }
  } catch (const mtc::InitStageException& e) {
    RCLCPP_ERROR(node->get_logger(), "InitStageException: %s", e.what());
  }

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}