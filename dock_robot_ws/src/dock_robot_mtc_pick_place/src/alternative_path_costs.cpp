#include "dock_robot_mtc_pick_place/alternative_path_costs.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/cost_terms.h>
#include <iostream>

using namespace moveit::task_constructor;

int main(int argc,char ** argv)
{
    rclcpp::init(argc, argv);
    std::cout << "ROS2 initialized successfully!" << std::endl;
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("alternative_path_costs_node", node_options);
    std::cout << "Node created successfully!" << std::endl;

    std::thread spinning_thread([node]{rclcpp::spin(node);});

    Task t;
    t.stages()->setName("alternative_path_costs");
    t.enableIntrospection();
    t.publishAllSolutions(true);
    std::cout << "Task created successfully!" << std::endl;

    t.loadRobotModel(node, "robot_description");
    std::cout << "Loaded robot model: " << t.getRobotModel()->getName() << std::endl;

    assert(t.getRobotModel()->getName() == "my_robot");

    auto scene{std::make_shared<planning_scene::PlanningScene>(t.getRobotModel())};
    std::cout << "Planning scene created successfully!" << std::endl;

    auto& robot_state{scene->getCurrentStateNonConst()};
    robot_state.setToDefaultValues();
    robot_state.setToDefaultValues(robot_state.getJointModelGroup("left_arm"), "left_arm_home");
    std::cout << "Initialized robot state in planning scene to default values and 'home' position" << std::endl;

    auto initial {std::make_unique<stages::FixedState>("start")};
    initial->setState(scene);
    t.add(std::move(initial));
    std::cout << "Added initial stage with fixed state" << std::endl;

    auto pipeline{std::make_shared<solvers::PipelinePlanner>(node)};
    std::cout << "Pipeline planner created successfully!" << std::endl;

    auto alternatives{std::make_unique<Alternatives>("connect")};
    std::cout << "Alternatives container created successfully!" << std::endl;

    // Strategy 1: Minimize path length
    {
        auto connect{std::make_unique<stages::Connect>
            ("path length", stages::Connect::GroupPlannerVector{{"left_arm", pipeline}})};
        connect->setCostTerm(std::make_unique<cost::PathLength>());
        alternatives->add(std::move(connect));
        std::cout << "Added 'path length' strategy" << std::endl;
    }
    // Strategy 2: Minimize trajectory duration
    {
        auto connect{std::make_unique<stages::Connect>
            ("trajectory duration", stages::Connect::GroupPlannerVector{{"left_arm", pipeline}})};
        connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());
        alternatives->add(std::move(connect));
        std::cout << "Added 'trajectory duration' strategy" << std::endl;
    }


    // Strategy 3: Minimize end-effector motion
    {
        auto connect{std::make_unique<stages::Connect>
            ("end-effector motion", stages::Connect::GroupPlannerVector{{"left_arm", pipeline}})};
        connect->setCostTerm(std::make_unique<cost::LinkMotion>("link5"));
        alternatives->add(std::move(connect));
        std::cout << "Added 'end-effector motion' strategy" << std::endl;
    }

    // Strategy 4: Minimize elbow motion
    {
        auto connect{std::make_unique<stages::Connect>
            ("elbow motion", stages::Connect::GroupPlannerVector{{"left_arm", pipeline}})};
        connect->setCostTerm(std::make_unique<cost::LinkMotion>("link4"));
        alternatives->add(std::move(connect));
        std::cout << "Added 'elbow motion' strategy" << std::endl;
    }

    t.add(std::move(alternatives));
    std::cout << "Added all strategies in the Alternatives Container to the task" << std::endl;

    auto goal_scene{scene->diff()};
    goal_scene->getCurrentStateNonConst().setToDefaultValues(robot_state.getJointModelGroup("left_arm"), "left_arm_pose1");
    std::cout << "Created goal scene with 'left_arm_pose1' position" << std::endl;

    auto goal{std::make_unique<stages::FixedState>("goal")};
    goal->setState(goal_scene);
    t.add(std::move(goal));
    std::cout << "Added goal state to the task" << std::endl;

    std::cout << "Planning..." << std::endl;

    try {
        auto result = t.plan(0);

        if (t.numSolutions() == 0) {
            std::cout << "Task planning finished, but no solutions found." << std::endl;
            t.printState();
            return 1;
        }

        std::cout << "Task planning completed successfully" << std::endl;
        std::cout << "Solution count: " << t.numSolutions() << std::endl;

        std::cout << "Planning results:" << std::endl;
        t.printState();

        // 给 RViz 一点订阅时间
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // 推荐：发布所有方案给 RViz Motion Planning Tasks
        // t.publishAllSolutions(true);

        // 如果 Humble 里 t.publishAllSolutions(true) 编译不过，就用下面这一行替代：
        t.introspection().publishAllSolutions(true);

        std::cout << "Published all MTC solutions to RViz /solution topic" << std::endl;
    }
    catch (const InitStageException& e) {
        std::cout << "Task planning failed: " << e << std::endl;
    }

    // Keep the node alive for interactive inspection in RViz
    std::cout << "Keeping node alive for RViz inspection. Press Ctrl+C to exit." << std::endl;
    spinning_thread.join();
    return 0;
}