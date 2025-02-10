#include "KinematicChain.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/Cost.h>
#include <ompl/base/OptimizationObjective.h>
#include <memory>
#include <ompl/base/PlannerTerminationCondition.h>

Environment env;
int planner_id = 0;
bool bonus = false;
double goal_x, goal_y;


void makeScenario1(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{

    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -3;
    start[1] = -3;
    goal[0] = 2;
    goal[1] = 2;
    goal[2] = 0;
    goal[4] = -0.5 * M_PI;

    // Obstacle 1
    env.emplace_back(2, -1, 2.8, -1);
    env.emplace_back(2.8, -1, 2.8, 0.5);
    env.emplace_back(2.8, 0.5, 2, 0.5);
    env.emplace_back(2, 0.5, 2, -1);

    // Obstacle 2
    env.emplace_back(3.2, -1, 4, -1);
    env.emplace_back(4, -1, 4, 0.5);
    env.emplace_back(4, 0.5, 3.2, 0.5);
    env.emplace_back(3.2, 0.5, 3.2, -1);
}

void makeScenario2(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{
    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -4;
    start[1] = -4;
    start[2] = 0;
    goal[0] = 3;
    goal[1] = 3;
    goal[2] = 0;

    // Obstacle 1
    env.emplace_back(-1, -1, 1, -1);
    env.emplace_back(1, -1, 1, 1);
    env.emplace_back(1, 1, -1, 1);
    env.emplace_back(-1, 1, -1, -1);
}

void planScenario1(ompl::geometric::SimpleSetup &ss)
{
    // TODO: Plan for chain_box in the plane, and store the path in path1.txt.
    // The environment obstacles, start, and goal are already provided to you in makeScenario1.
    // Choose the most efficient planner from rrt, prm, or rrtconnect. to solve this problem.

    // auto planner = std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation(), true);
    // auto planner = std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation()); // not working properly
    auto planner = std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation());
    
    ss.setPlanner(planner);

    // Attempt to solve the planning problem with a 20-second timeout
    ompl::base::PlannerStatus solved = ss.solve(20.0);
    

    if (solved)
    {
        ss.simplifySolution();
        auto path = ss.getSolutionPath();

        // Reduce the number of states in the path using interpolation
        unsigned int numStates = 20; // Specify the desired number of interpolated states
        path.interpolate(numStates);
        std::cout << "Found solution!" << std::endl;

        // Simplify the path to remove unnecessary waypoints

        // Save the solution path to a file (path1.txt)
        std::ofstream outFile("path1.txt");
        path.printAsMatrix(outFile);

        outFile.close();

        // Optionally, visualize the solution path in the terminal
        ss.getSolutionPath().print(std::cout);
    }
    else
    {
        std::cout << "No solution found within the time limit." << std::endl;
    }
}

void benchScenario1(ompl::geometric::SimpleSetup &ss)
{
    // TODO: Benchmark PRM with uniform, bridge, gaussian, and obstacle-based Sampling. Do 20 trials with 20 seconds each

    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "ChainBox_Narrow");

    b.addPlanner(std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation()));

    int sampler_id = 0;

    while (sampler_id < 4)
    {
        if (sampler_id == 0)
        {
            // run all planners with a uniform valid state sampler on the benchmark problem
            ss.getSpaceInformation()->setValidStateSamplerAllocator(
                [](const ompl::base::SpaceInformation *si) -> ompl::base::ValidStateSamplerPtr
                {
                    return std::make_shared<ompl::base::UniformValidStateSampler>(si);
                });
            b.addExperimentParameter("sampler_id", "STRING", "Uniform Sampler");
            b.benchmark(request);
            b.saveResultsToFile();
        }

        if (sampler_id == 1)
        {
            // run all planners with a Gaussian valid state sampler on the benchmark problem
            ss.getSpaceInformation()->setValidStateSamplerAllocator(
                [](const ompl::base::SpaceInformation *si) -> ompl::base::ValidStateSamplerPtr
                {
                    return std::make_shared<ompl::base::GaussianValidStateSampler>(si);
                });
            b.addExperimentParameter("sampler_id", "STRING", "Gaussian Sampler");
            b.benchmark(request);
            b.saveResultsToFile();
        }

        if (sampler_id == 2)
        {
            // run all planners with a bridge-test valid state sampler on the benchmark problem
            ss.getSpaceInformation()->setValidStateSamplerAllocator(
                [](const ompl::base::SpaceInformation *si) -> ompl::base::ValidStateSamplerPtr
                {
                    return std::make_shared<ompl::base::BridgeTestValidStateSampler>(si);
                });
            b.addExperimentParameter("sampler_id", "STRING", "Bridge-Test Sampler");
            b.benchmark(request);
            b.saveResultsToFile();
        }

        if (sampler_id == 3)
        {
            // run all planners with a obstacle-based valid state sampler on the benchmark problem
            ss.getSpaceInformation()->setValidStateSamplerAllocator(
                [](const ompl::base::SpaceInformation *si) -> ompl::base::ValidStateSamplerPtr
                {
                    return std::make_shared<ompl::base::ObstacleBasedValidStateSampler>(si);
                });
            b.addExperimentParameter("sampler_id", "STRING", "Obstacle-Based Sampler");
            b.benchmark(request);
            b.saveResultsToFile();
        }

        sampler_id++;
    }
}

class ClearanceObjective : public ompl::base::StateCostIntegralObjective
{
public:
    Environment env_;
    ClearanceObjective(const ompl::base::SpaceInformationPtr &si, const Environment &env) :
        ompl::base::StateCostIntegralObjective(si, true),
        env_(env)
    {
    }

    ompl::base::Cost stateCost(const ompl::base::State *s) const override
    {
        // // Access the SE2 component of the compound state
        // const auto *st = s->as<ompl::base::CompoundState>();
        // double x = st->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
        // double y = st->as<ompl::base::SE2StateSpace::StateType>(0)->getY();

        // Call the const customClearance function
        return ompl::base::Cost(1 / (customClearance(s,si_) + 1e-6));
    }

    // Mark the function as const so it can be called from a const context
    double customClearance(const ompl::base::State* s, const ompl::base::SpaceInformationPtr& si) const
    {
        const auto *st = s->as<ompl::base::CompoundState>();
        double x = st->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
        double y = st->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
        double theta = st->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw();

        if (!bonus)
        {
            double min_dist = std::numeric_limits<double>::infinity();
            for (const auto &envSegment : env_)
            {
                double dist = sqrt(pow(x - envSegment.x0, 2) + pow(y - envSegment.y0, 2));
                if (dist < min_dist)
                {
                    min_dist = dist;
                }
            }
            return pow(min_dist, 1);
        }
        else
        {    
            double min_dist = std::numeric_limits<double>::infinity();

            // to create line segments for the box
            std::vector<Segment> boxSegments;
            double boxLinkLength = 1.0;
            double x1 = x + boxLinkLength / 2 * cos(theta) - boxLinkLength / 2 * sin(theta);
            double y1 = y + boxLinkLength / 2 * sin(theta) + boxLinkLength / 2 * cos(theta);
            double x2 = x - boxLinkLength / 2 * cos(theta) - boxLinkLength / 2 * sin(theta);
            double y2 = y - boxLinkLength / 2 * sin(theta) + boxLinkLength / 2 * cos(theta);
            double x3 = x - boxLinkLength / 2 * cos(theta) + boxLinkLength / 2 * sin(theta);
            double y3 = y - boxLinkLength / 2 * sin(theta) - boxLinkLength / 2 * cos(theta);
            double x4 = x + boxLinkLength / 2 * cos(theta) + boxLinkLength / 2 * sin(theta);
            double y4 = y + boxLinkLength / 2 * sin(theta) - boxLinkLength / 2 * cos(theta);
            boxSegments.emplace_back(x1, y1, x2, y2);
            boxSegments.emplace_back(x2, y2, x3, y3);
            boxSegments.emplace_back(x3, y3, x4, y4);
            boxSegments.emplace_back(x4, y4, x1, y1);

            // to create line segments for the chain
            std::vector<Segment> chainSegments;
            double chainLinkLength = 1.0;
            double xN, yN;
            double thetaN = theta;
            double x0 = x, y0 = y;
            chainSegments.reserve(5);
            for (unsigned int i = 0; i < 4; ++i)
            {
                thetaN += st->as<ompl::base::SO2StateSpace::StateType>(i + 1)->value;
                xN = x0 + cos(thetaN) * chainLinkLength;
                yN = y0 + sin(thetaN) * chainLinkLength;
                chainSegments.emplace_back(x0, y0, xN, yN);
                x0 = xN;
                y0 = yN;
            }

            for (const auto &boxSegment : boxSegments)
            {
                for (const auto &envSegment : env_)
                {
                    double dist = sqrt(pow(boxSegment.x0 - envSegment.x0, 2) + pow(boxSegment.y0 - envSegment.y0, 2));

                    if (dist < min_dist)
                    {
                        min_dist = dist;
                    }
                }
            }

            for (const auto &chainSegment : chainSegments)
            {
                for (const auto &envSegment : env_)
                {
                    double dist = sqrt(pow(chainSegment.x1 - envSegment.x0, 2) + pow(chainSegment.y1 - envSegment.y0, 2));

                    if (dist < min_dist)
                    {
                        min_dist = dist;
                    }
                }
            }
            return pow(min_dist, 1);
        }
    }
};

void customSimplifySolution(ompl::geometric::SimpleSetup &ss, double percentage)
{
    // Get the original solution path
    auto originalPath = ss.getSolutionPath();
    unsigned int originalSize = originalPath.getStateCount();
    unsigned int minRetainedStates = std::max(static_cast<unsigned int>(originalSize * percentage), 1u);

    // Create a new path to hold the simplified solution
    ompl::geometric::PathGeometric simplifiedPath(ss.getSpaceInformation());

    // Retain the first state
    simplifiedPath.append(originalPath.getState(0));

    double minSpacing = 3.0; // Minimum spacing between retained states

    // Variable to track the last added state in the simplified path
    const ompl::base::State* lastAddedState = originalPath.getState(0);

    for (unsigned int i = 1; i < originalSize; ++i)
    {
        const ompl::base::State* currentState = originalPath.getState(i);

        // Calculate the distance between last added state and the current state
        double distance = (ss.getSpaceInformation()->distance(lastAddedState, currentState));

        // If the current state is far enough from the last added state, add it
        if (distance >= minSpacing)
        {
            simplifiedPath.append(currentState);
            lastAddedState = currentState; // Update the last added state
        }
    }

    // Ensure we have at least the minimum required states
    while (simplifiedPath.getStateCount() < minRetainedStates && simplifiedPath.getStateCount() < originalSize)
    {
        simplifiedPath.append(originalPath.getState(simplifiedPath.getStateCount()));
    }
    simplifiedPath.append(originalPath.getState(originalSize - 1)); // Retain the last state

    // Replace the original path with the simplified one
    ss.getSolutionPath().clear(); // Clear the original path
    ss.getSolutionPath().append(simplifiedPath); // Append the simplified path
}

void planScenario2(ompl::geometric::SimpleSetup &ss)
{
    ss.setOptimizationObjective(std::make_shared<ClearanceObjective>(ss.getSpaceInformation(), env));
    
    auto planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());

    planner->setRange(5.0);
    planner->as<ompl::geometric::RRTstar>()->setGoalBias(0.05);
    
    ss.setPlanner(planner);

    ompl::base::PlannerStatus solved = ss.solve(60.0);

    if (solved)
    {
    
        std::cout << "Custom simplified solution path:" << std::endl;
        customSimplifySolution(ss, 0.3);
        // ss.simplifySolution();
        auto path = ss.getSolutionPath();
        unsigned int numStates = 200; // Specify the desired number of interpolated states
        path.interpolate(numStates);

        // Get a reference to the internal state vector
        auto &states = path.getStates();

        // Get the goal state (the last state in the path)
        ompl::base::State* goalState = states.back(); 

        // Check validity of interpolated states, remove invalid ones (except the goal state)
        auto si = ss.getSpaceInformation();
        for (unsigned int i = 0; i < states.size() - 1; /* no increment here */)  // Skip the last state
        {
            if (!si->isValid(states[i])) {
                std::cout << "Invalid state detected after interpolation at state: " << i << std::endl;
                
                // Remove the invalid state
                states.erase(states.begin() + i);

                // Do not increment i, as the new state at index i needs to be checked
            } else {
                ++i;
            }
        }

        std::ofstream outFile("path2.txt");
        path.printAsMatrix(outFile);
        outFile.close();

        // Solution path in the terminal
        ss.getSolutionPath().print(std::cout);
    }
    else
    {
        std::cout << "No solution found within the time limit." << std::endl;
    }
}


void benchScenario2(ompl::geometric::SimpleSetup &ss)
{
    // TODO: Benchmark RRT*, PRM*, RRT# for 10 trials with 60 secounds timeout.
    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "ChainBox_Clearance");

    b.addPlanner(std::make_shared<ompl::geometric::PRMstar>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));

    b.benchmark(request);
    b.saveResultsToFile();  
}

std::shared_ptr<ompl::base::CompoundStateSpace> createChainBoxSpace()
{ // TODO Create the Chainbox ConfigurationSpace

    auto space = std::make_shared<ompl::base::CompoundStateSpace>();

    // 1. Add a RealVectorStateSpace for the base position (x, y)
    auto baseSpace = std::make_shared<ompl::base::SE2StateSpace>();
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, -5.0);  // x lower bound
    bounds.setHigh(0, 5.0);  // x upper bound
    bounds.setLow(1, -5.0);  // y lower bound
    bounds.setHigh(1, 5.0);  // y upper bound
    bounds.setLow(2, -M_PI); // theta lower bound
    bounds.setHigh(2, M_PI); // theta upper bound
    baseSpace->setBounds(bounds);

    // 2. Add SO2StateSpaces for each of the 4 joints of the chain
    auto joint1Space = std::make_shared<ompl::base::SO2StateSpace>();
    auto joint2Space = std::make_shared<ompl::base::SO2StateSpace>();
    auto joint3Space = std::make_shared<ompl::base::SO2StateSpace>();
    auto joint4Space = std::make_shared<ompl::base::SO2StateSpace>();

    // Add all components to the compound state space
    space->addSubspace(baseSpace, 1.0);   // Base position
    space->addSubspace(joint1Space, 0.01); // Joint 1
    space->addSubspace(joint2Space, 0.01); // Joint 2
    space->addSubspace(joint3Space, 0.01); // Joint 3
    space->addSubspace(joint4Space, 0.01); // Joint 4

    // Optional: Lock the CompoundStateSpace to ensure all subspaces have weights
    space->lock();

    return space;
}

// to check if two segments intersect
bool intersectionCheck(const Segment &s0, const Segment &s1)
{
    double s10_x = s0.x1 - s0.x0;
    double s10_y = s0.y1 - s0.y0;
    double s32_x = s1.x1 - s1.x0;
    double s32_y = s1.y1 - s1.y0;

    double denom = s10_x * s32_y - s32_x * s10_y;
    if (denom == 0)
        return false; // Collinear

    bool denomPositive = denom > 0;

    double s02_x = s0.x0 - s1.x0;
    double s02_y = s0.y0 - s1.y0;
    double s_numer = s10_x * s02_y - s10_y * s02_x;
    if ((s_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
        return false; // No collision
    double t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
        return false; // No collision
    if (((s_numer - denom > -std::numeric_limits<float>::epsilon()) == denomPositive) ||
        ((t_numer - denom > std::numeric_limits<float>::epsilon()) == denomPositive))
        return false; // No collision
    return true;
}

// to check if the chain or box go out of bounds
bool environmentIntersectionCheck(const std::vector<Segment> &segments)
{
    // check if any part of segment is outside the environment's bounds'
    for (const auto &segment : segments)
    {
        if (segment.x0 < -5 || segment.x0 > 5 || segment.y0 < -5 || segment.y0 > 5 ||
            segment.x1 < -5 || segment.x1 > 5 || segment.y1 < -5 || segment.y1 > 5)
        {
            return true; // Intersection found
        }
    }

    return false; // No intersection found
}

// to check if the box center go out of bounds
bool environmentCenterCheck(const double x, const double y)
{
    // check if any part of segment is outside the environment's bounds'
    if (x < -5 || x > 5 || y < -5 || y > 5)
    {
        return true; // Intersection found
    }

    return false; // No intersection found
}

// to check if the chain or box intersects with the obstacles
bool obstacleIntersectionCheck(const std::vector<Segment> &segments, const Environment &env)
{
    for (const auto &segment : segments)
    {
        for (const auto &envSegment : env)
        {
            if (intersectionCheck(segment, envSegment))
            {
                return true; // Intersection found
            }
        }
    }

    return false; // No intersection found
}

// to check if the chain intersects with itself
bool selfIntersectionCheck(const std::vector<Segment> &segments)
{
    for (unsigned int i = 0; i < segments.size(); ++i)
    {
        for (unsigned int j = i + 1; j < segments.size(); ++j)
        {
            if (intersectionCheck(segments[i], segments[j]))
            {
                return true; // Intersection found
            }
        }
    }
    return false; // No intersection found
}

// to check if the chain intersects with box
bool chainBoxIntersectionCheck(const std::vector<Segment> &chains, const std::vector<Segment> &boxes)
{
    // Start the loop from the second element (index 1)
    for (size_t i = 1; i < chains.size(); ++i)
    {
        const auto &chainSegment = chains[i]; // Access the current chain segment
        for (const auto &boxSegment : boxes)
        {
            if (intersectionCheck(chainSegment, boxSegment))
            {
                return true; // Intersection found
            }
        }
    }
    return false; // No intersection found
}

bool isValidChain(const ompl::base::State *state, const Environment &env)
{
    bool valid = false;

    const auto *s = state->as<ompl::base::CompoundState>();
    double x = s->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
    double y = s->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
    double theta = s->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw();
    double j1 = s->as<ompl::base::SO2StateSpace::StateType>(1)->value;
    double j2 = s->as<ompl::base::SO2StateSpace::StateType>(2)->value;
    double j3 = s->as<ompl::base::SO2StateSpace::StateType>(3)->value;
    double j4 = s->as<ompl::base::SO2StateSpace::StateType>(4)->value;

    if (planner_id == 2)
    {

        double angleToGoal = atan2(goal_y - y, goal_x - x);
        double alignmentThreshold = 0.05; // radians
        double angularDifference = std::fabs(angleToGoal - theta);
        
        // Normalize angular difference to be within [-π, π]
        if (angularDifference > M_PI) {
            angularDifference = 2 * M_PI - angularDifference;
        }
        // If the robot is not facing the goal, allow only rotation
        if (angularDifference > alignmentThreshold)
        {
            // If the robot is too far from the goal position, restrict forward movement
            if (std::hypot(goal_x - x, goal_y - y) > 0.1) // Check distance from the goal
            {
                // Allow rotation only; restrict movement
                valid = true; // Consider the state valid for rotation
            }
        }

    }

    // to create line segments for the box
    std::vector<Segment> boxSegments;
    double boxLinkLength = 1.0;
    double x1 = x + boxLinkLength / 2 * cos(theta) - boxLinkLength / 2 * sin(theta);
    double y1 = y + boxLinkLength / 2 * sin(theta) + boxLinkLength / 2 * cos(theta);
    double x2 = x - boxLinkLength / 2 * cos(theta) - boxLinkLength / 2 * sin(theta);
    double y2 = y - boxLinkLength / 2 * sin(theta) + boxLinkLength / 2 * cos(theta);
    double x3 = x - boxLinkLength / 2 * cos(theta) + boxLinkLength / 2 * sin(theta);
    double y3 = y - boxLinkLength / 2 * sin(theta) - boxLinkLength / 2 * cos(theta);
    double x4 = x + boxLinkLength / 2 * cos(theta) + boxLinkLength / 2 * sin(theta);
    double y4 = y + boxLinkLength / 2 * sin(theta) - boxLinkLength / 2 * cos(theta);
    boxSegments.emplace_back(x1, y1, x2, y2);
    boxSegments.emplace_back(x2, y2, x3, y3);
    boxSegments.emplace_back(x3, y3, x4, y4);
    boxSegments.emplace_back(x4, y4, x1, y1);

    // to create line segments for the chain
    std::vector<Segment> chainSegments;
    double chainLinkLength = 1.0;
    double xN, yN;
    double thetaN = theta;
    double x0 = x, y0 = y;
    chainSegments.reserve(5);
    for (unsigned int i = 0; i < 4; ++i)
    {
        thetaN += s->as<ompl::base::SO2StateSpace::StateType>(i + 1)->value;
        xN = x0 + cos(thetaN) * chainLinkLength;
        yN = y0 + sin(thetaN) * chainLinkLength;
        chainSegments.emplace_back(x0, y0, xN, yN);
        x0 = xN;
        y0 = yN;
    }
    
    // Scenario 1:
    if(planner_id == 1)
    {
        return !(obstacleIntersectionCheck(chainSegments, env) || obstacleIntersectionCheck(boxSegments, env) || selfIntersectionCheck(chainSegments) || chainBoxIntersectionCheck(chainSegments, boxSegments) || environmentIntersectionCheck(chainSegments) || environmentIntersectionCheck(boxSegments));
    }
    // Scenario 2:
    else if(planner_id == 2 && valid)
    {
        return !(obstacleIntersectionCheck(chainSegments, env) || obstacleIntersectionCheck(boxSegments, env) || selfIntersectionCheck(chainSegments) || chainBoxIntersectionCheck(chainSegments, boxSegments) || environmentIntersectionCheck(boxSegments));
        // return !(obstacleIntersectionCheck(chainSegments, env) || obstacleIntersectionCheck(boxSegments, env) || selfIntersectionCheck(chainSegments) || chainBoxIntersectionCheck(chainSegments, boxSegments) || environmentCenterCheck(x, y));
    }
}

void setupCollisionChecker(ompl::geometric::SimpleSetup &ss, const Environment &env)
{
    ss.setStateValidityChecker([&env](const ompl::base::State *state) -> bool
                               { return isValidChain(state, env); });
}

int main(int argc, char **argv)
{

    int scenario;
    std::vector<double> startVec;
    std::vector<double> goalVec;
    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) Robot Reaching Task" << std::endl;
        std::cout << " (2) Robot Avoiding Task" << std::endl;
        std::cout << " (3) (BONUS) Robot Avoiding Task" << std::endl;

        std::cin >> scenario;
    } while (scenario < 1 || scenario > 4);

    switch (scenario)
    {
    case 1:
        planner_id = 1;
        makeScenario1(env, startVec, goalVec);
        break;
    case 2:
        planner_id = 2;
        makeScenario2(env, startVec, goalVec);
        break;
    case 3:
        planner_id = 2;
        makeScenario2(env, startVec, goalVec);
        break;
    default:
        std::cerr << "Invalid Scenario Number!" << std::endl;
    }

    auto space = createChainBoxSpace();
    ompl::geometric::SimpleSetup ss(space);

    goal_x = goalVec[0];
    goal_y = goalVec[1];

    setupCollisionChecker(ss, env);

    // setup Start and Goal
    ompl::base::ScopedState<> start(space), goal(space);
    space->setup();
    space->copyFromReals(start.get(), startVec);
    space->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);


    switch (scenario)
    {
    case 1:
        planScenario1(ss);
        benchScenario1(ss);
        break;
    case 2:
        planScenario2(ss);
        benchScenario2(ss);
        break;
    case 3:
        bonus = true;
        planScenario2(ss);
        break;
    default:
        std::cerr << "Invalid Scenario Number!" << std::endl;
    }
}
