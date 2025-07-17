#include <amra/interface.h>
#include <amra/grid2d.hpp>
#include <amra/helpers.hpp>
#include <chrono>

int plan_2d(
    std::vector<float>& origin,
    std::vector<int>& dim,
    std::vector<signed char>& map_data,
    std::vector<float>& start_f,
    std::vector<float>& goal_f,
    float resolution,
    std::vector<std::vector<double>>& path,
    double& time_spent)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    int width = dim[0];
    int height = dim[1];

    AMRA::Grid2D grid(width, height, map_data);
    grid.CreateSearch();

    int start_x = (start_f[0] - origin[0]) / resolution;
    int start_y = (start_f[1] - origin[1]) / resolution;
    int goal_x = (goal_f[0] - origin[0]) / resolution;
    int goal_y = (goal_f[1] - origin[1]) / resolution;

    grid.SetStart(start_x, start_y);
    grid.SetGoal(goal_x, goal_y);

    bool success = grid.Plan(false);

    if (success)
    {
        std::vector<int> solution_i;
        std::vector<int> action_ids;
        int solcost;
        grid.GetSolution(solution_i, action_ids, solcost);

        for (const auto& state_id : solution_i)
        {
            AMRA::MapState state;
            grid.GetStateFromID(state_id, state);
            std::vector<double> point;
            point.push_back(state.coord[0] * resolution + origin[0]);
            point.push_back(state.coord[1] * resolution + origin[1]);
            path.push_back(point);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();

    return success ? 1 : 0;
}
