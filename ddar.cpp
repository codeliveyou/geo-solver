#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <pybind11/iostream.h>
namespace py = pybind11;

std::tuple <
    std::vector<std::map<std::string, std::vector<std::vector<py::object>>>>,
    std::vector<std::map<std::string, std::vector<std::vector<py::object>>>>,
    std::vector<int>,
    std::vector<py::object>
> saturate_or_goal(
    py::object g,
    std::map<std::string, py::object> theorems,
    std::vector<float> level_times,
    py::object p,
    int max_level = 100,
    int timeout = 600
) {
    py::module dd = py::module::import("dd");
    py::function bfs_one_level = dd.attr("bfs_one_level");
    std::vector<std::map<std::string, std::vector<std::vector<py::object>>>> derives;
    std::vector<std::map<std::string, std::vector<std::vector<py::object>>>> eq4s;
    std::vector<int> branching;
    std::vector<py::object> all_added;

    while(level_times.size() < max_level) {
        int level = level_times.size() + 1;
        auto start = std::chrono::steady_clock::now();
        auto [added, derv, eq4, n_branching] = bfs_one_level(g, theorems, level, p, false, true, timeout).cast<std::tuple<
            std::vector<py::object>,
            std::map<std::string, std::vector<std::vector<py::object>>>,
            std::map<std::string, std::vector<std::vector<py::object>>>,
            int
        >>();
        all_added.insert(all_added.end(), added.begin(), added.end());
        branching.push_back(n_branching);

        derives.push_back(derv);
        eq4s.push_back(eq4);

        auto end = std::chrono::steady_clock::now();
        float level_time = std::chrono::duration<float>(end - start).count();
        std::cout << "Depth " << level << "/" << max_level << " time = " << level_time << std::endl;

        level_times.push_back(level_time);

        if(!p.attr("goal").is_none()) {
            ////////////////////////////////////
            std::vector<py::object> args = p.attr("goal").attr("args").cast<std::vector<py::object>>();
            std::vector<py::object> goal_args;
            ////////////////////////////////////
            for(auto x : args) {
                goal_args.push_back(g.attr("get")(x.cast<std::string>()).cast<py::object>());
            }
            if(!g.attr("check")(p.attr("goal").attr("name"), goal_args).is_none()
                && g.attr("check")(p.attr("goal").attr("name"), goal_args).cast<bool>()) {
                break;
            }
        }

        if(added.empty()) {
            break;
        }

        if(level_time > timeout) {
            break;
        }
    }
    return {derives, eq4s, branching, all_added};
}

std::tuple <
    py::object,
    std::vector<float>,
    std::string,
    std::vector<int>,
    std::vector<py::object>
> solve(
    py::object g,
    std::map<std::string, py::object> theorems,
    py::object controller,
    int max_level = 1000,
    int timeout = 600
) {
    py::module dd = py::module::import("dd");
    
    py::function derive_algebra = g.attr("derive_algebra");
    py::function names2points = g.attr("names2points");
    py::function apply_derivations = dd.attr("apply_derivations");

    std::string status = "saturated";
    std::vector<float> level_times;

    auto [dervs, eq4] = derive_algebra(0, false).cast<
        std::tuple<
            std::map<std::string, std::vector<std::vector<py::object>>>,
            std::map<std::string, std::vector<std::vector<py::object>>>
        >
    >();

    std::vector<std::map<std::string, std::vector<std::vector<py::object>>>> derives = {dervs};
    std::vector<std::map<std::string, std::vector<std::vector<py::object>>>> eq4s = {eq4};
    std::vector<int> branches;
    std::vector<py::object> all_added;

    while(level_times.size() < max_level) {
        auto [new_dervs, new_eq4, next_branches, added] = saturate_or_goal(
            g, theorems, level_times, controller, max_level, timeout
        );

        all_added.insert(all_added.end(), added.begin(), added.end());

        derives.insert(derives.end(), new_dervs.begin(), new_dervs.end());
        eq4s.insert(eq4s.end(), new_eq4.begin(), new_eq4.end());
        branches.insert(branches.end(), next_branches.begin(), next_branches.end());

        if(!controller.attr("goal").is_none()) {
            std::cout << controller.attr("goal").attr("args").cast<std::vector<py::object>>().size() << std::endl;
            auto goal_args = names2points(controller.attr("goal").attr("args"));
            if(g.attr("check")(controller.attr("goal").attr("name"), goal_args)) {
                status = "solved";
                break;
            }
        }

        if(derives.empty()) {
            break;
        }

        std::vector<py::object> new_added;
        while(!derives.empty() && new_added.empty()) {
            auto current = derives.front();
            derives.erase(derives.begin());
            std::vector<py::object> applied = apply_derivations(g, current).cast<std::vector<py::object>>();
            new_added.insert(new_added.end(), applied.begin(), applied.end());
        }

        if(!new_added.empty()) {
            continue;
        }
        while(!eq4s.empty() && added.empty()) {
            auto current = eq4s.front();
            derives.erase(eq4s.begin());
            std::vector<py::object> applied = apply_derivations(g, current).cast<std::vector<py::object>>();
        }

        all_added.insert(all_added.end(), new_added.begin(), new_added.end());

        if(added.empty()) {
            break;
        }
    }

    return {g, level_times, status, branches, all_added};
}

auto get_proof_steps(
    py::object g,
    py::object goal,
    bool merge_trivials = false
) {
    py::module problem = py::module::import("problem");
    py::module trace_back = py::module::import("trace_back");
    py::function point_log = trace_back.attr("point_log");
    py::function get_logs = trace_back.attr("get_logs");

    auto goal_args = g.attr("names2nodes")(goal.attr("args"));
    py::object Dependency = problem.attr("Dependency");
    py::object query = Dependency(goal.attr("name").cast<std::string>(), goal_args);
    
    auto [setup, aux, log, setup_points] = get_logs(query, g, merge_trivials).cast<std::tuple<
        std::vector<py::object>,
        std::vector<py::object>,
        py::object,
        py::object
    >>();
    
    py::dict refs;
    std::vector<
        std::tuple<py::object, std::vector<py::object>>
    > setup_ = point_log(setup, refs, std::vector<py::object>()).cast<
        std::vector<std::tuple<py::object, std::vector<py::object>>>
    >();
    std::vector<
        std::tuple<py::object, std::vector<py::object>>
    > aux_ = point_log(aux, refs, setup_points).cast<
        std::vector<std::tuple<py::object, std::vector<py::object>>>
    >();
    
    std::vector<std::tuple<std::vector<py::object>, std::vector<py::object>>> _setup_, _aux_;
    for(auto [p, prems] : setup_) {
        std::vector<py::object> vec;
        vec.push_back(p);
        _setup_.push_back(std::make_tuple(prems, vec));
    }
    for(auto [p, prems] : aux_) {
        std::vector<py::object> vec;
        vec.push_back(p);
        _aux_.push_back(std::make_tuple(prems, vec));
    }
    return std::make_tuple(_setup_, _aux_, log, refs);
}

PYBIND11_MODULE(ddar, m) {
    m.def("saturate_or_goal", &saturate_or_goal,
        py::arg("g"),
        py::arg("theorems"),
        py::arg("level_times"),
        py::arg("p"),
        py::arg("max_level") = 1000,
        py::arg("timeout") = 600);
    m.def("solve", &solve,
        py::arg("g"),
        py::arg("theorems"),
        py::arg("controller"),
        py::arg("max_level") = 1000,
        py::arg("g") = 600);
    m.def("get_proof_steps", &get_proof_steps,
        py::arg("g"),
        py::arg("goal"),
        py::arg("merge_trivials") = false);
}