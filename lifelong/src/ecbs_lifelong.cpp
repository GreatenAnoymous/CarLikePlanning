#include "ecbs_lifelong.hpp"

#include <boost/program_options.hpp>

void test_demo()
{
    std::string tasks_file = "./test.json";
    size_t xmax, ymax;
    std::unordered_set<Location> obstacles;
    std::vector<std::vector<State>> tasks;
    std::vector<State> starts;
    std::vector<State> firstGoals;
    readMapsFromJson(tasks_file, xmax, ymax, starts, tasks, obstacles);
    for (size_t i = 0; i < tasks.size(); i++)
    {
        firstGoals.push_back(tasks[i][0]);
    }

    Environment environment(xmax, ymax, obstacles, firstGoals);
    ECBSLifelong ecbs(environment, tasks);
    ecbs.setInitialStates(starts);
    ecbs.sim();
    ecbs.dumpOutputToJson("./demo.json");
}

int ECBS_exe(int argc, char *argv[])
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    float w;
    desc.add_options()("help", "produce help message")(
        "input,i", po::value<std::string>(&inputFile)->required(),
        "input file (YAML)")("output,o",
                             po::value<std::string>(&outputFile)->required(),
                             "output file (YAML)")(
        "suboptimality,w", po::value<float>(&w)->default_value(1.0),
        "suboptimality bound");

    try
    {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help") != 0u)
        {
            std::cout << desc << "\n";
            return 0;
        }
    }
    catch (po::error &e)
    {
        std::cerr << e.what() << std::endl
                  << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }

    size_t dimx, dimy;

    std::unordered_set<Location> obstacles;

    std::vector<std::vector<State>> tasks;
    std::vector<State> startStates;
    readMapsFromJson(inputFile, dimx, dimy, startStates, tasks, obstacles);
    std::vector<State> firstGoals;
    for (size_t i = 0; i < tasks.size(); i++)
    {
        firstGoals.push_back(tasks[i][0]);
    }
    Environment environment(dimx, dimy, obstacles, firstGoals);
    ECBSLifelong ecbs(environment, tasks);
    ecbs.setInitialStates(startStates);
    ecbs.sim();
    ecbs.dumpOutputToJson(outputFile);
    return 0;
}

int main(int argc, char *argv[])
{
    ECBS_exe(argc, argv);
    return 0;
    // test_demo();
}