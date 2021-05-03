#include <cstdio>
#include <chrono>
#include <octomap/OcTree.h>
#include <boost/program_options.hpp> // program cli options
#include <boost/filesystem.hpp> // loading tree files

#include "gp_engine.cpp"
#include "gp_utils.cpp"

//using namespace std::chrono_literals;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// declare cli options handlers
po::options_description desc("Allowed options");
po::variables_map vm;

int main(int argc, char *argv[])
{   
    // * Add CLI options
    desc.add_options()
    ("help", "Print this message. Options declared will be serially executed in the order listed here")
    ("target", po::value<std::string>(), "Target tree filepath")
    ("gripper", po::value<std::string>(), "Gripper tree filepath")
    ("use_simple_gripper", "Use a simple gripper model instead of importing a gripper tree")
    ("write_target", po::value<std::string>()->implicit_value("target.ot"), "Write target octree to file")
    ("write_gripper", po::value<std::string>()->implicit_value("gripper.ot"), "Write gripper octree to file")
    ("write_color_target", po::value<std::string>()->implicit_value("colortree_target.ot"), "Write to file ColorOcTree version of target octree")
    ("write_color_gripper", po::value<std::string>()->implicit_value("colortree_gripper.ot"), "Write to file ColorOcTree version of gripper octree")
    ("gp_algorithm", po::value<unsigned int>(), "Select grasp planning algorithm to use in idx range [1-4]")
    ("global_analysis", "Perform a global graspability analysis")
    ("local_analysis", po::value<std::vector<float>>(), "Perform a local analysis at a defined target 3D point. Pass arg with no spaces and with equal sign (i.e. --local_analysis={x,y,z}) [m]")
    ;

    // * Parse commandline
    po::store(po::parse_command_line(argc,argv,desc), vm);
    po::notify(vm);
    if (vm.count("help") || argc == 1) // early exit for help CLI option
    {
        std::cout << desc << "\n";
        return 1;
    }

    // * Initialise objects
    // Grasp Quality Map object handle and declare gp algorithm select
    graspQualityMap gqm{0.01};
    unsigned int gp_algorithm_select;

    // Graspable gripper BBX definition and grasping normal vector // TODO Find a way to optionally pass this as CLI args
    const octomap::point3d min_point3d{-0.04,0.113,0.083}; // xyz min
    const octomap::point3d max_point3d{0.042,0.152,0.104}; // xyz max
    const octomap::point3d gripper_normal{0,1,0}; // y-axis points towards target surface normal in 2F85 gripper model

    // *** Parse CLI args
    // * Options for loading objects to memory
    if (vm.count("target"))
    {
        std::string target_filepath{vm["target"].as<std::string>()};
        if (fs::exists(target_filepath) && fs::is_regular_file(target_filepath))
        {
            if (fs::extension(target_filepath) == ".bt")
            {
                octomap::OcTree* target_tree = new octomap::OcTree(target_filepath);
                target_tree->expand();
                gqm.set_target_tree(target_tree); 
            }
            else if (fs::extension(target_filepath) == ".ot")
            {
                octomap::OcTreeGraspQuality* target_tree = new octomap::OcTreeGraspQuality(target_filepath);
                target_tree->expand();
                gqm.set_target_tree(target_tree);
            }
            else
            {
                std::cerr << "Invalid target tree file extension '" << fs::extension(target_filepath) << "'" << std::endl;
                return 1;
            }
        }
        else
        {
            std::cerr << "Provided target tree filepath does not exist or is not a regular file" << std::endl;
            return 1;
        }
    }
    else if (vm.count("global_analysis") || vm.count("local_analysis") || vm.count("write_target") || vm.count("write_color_target")) // if target required, raise error
    {
        std::cerr << "Missing required '--target' CLI option" << std::endl;
        return 1;
    }
    if ((vm.count("gripper") + vm.count("use_simple_gripper")) == 1) // if and only if one option called exactly once
    {
        if (vm.count("gripper"))
        {
            std::string gripper_filepath{vm["gripper"].as<std::string>()};
            if (fs::exists(gripper_filepath) && fs::is_regular_file(gripper_filepath))
            {
                if (fs::extension(gripper_filepath) == ".bt")
                {
                    octomap::OcTree* gripper_tree = new octomap::OcTree(gripper_filepath);
                    gripper_tree->expand();
                    gqm.set_gripper_tree(gripper_tree, gripper_normal, min_point3d, max_point3d);
                }
                else if (fs::extension(gripper_filepath) == ".ot")
                {
                    octomap::OcTreeGripper* gripper_tree = new octomap::OcTreeGripper(gripper_filepath);
                    gripper_tree->expand();
                    gqm.set_gripper_tree(gripper_tree);
                }
                else
                {
                    std::cerr << "Invalid gripper tree file extension '" << fs::extension(gripper_filepath) << "'" << std::endl;
                    return 1;
                }
            }
            else
            {
                std::cerr << "Provided gripper tree filepath does not exist or is not a regular file" << std::endl;
                return 1;
            }
        }
        else
        {
            gqm.set_simple_gripper(gripper_normal, min_point3d, max_point3d);
        }
    }
    else if (vm.count("global_analysis") || vm.count("local_analysis") || vm.count("write_gripper") || vm.count("write_color_gripper")) // if gripper required, raise error
    {
        std::cerr << "Must use either '--gripper' or '--use_simple_gripper' CLI options" << std::endl;
        return 1;
    }
        
    // * Options for writing objects to file
    if (vm.count("write_target"))
    {
        std::string filepath{vm["write_target"].as<std::string>()};
        if (fs::extension(filepath) != ".ot") filepath += ".ot";
        gqm.get_target_tree()->write(filepath);
    }
    if (vm.count("write_gripper"))
    {
        std::string filepath{vm["write_gripper"].as<std::string>()};
        if (fs::extension(filepath) != ".ot") filepath += ".ot";
        gqm.get_gripper_tree()->write(filepath);
    }
    if (vm.count("write_color_target"))
    {
        std::string filepath{vm["write_color_target"].as<std::string>()};
        if (fs::extension(filepath) != ".ot") filepath += ".ot";
        ((octomap::ColorOcTree)*gqm.get_target_tree()).write(filepath);
    }
    if (vm.count("write_color_gripper"))
    {
        std::string filepath{vm["write_color_gripper"].as<std::string>()};
        if (fs::extension(filepath) != ".ot") filepath += ".ot";
        ((octomap::ColorOcTree)*gqm.get_gripper_tree()).write(filepath);
    }

    // * Options for analysing graspability
    if ((vm.count("global_analysis") + vm.count("local_analysis")) >= 1) // gp_algorithm must be provided if a grasp analysis option is used
    {
        if (vm.count("gp_algorithm")) gp_algorithm_select = vm["gp_algorithm"].as<unsigned int>();
        else
        {
            std::cerr << "Must provide option '--gp_algorithm'" << std::endl;
            return 1;
        }
    }
    if (vm.count("global_analysis"))
    {
        gqm.analyse_global_grasp_quality(gp_algorithm_select);

        // Write results to file
        gqm.get_target_tree()->writeGraspQualityHistogram("out_gq_histogram");
        gqm.get_target_tree()->write("out_target.ot");
        gqm.get_gripper_tree()->write("out_gripper.ot");
        ((octomap::ColorOcTree)*gqm.get_target_tree()).write("out_color_target.ot");
        ((octomap::ColorOcTree)*gqm.get_gripper_tree()).write("out_color_gripper.ot");

    }
    if (vm.count("local_analysis"))
    {
        std::vector<float> vf = vm["local_analysis"].as<std::vector<float>>();
        if (vf.size() != 3)
        {
            std::cerr << "3D coordinates incorrectly parsed. Use {x,y,z} without spaces (i.e. --local_analysis={-1.23,4.0,-49})" << std::endl;
            return 1;
        }
        octomap::point3d coord_node; // a good point is {-8.55646, -0.0842408, -1.43209};
        coord_node.x() = vf[0];
        coord_node.y() = vf[1];
        coord_node.z() = vf[2];
        octomap::OcTreeGraspQuality::iterator node_it{GraspPlanningUtils::searchIterator(coord_node, gqm.get_target_tree())};
        if (node_it == NULL) // if null ptr exit
        {
            std::cerr << "[local_analysis] Error: Point " << coord_node << " not in target tree.\n Exiting..." << std::endl;
            return -1;
        }
        Eigen::Affine3f T{gqm.analyse_local_grasp_quality(node_it, gp_algorithm_select)};
        gqm.write_grasp_visualisations(T);
    }
    return 0;
}