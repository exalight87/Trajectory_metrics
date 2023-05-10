#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <array>

struct Point{
    int x, y, t;

    double distance_from(const Point& p) const
    {
        return std::sqrt(std::pow(p.x - x, 2) + std::pow(p.y - y, 2));
    }
};

std::vector<std::vector<Point>> read_file(const std::string& filepath)
{
    std::ifstream ifs(filepath, std::ios::in);

    if(!ifs)
    {       
        throw std::runtime_error(filepath + ": " + std::strerror(errno));
    }

    std::vector<std::vector<Point>> trajectories;

    std::string trajectory_count_str;
    std::getline(ifs, trajectory_count_str, ' ');

    int trajectory_count = std::stoi(trajectory_count_str);
    trajectories.reserve( trajectory_count );

    for(int traj_idx = 0; traj_idx < trajectory_count; ++traj_idx)
    {
        std::string point_count_str;
        std::getline(ifs, point_count_str, ' ');

        int point_count = std::stoi(point_count_str);

        std::vector<Point> trajectory;
        trajectory.reserve(point_count);

        for(int pt_idx = 0; pt_idx < point_count; ++pt_idx)
        {
            std::string x_str;
            std::getline(ifs, x_str, ' ');
            std::string y_str;
            std::getline(ifs, y_str, ' ');
            std::string t_str;
            std::getline(ifs, t_str, ' ');

            trajectory.push_back( Point{std::stoi(x_str), std::stoi(y_str), std::stoi(t_str)} );
        }

        std::sort(trajectory.begin(), trajectory.end(), [](const Point& p1, const Point& p2) { return p1.t < p2.t; });
        trajectories.push_back(trajectory);
    }

    return std::move(trajectories);

}

double length(const std::vector<Point>& trajectory)
{
    double length = 0;

    for(int i = 0; i < trajectory.size() - 1; ++i)
    {
        length += trajectory[i].distance_from(trajectory[i+1]);
    }

    return length;
}

std::vector<std::map<double, int>> compute_length_classification(const std::vector<std::vector<Point>>& trajectories, const std::vector<double>& lengths)
{
    std::vector<std::map<double, int>> length_diff_classifications;
    length_diff_classifications.reserve(trajectories.size());

    for(int i = 0; i < trajectories.size(); ++i)
    {
        std::map<double, int> length_diff_classification;
        for(int j = 0; j < trajectories.size(); ++j)
        {
            if(i==j) 
            {
                continue;
            }
            length_diff_classification[std::abs(lengths[i] - lengths[j])] = j;
        }
        length_diff_classifications.push_back(length_diff_classification);
    }

    return std::move(length_diff_classifications);
}

double speed(const std::vector<Point>& trajectory, double distance)
{    
    double diff_time = trajectory.rend()->t - trajectory.begin()->t;
    return (diff_time == 0.0 || distance == 0.0) ? 0.0 : diff_time / distance;
}

std::vector<std::map<double, int>> compute_speed_classification(const std::vector<std::vector<Point>>& trajectories, const std::vector<double>& lengths)
{
    std::vector<double> speeds(trajectories.size(), 0);
    for(int i = 0; i < trajectories.size(); ++i)
    {
        speeds[i] = speed(trajectories[i], lengths[i]);
    }

    std::vector<std::map<double, int>> speed_diff_classifications;
    speed_diff_classifications.reserve(trajectories.size());

    for(int i = 0; i < trajectories.size(); ++i)
    {
        std::map<double, int> speed_diff_classification;
        for(int j = 0; j < trajectories.size(); ++j)
        {
            if(i==j) 
            {
                continue;
            }
            speed_diff_classification[std::abs(speeds[i] - speeds[j])] = j;
        }
        speed_diff_classifications.push_back(speed_diff_classification);
    }

    return std::move(speed_diff_classifications);
}

auto load_data(const std::string& filename)
{
    auto trajectories = read_file(filename);

    std::vector<double> lengths(trajectories.size(), 0);
    for(int i = 0; i < trajectories.size(); ++i)
    {
        lengths[i] = length(trajectories[i]);
    }

    return std::move(std::make_pair(
        compute_length_classification(trajectories, lengths), 
        compute_speed_classification(trajectories, lengths)));
}

template <size_t S>
std::array<int, S> getClosest(std::map<double, int> diff_classifications)
{
    std::array<int, S> closest{-1, -1, -1};
    int count = 0;
    for( const auto& [diff, traj] : diff_classifications)
    {
        closest[count] = traj;

        if( ++count >= S )
        {
            break;
        }
    }

    return std::move(closest);
}

enum Metric 
{
    Unknown,
    Length,
    Speed,
    EnumSize
};

std::istream& operator>>(std::istream& is, Metric& metric)
{
    int m;
    is >> m;
    metric = static_cast<Metric>(m);

    return is;
}

std::string help_menu_str = R"(
    -h : show help menu
    --filename <path> : specify the path of the loaded file
    --showClassifications : debug command to show the classifcations
)";

int main(int argc, char *argv[])
{
    if(argc == 1)
    {
        std::cout << help_menu_str << '\n';
        return 0;
    }

    std::string filename;
    bool showClassification = false;

    // read parameters
    for (int i = 1; i < argc; ++i)
    {
        std::string arg{argv[i]};

        if( arg == "--filename" )
        {
            filename = argv[++i];
        }
        else if( arg == "--showClassifications" )
        {
            showClassification = true;
        }
        else
        {
            std::cout << help_menu_str << '\n';
            return 0;
        }
    }
    
    auto [length_diff_classifications, speed_diff_classifications] = load_data(filename);

    // CLI menu and action loop
    while(true)
    {
        int traj = 0;
        Metric metric = Unknown;

        std::cout << "Please select trajectory and metric.\n";
        std::cout << "  Trajectories [ 0 - "  << length_diff_classifications.size() <<"] : ";
        std::cin >> traj;
        std::cout << "  Metrics ( Length: 1, Speed: 2 ) : ";
        std::cin >> metric;

        if(traj < 0 || traj >= length_diff_classifications.size() || metric <= Unknown || metric >= EnumSize)
        {
            std::cout << "Trajectory or Metric have bad values\n";
            continue;
        }

        if(metric == Length)
        {
            std::cout << "Closest trajectories from trajectory " << traj << " based on length \n";
            for(const int& traj_idx : getClosest<3>(length_diff_classifications.at(traj)))
            {
                if(traj_idx == -1) {break;}
                std::cout << traj_idx << " ";
            }
            std::cout << '\n';
        }
        else if( metric == Speed )
        {
            std::cout << "Closest trajectories from trajectory " << traj << " based on speed \n";
            for(const int& traj_idx : getClosest<3>(speed_diff_classifications.at(traj)))
            {
                if(traj_idx == -1) {break;}
                std::cout << traj_idx << " ";
            }
            std::cout << '\n';
        }
        
        // Debug prompt if needed
        if (showClassification)
        {
            std::cout << "Lengths :\n";
            for(int i = 0; i < length_diff_classifications.size(); ++i)
            {
                std::cout<< "traj[" << i << "] : ";
                for(const auto& [lenght_diff, traj] : length_diff_classifications[i])
                {
                    std::cout << lenght_diff << " (" << traj << "), ";
                }
                std::cout << '\n';
            }

            std::cout << "Speeds :\n";
            for(int i = 0; i < speed_diff_classifications.size(); ++i)
            {
                std::cout<< "traj[" << i << "] : ";
                for(const auto& [speed_diff, traj] : speed_diff_classifications[i])
                {
                    std::cout << speed_diff << " (" << traj << "), ";
                }
                std::cout << '\n';
            }
        }
    }

    return 0;
}