#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <array>
#include <optional>
#include <chrono>

class Bench
{
public:
    Bench(std::string name) :
        _name(name),
        _begin(std::chrono::high_resolution_clock::now())
        {};

    ~Bench()
    {
        std::chrono::duration<double, std::milli> duration = std::chrono::high_resolution_clock::now() - _begin;
        std::cout << _name << " took : " << duration.count() << "ms" << std::endl;
    }

private:
    std::string _name;
    std::chrono::high_resolution_clock::time_point _begin;
};

struct Point{
    int x, y, t;

    double distance_from(const Point& p) const
    {
        return std::sqrt(std::pow(p.x - x, 2) + std::pow(p.y - y, 2));
    }
};

#define NB_NEIGHBOURS_WANTED 3

class Trajectory
{
public:
    Trajectory() = delete;
    Trajectory(uint32_t id, std::vector<Point>&& points) : 
        _id(id),
        _points(points),
        _length(std::nullopt),
        _duration(std::nullopt),
        _speed(std::nullopt)
    {
        _neighbours_length.fill({ -1.0, nullptr });
        _neighbours_speed.fill({ -1.0, nullptr });
    };

    double length()
    {
        if(!_length)
        {
            _length = 0;

            for(int i = 0; i < _points.size() - 1; ++i)
            {
                _length.value() += _points[i].distance_from(_points[i+1]);
            }
        }

        return _length.value();
    }

    double duration()
    {
        if(!_duration)
        {
            _duration =_points.back().t - _points.front().t;
        }

        return _duration.value();
    }

    double speed()
    {
        if(!_speed)
        {
            _speed = (duration() == 0.0 || length() == 0.0) ? 0.0 : length() / duration();
        }

        return _speed.value();
    }

    uint32_t id() const { return _id; };

    void update_neighbours(double length, double speed, Trajectory* neighbour)
    {
        auto shift_right = [](std::array< std::pair<double, Trajectory*>, NB_NEIGHBOURS_WANTED>& neighbours, uint32_t pos)
        {
            for (uint32_t i = neighbours.size() - 1; i > pos; --i)
            {
                neighbours[i] = neighbours[i - 1];
            }
        };

        for (uint32_t i = 0; i < _neighbours_length.size(); ++i)
        {
            if (_neighbours_length[i].first < length)
            {
                shift_right(_neighbours_length, i);
                _neighbours_length[i] = { length, neighbour };
                break;
            }
        }

        for (uint32_t i = 0; i < _neighbours_speed.size(); ++i)
        {
            if (_neighbours_speed[i].first < speed)
            {
                shift_right(_neighbours_speed, i);
                _neighbours_speed[i] = { speed, neighbour };
                break;
            }
        }
    }

    const auto& neighbours_length() const { return _neighbours_length; };
    const auto& neighbours_speed() const { return _neighbours_speed; };
private:
    uint32_t _id;
    std::vector<Point> _points;
    std::optional<double> _length;
    std::optional<double> _duration;
    std::optional<double> _speed;

    std::array< std::pair<double, Trajectory*>, NB_NEIGHBOURS_WANTED> _neighbours_length;
    std::array< std::pair<double, Trajectory*>, NB_NEIGHBOURS_WANTED> _neighbours_speed;
};

std::vector<Trajectory> read_file(const std::string& filepath)
{
    Bench bench("Read file");

    std::ifstream ifs(filepath, std::ios::in);

    if(!ifs)
    {       
        throw std::runtime_error(filepath + ": " + std::strerror(errno));
    }

    std::vector<Trajectory> trajectories;

    std::string trajectory_count_str;
    std::getline(ifs, trajectory_count_str, ' ');

    int trajectory_count = std::stoi(trajectory_count_str);
    trajectories.reserve( trajectory_count );

    for(uint32_t traj_idx = 0; traj_idx < trajectory_count; ++traj_idx)
    {
        std::string point_count_str;
        std::getline(ifs, point_count_str, ' ');

        int point_count = std::stoi(point_count_str);

        std::vector<Point> points;
        points.reserve(point_count);

        for(uint32_t pt_idx = 0; pt_idx < point_count; ++pt_idx)
        {
            std::string x_str;
            std::getline(ifs, x_str, ' ');
            std::string y_str;
            std::getline(ifs, y_str, ' ');
            std::string t_str;
            std::getline(ifs, t_str, ' ');

            points.push_back( Point{std::stoi(x_str), std::stoi(y_str), std::stoi(t_str)} );
        }

        std::sort(points.begin(), points.end(), [](const Point& p1, const Point& p2) { return p1.t < p2.t; });
        
        trajectories.emplace_back(traj_idx, std::move(points));
    }

    return trajectories;
}


void compute_classifications(std::vector<Trajectory>& trajectories)
{
    Bench bench("Compute classifications");

    for(uint32_t i = 0; i < trajectories.size(); ++i)
    {
        for(uint32_t j = i + 1; j < trajectories.size(); ++j)
        {
            double length_diff = std::abs(trajectories[i].length() - trajectories[j].length());
            double speed_diff = std::abs(trajectories[i].speed() - trajectories[j].speed());

            trajectories[i].update_neighbours(length_diff, speed_diff, &trajectories[j]);
            trajectories[j].update_neighbours(length_diff, speed_diff, &trajectories[i]);
        }
    }
}

auto load_data(const std::string& filename)
{
    Bench bench("Load data");
    auto trajectories = read_file(filename);
    compute_classifications(trajectories);
    return trajectories;
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
    
    auto trajectories = load_data(filename);

    // CLI menu and action loop
    while(true)
    {
        int traj = 0;
        Metric metric = Unknown;

        std::cout << "Please select trajectory and metric.\n";
        std::cout << "  Trajectories [ 0 - "  << trajectories.size() <<"] : ";
        std::cin >> traj;
        std::cout << "  Metrics ( Length: 1, Speed: 2 ) : ";
        std::cin >> metric;

        if(traj < 0 || traj >= trajectories.size() || metric <= Unknown || metric >= EnumSize)
        {
            std::cout << "Trajectory or Metric have bad values\n";
            continue;
        }

        if(metric == Length)
        {
            std::cout << "Closest trajectories from trajectory " << traj << " based on length \n";
            for(const auto& [length, trajectory] : trajectories[traj].neighbours_length())
            {
                if(trajectory->id() == -1) { break; }
                std::cout << trajectory->id() << " ";
            }
            std::cout << '\n';
        }
        else if( metric == Speed )
        {
            std::cout << "Closest trajectories from trajectory " << traj << " based on speed \n";
            for (const auto& [length, trajectory] : trajectories[traj].neighbours_speed())
            {
                if (trajectory->id() == -1) { break; }
                std::cout << trajectory->id() << " ";
            }
            std::cout << '\n';
        }
        
        
        // Debug prompt if needed
        if (showClassification)
        {
            std::cout << "Lengths :\n";
            for(int i = 0; i < trajectories.size(); ++i)
            {
                std::cout<< "traj[" << i << "] : ";
                for(const auto& [length, trajectory] : trajectories[i].neighbours_length())
                {
                    std::cout << length << " (" << trajectory->id() << "), ";
                }
                std::cout << '\n';
            }

            std::cout << "Speeds :\n";
            for(int i = 0; i < trajectories.size(); ++i)
            {
                std::cout<< "traj[" << i << "] : ";
                for(const auto& [speed, trajectory] : trajectories[i].neighbours_speed())
                {
                    std::cout << speed << " (" << trajectory->id() << "), ";
                }
                std::cout << '\n';
            }
        }
    }

    return 0;
}