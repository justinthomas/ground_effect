#include <string>
#include <vector>

typedef std::vector< std::vector< std::vector<double> > > traj_type;

int loadTraj(const std::string &filename, traj_type &traj);
