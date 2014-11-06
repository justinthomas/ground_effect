#include "trajectory.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

/* 
int main()
{
  // traj[time_idx][flat_output][derivative]
  vector< vector< vector<double> > > traj;

  int success = loadTraj("traj.csv", traj);

  cout << "Returned: " << success << endl;

  // Output the array

  // Loop through flat outputs 
  for (unsigned int i2=0; i2 < traj[0].size(); i2++)
  {
    cout << "Flat Output (group): " << i2 << endl;
    
    // Loop through times
    for (unsigned int i1=0; i1 < traj.size(); i1++)
    {
      
      // Loop through derivatives 
      for (unsigned int i3=0; i3 < traj[i1][i2].size(); i3++)
      {
        cout << traj[i1][i2][i3] << " ";
      }
      cout << endl;
    }
    cout << endl;
  }

  for (unsigned int i3=0; i3 < traj[0][0].size(); i3++)
  {
    cout << "Derivative: " << i3 << endl;
    for (unsigned int i1=0; i1 < traj.size(); i1++)
    {
      // Flat Outputs
      for (unsigned int i2=0; i2 < traj[0].size(); i2++)
      {
        cout << traj[i1][i2][i3] << " ";
      }
      cout << endl;
    }
    cout << endl;
  }
}
*/

// Much of this is from http://www.cplusplus.com/forum/unices/112048/
int loadTraj(const std::string &filename, traj_type &traj)
{
  // dims[0] is the number of rows
  // dims[1] is the number of flat outputs
  // dims[2] is the number of derivatives
  // dims[3] is the number of additional columns 
  int dims[4];

  // Load the file
  std::ifstream file(filename.c_str());

  // Define some variables
  std::string line;
  std::string val;
 
  // Read the first line and make sure it is good 
  std::getline(file, line);
  if (!file.good())
    return 1;

  // Parse the line for commas
  // Note: we expect that the first line contains
  // # of time steps, # of flat outputs, # of derivatives
  // such that the product of the first line is the total number of elements
  std::stringstream iss(line);
  for(int idx=0; idx<4; idx++)
  {
    std::getline(iss, val, ',');
    if (iss.fail())
      return 2;

    std::stringstream convertor(val);
    convertor >> dims[idx];
  }
 
  cout << "Dimensions: {" << dims[0] << ", " << dims[1] << ", " << dims[2] << "}" << " + " << dims[3] << " additional columns" << endl;

  // Resize the trajectory vector of vectors
  traj.resize(dims[0]);
  for (int i = 0; i < dims[0]; i++)
  {
    traj[i].resize(dims[1]+1);
    for (int j = 0; j < dims[1]; j++)
    {
      traj[i][j].resize(dims[2]);
    }
    traj[i][dims[1]].resize(dims[3]);
  }
 
  // This dimension indexes the time
  for(int dim0 = 0; dim0 < dims[0]; dim0++)
  {
    // Get the entire line
    std::getline(file, line);
    if (file.fail())
    {
      cout << "Error reading line " << dim0+1 << endl;
      return 3;
    }
   
    // Create a stringstream for the line
    std::stringstream iss(line);

    // This dimension is the derivative
    for (int dim2 = 0; dim2 < dims[2]; dim2++)
    {
      // This dimension is the flat output
      for (int dim1 = 0; dim1 < dims[1]; dim1++)
      {
        std::getline(iss, val, ',');
        if (iss.fail())
          return 4;
     
        // The converter
        std::stringstream convertor(val);
        convertor >> traj[dim0][dim1][dim2];
      }
    }

    // The additional columns
    for (int dim3 = 0; dim3 < dims[3]; dim3++)
    {
      std::getline(iss, val, ',');
      if (iss.fail())
        return 4;
      
      // The converter
      std::stringstream convertor(val);
      convertor >> traj[dim0][dims[1]][dim3];
    }
  }
 
  cout << "Trajectory loaded" << endl;
  return 0;
}
