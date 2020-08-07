// The spheres and poses are fused in a single dataset, instead of two datasets for sphere and poses
#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseArray.h"
#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <map_creator/sphere_discretization.h>
#include <map_creator/aubokinematics.h>
// #include <map_creator/kinematics.h>
#include<map_creator/hdf5_dataset.h>

//struct stat st;

typedef std::vector<std::pair< std::vector< double >, const std::vector< double >* > > MultiVector;
//typedef std::multimap< const std::vector< double >*, const std::vector< double >* > MultiMap;

bool isFloat(std::string s)
{
  std::istringstream iss(s);
  float dummy;
  iss >> std::noskipws >> dummy;
  return iss && iss.eof();  // Result converted to bool
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "workspace");
  ros::NodeHandle n;
  ros::Time startit = ros::Time::now();
  float resolution = 0.08;
  std::string file = str(boost::format("Auboi10_r%d_reachability.h5") % resolution);
  std::string path(ros::package::getPath("map_creator") + "/maps/");
  std::string filename="./aubo10.h5";
  // kinematics::Kinematics k;
  ros::Rate loop_rate(10);

  int count = 0;
  VectorXd q_ref(6);
  q_ref<<0,0,0,0,0,0;
  while (ros::ok())
  {
    unsigned char max_depth = 16;
    unsigned char minDepth = 0;

    // A box of radius 1 is created. It will be the size of the robot+1.5. Then the box is discretized by voxels of
    // specified resolution
    // TODO resolution will be user argument
    // The center of every voxels are stored in a vector

    sphere_discretization::SphereDiscretization sd;
    // Aubo workspace radius
    float r = 1;

    octomap::point3d origin = octomap::point3d(0, 0, 0);  // This point will be the base of the robot
    octomap::OcTree *tree = sd.generateBoxTree(origin, r, resolution);
    std::vector< octomap::point3d > new_data;
    ROS_INFO("Creating the box and discretizing with resolution: %f", resolution);
    int sphere_count = 0;
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
      sphere_count++;
    }
    new_data.reserve(sphere_count);
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
      new_data.push_back(it.getCoordinate());
      // std::cout<<"it.getCoordinate()"<<it.getCoordinate()<<std::endl;
    }

    ROS_INFO("Total no of spheres now: %lu", new_data.size());
    ROS_INFO("Please hold ON. Spheres are discretized and all of the poses are checked for Ik solutions. May take some "
             "time");

    // A sphere is created in every voxel. The sphere may be created by default or other techniques.
    // TODO Other techniques need to be modified. the user can specifiy which technique they want to use
    // TODO The sphere discretization parameter and rotation of every poses will be taken as argument. If the final
    // joints can rotate (0, 2pi) we dont need to rotate the poses.
    // Every discretized points on spheres are converted to pose and all the poses are saved in a multimap with their
    // corresponding sphere centers
    // If the resolution is 0.01 the programs not responds

    float radius = resolution;

    VectorOfVectors sphere_coord;
    sphere_coord.resize( new_data.size() );

    MultiVector pose_col;
    pose_col.reserve( new_data.size() * 50);

    for (int i = 0; i < new_data.size(); i++)
    {
      static std::vector< geometry_msgs::Pose > pose;
      sd.convertPointToVector(new_data[i], sphere_coord[i]);

      sd.make_sphere_poses(new_data[i], radius, pose);
      for (int j = 0; j < pose.size(); j++)
      {
        static std::vector< double > point_on_sphere;
        sd.convertPoseToVector(pose[j], point_on_sphere);
        pose_col.push_back( std::make_pair(point_on_sphere, &sphere_coord[i]));
        // std::cout<<"point_on_sphere"<<point_on_sphere[j]<<"sphere_coord[i]"<<sphere_coord[i][j]<<std::endl;
      }
    }

    // // Every pose is checked for IK solutions. The reachable poses and the their corresponsing joint solutions are
    // // stored. Only the First joint solution is stored. We may need this solutions in the future. Otherwise we can show
    // // the robot dancing with the joint solutions in a parallel thread
    // // TODO Support for more than 6DOF robots needs to be implemented.



    MultiMapPtr pose_col_filter;
    VectorOfVectors ik_solutions;
    ik_solutions.reserve( pose_col.size() );

    for (MultiVector::iterator it = pose_col.begin(); it != pose_col.end(); ++it)
    {
      // static std::vector< double > joints(6);
      // int solns;
      // if (k.isIKSuccess(it->first, joints, solns))
      // {
      //   pose_col_filter.insert( std::make_pair( it->second, &(it->first)));
      //   ik_solutions.push_back(joints);
      //   // cout<<it->first[0]<<" "<<it->first[1]<<" "<<it->first[2]<<" "<<it->first[3]<<" "<<it->first[4]<<"
      //   // "<<it->first[5]<<" "<<it->first[6]<<endl;
      // }

      // std::cout<<"Qutanion"<<it->first[0]<<" "<<it->first[1]<<" "<<it->first[2]<<" "<<it->first[3]<<" "<<it->first[4]<<" "<<it->first[5]<<" "<<it->first[6]<<std::endl;
      double quat[4],eerot[9];
      quat[0]=it->first[6];
      quat[1]=it->first[3];
      quat[2]=it->first[4];
      quat[3]=it->first[5];

      
    
      quaternionToOriMatrix(quat,eerot);
      MatrixXd T_target(4,4);
      T_target(0,0) = eerot[0];
      T_target(0,1) = eerot[1];
      T_target(0,2) = eerot[2];
      T_target(0,3) = it->first[0];
      T_target(1,0) = eerot[3];
      T_target(1,1) = eerot[4];
      T_target(1,2) = eerot[5];
      T_target(1,3) = it->first[1];
      T_target(2,0) = eerot[6];
      T_target(2,1) = eerot[7];
      T_target(2,2) = eerot[8];
      T_target(2,3) = it->first[2];
      T_target.row(3) << 0, 0, 0, 1;

      // printf("Rotation Matrix:%d\n",sizeof(eerot) / sizeof(eerot[0]));
      // for (size_t i = 0; i < sizeof(eerot) / sizeof(eerot[0]); i++)
      // {
      //   /* code */
      //   std::cout<<" "<<eerot[i]<<" ";
      // }
      // printf("\n");
    VectorXd q_result(6);
    if(GetInverseResult(T_target,q_ref,q_result))
    {
      static std::vector< double > joints(6);
      for (size_t i = 0; i < 6; i++)
      {
        joints[i]=q_result[i];
        /* code */
      }
      
      pose_col_filter.insert( std::make_pair( it->second, &(it->first)));
      ik_solutions.push_back(joints);
    }

    }

    ROS_INFO("Total number of poses: %lu", pose_col.size());
    ROS_INFO("Total number of reachable poses: %lu", pose_col_filter.size());

    // // The centers of reachable spheres are stored in a map. This data will be utilized in visualizing the spheres in
    // // the visualizer.
    // // TODO there are several maps are implemented. We can get rid of few maps and run large loops. The complexity of
    // // accessing map is Olog(n)

    MapVecDoublePtr sphere_color;


    for (MultiMapPtr::iterator it = pose_col_filter.begin(); it != pose_col_filter.end(); ++it)
    {
      const std::vector<double>* sphere_coord    = it->first;
      //const std::vector<double>* point_on_sphere = it->second;

      // Reachability Index D=R/N*100;
      float d = float(pose_col_filter.count(sphere_coord)) / (pose_col.size() / new_data.size()) * 100;
      sphere_color.insert( std::make_pair(it->first, double(d)));
    }

    ROS_INFO("No of spheres reachable: %lu", sphere_color.size());

    // Creating maps now
    //Saving map to dataset
    hdf5_dataset::Hdf5Dataset h5(filename);
    h5.saveReachMapsToDataset(pose_col_filter, sphere_color, resolution);

    double dif = ros::Duration( ros::Time::now() - startit).toSec();
    ROS_INFO("Elasped time is %.2lf seconds.", dif);
    ROS_INFO("Completed");
    
    ros::spinOnce();
    // sleep(10000);
    return 1;
    loop_rate.sleep();
    count;
  }
  return 0;
}
