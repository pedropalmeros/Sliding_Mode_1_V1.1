#include "Eigen/Dense"
#include <iostream>

using namespace std;


using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::VectorXf;
using Eigen::Vector3f;
using Eigen::ArrayXf;
using Eigen::Quaternion;
using Eigen::Quaternionf;

using namespace Eigen;


 
int main()
{
  Vector3f v1(1,2,3);
  Vector3f w2(1,0,0);

  cout << "v1.x(): " << v1.x() << endl; 
  cout << "v1.y(): " << v1.y() << endl; 
  cout << "v1.z(): " << v1.z() << endl;
  
  double dotProduct = v1.dot(w2);

  cout << "Inner product: " << dotProduct << endl;

  Vector3f res_vec = v1.cross(w2);

  cout << "Cross product: " << endl;
  cout << res_vec << endl; 

  float b = 0.5;
  cout << endl << endl;
  cout << "scalar product vector:  " << endl;
  cout << b*v1 << endl;
  
  Quaternionf q_1(0,1,2,3);
  cout << "Quaternion test: " << endl;
  cout << q_1.w() << endl << q_1.vec() << endl << endl << endl; 

  cout << "DotProcut: " << q_1.vec().dot(v1) << endl; 
}
