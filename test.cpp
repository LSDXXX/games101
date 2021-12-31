#include <eigen3/Eigen/Eigen>
#include <iostream>


int main() {
	Eigen::Vector3f v({1.0, 1.0, 1.0});
	Eigen::Vector3f v1({1.1, 1.1, 1.1});
	std::cout << v.dot(v1) << std::endl;
}


