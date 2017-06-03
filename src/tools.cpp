#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  cout << "Tools::CalculateRMSE" << endl;

  // Lesson 5: Section 22
	VectorXd rmse(4);
	rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	
	if (estimations.size() == 0) {
	    cout << "Estimations size 0" << endl;
	    return rmse;
	}
	
	if (estimations.size() != ground_truth.size()) {
	    cout << "GT & estimation size mismatch" << endl;
	    return rmse;
	}
	
	std::vector<VectorXd> vecs;

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd err = estimations[i] - ground_truth[i];
		vecs.push_back((err.array() * err.array()).matrix());
	}

	//calculate the mean
	// ... your code here
	
	for (int i = 0; i < vecs.size(); ++i) {
	    rmse += vecs[i];
	}
	
	rmse /= vecs.size();

	//calculate the squared root
	// ... your code here
	
	rmse = rmse.array().sqrt().matrix();
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  cout << "Tools::CalculateJacobian" << endl;

  // Lesson 5: Section 18
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 
	
	if (px == 0 || py == 0 || vx == 0 || vy == 0) {
	    cout << "Divvy by zero" << endl;
	    return Hj;
	}

	//check division by zero
	
	//compute the Jacobian matrix
    
    Hj <<
        px/sqrt(px*px+py*py), py/sqrt(px*px+py*py), 0, 0,
        -py/(px*px+py*py), px/(px*px+py*py), 0, 0,
        
            (py*(vx*py-vy*px)) / sqrt(pow(px*px+py*py, 3)), 
            px*(vy*px-vx*py) / sqrt(pow(px*px+py*py, 3)), 
            px / sqrt(px*px+py*py), 
            py / sqrt(px*px+py*py) 
        
        ;
    
}
