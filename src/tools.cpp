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

	VectorXd rmse(4);
	rmse << 0,0,0,0;
	
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
        VectorXd err = estimations[i] - ground_truth[i];
		vecs.push_back((err.array() * err.array()).matrix());
	}

	//calculate the mean
	
	for (int i = 0; i < vecs.size(); ++i) {
	    rmse += vecs[i];
	}
	
	rmse /= vecs.size();

	//calculate the squared root
	
	rmse = rmse.array().sqrt().matrix();
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3,4);

	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	if (px == 0 || py == 0 || vx == 0 || vy == 0) {
	    vx = vy = 0.00001;
	}

	float px2_py2 = px * px + py * py;
	float sqrt_px2_py2 = sqrt( px2_py2 );
    
    Hj <<
        px/sqrt_px2_py2, py/sqrt_px2_py2, 0, 0,
        -py/px2_py2, px/px2_py2, 0, 0,
            (py*(vx*py-vy*px)) / sqrt(pow(px2_py2, 3)), 
            px*(vy*px-vx*py) / sqrt(pow(px2_py2, 3)), 
            px / sqrt_px2_py2, 
            py / sqrt_px2_py2;
    return Hj;
}
