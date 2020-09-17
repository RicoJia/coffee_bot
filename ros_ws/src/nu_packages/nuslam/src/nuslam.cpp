//
// Created by ricojia on 3/10/20.
//

//#include "nuslam/nuslam.hpp"
#include "../include/nuslam/nuslam.hpp"
#include <iostream>

using std::vector;
using rigid2d::Circle;
using rigid2d::angle;
using rigid2d::Twist2D;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::JacobiSVD;
using Eigen::SelfAdjointEigenSolver;

std::vector<std::vector<unsigned int>> nuslam::cluster_points (std::vector<double> range_vec, double range_max, double angle_increment){
    double dist_thre = range_max * angle_increment* 2.0;
    std::vector<std::vector<unsigned int>> clusters;

    // because we're using .back later, we need to have at least one vector with the first index
    std::vector<unsigned int> first_vec;
    first_vec.push_back(0);
    clusters.push_back(first_vec);

    // append the first term to the back so we can check if the two ends are in the same cluster.
    range_vec.push_back(range_vec[0]);
//
    // we check from i=1 to the end of the range vector.
    for (unsigned int i = 1; i < range_vec.size(); ++i) {
        double a = range_vec[i - 1];
        double b = range_vec[i];
        double dist = sqrt(pow(a, 2) + pow(b, 2) - 2.0 * cos(angle_increment) * a * b);

       //  if two points are close enough, we are going to merge them
        if (dist <= dist_thre) {
            // merge the first cluster and the last cluster
            if (i == range_vec.size()-1) {
                    clusters.back().insert(clusters.back().end(), clusters[0].begin(), clusters[0].end());
                    clusters.erase(clusters.begin());
            }
            else{
                clusters.back().push_back( i );
            }
        }
        //if two points are not close, we add new clusters
        else{
            std::vector<unsigned int> new_cluster{i};
            clusters.push_back(new_cluster);
        }
    }

    //reject the big circle if no obstacle is in sight
    for (auto i = clusters.begin(); i!=clusters.end();){
        if (clusters.size()> 0 && (i -> size() >= 180 || i -> size() < 4) ){
            clusters.erase(i);
        }
        else{
            ++i;
        }
    }

    return clusters;
}

std::vector<std::vector<rigid2d::Vector2D>> nuslam::get_cluster_xy (std::vector<std::vector<unsigned int>> clusters, std::vector<double> range_vec){
    std::vector< std::vector<rigid2d::Vector2D>> clusters_xy;

    for (auto cluster:clusters){
        std::vector<rigid2d::Vector2D> cluster_xy;
        for (auto index:cluster){
            //Testing Lambda Expression hehe
            auto get_point = [range_vec](unsigned int index)-> rigid2d::Vector2D {
                double theta = rigid2d::PI*2.0* static_cast<double>(index)/360.0;
                double range = range_vec[index];
                double x = range * cos(theta);
                double y = range * sin(theta);
                rigid2d::Vector2D point (x,y);
                return point;
                    };
            rigid2d::Vector2D point = get_point(index);
            cluster_xy.push_back(point);
        }

        clusters_xy.push_back(cluster_xy);
    }

    return clusters_xy;
}

std::vector<rigid2d::Circle> nuslam::get_circle_list(std::vector<std::vector<rigid2d::Vector2D>> clusters_xy){

    // make Z: [z1, x1, y1, 1], decentralized.
    vector<Circle> Circle_list;
    //One cluster
    for (auto cluster_xy: clusters_xy){
        unsigned int row_num = cluster_xy.size();
        MatrixXd Z(row_num, 4);
        Z = MatrixXd::Constant(row_num, 4, 0.0);

        //every single point
        unsigned int row_i = 0;
        for (auto point : cluster_xy){
            Z(row_i, 1)= point.x; Z(row_i, 2)=point.y; Z(row_i, 3) = 1.0;
            ++row_i;
        }
        // Normalize Z
        double x_mean = Z.col(1).mean(); double y_mean = Z.col(2).mean();
        Z.col(1) = (Z.col(1).array()-x_mean).matrix(); Z.col(2) = (Z.col(2).array()-y_mean).matrix();
        Z.col(0) = (Z.col(1).array() * Z.col(1).array() + Z.col(2).array() * Z.col(2).array()).matrix();
        double z_mean = Z.col(0).mean();

        MatrixXd H(4,4);
        H<<8*z_mean, 0, 0, 2,
            0, 1, 0, 0,
            0, 0, 1, 0,
            2, 0, 0, 0;

        MatrixXd H_inv(4,4);
        H_inv<<0, 0, 0, 1.0/2.0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                1.0/2.0, 0, 0, -2.0*z_mean;

        JacobiSVD<MatrixXd> svd(Z, Eigen::ComputeThinU | Eigen::ComputeThinV);

        MatrixXd A;
        auto V = svd.matrixV();
        V.resize(4, V.size()/4);
        if (svd.singularValues()(3) > pow(10, -12)){
            MatrixXd Sigma(4,4);
            Sigma = MatrixXd::Constant(V.size()/4, V.size()/4, 0);
            for (unsigned int i = 0; i < V.size()/4; ++i){
                Sigma(i, i) = svd.singularValues()(i);
            }

            auto Y = V * Sigma * V.transpose();
            auto Q = Y * H_inv * Y;
            SelfAdjointEigenSolver<MatrixXd> eigensolver(Q);
            if (eigensolver.info() != Eigen::Success) abort();


            Eigen::ArrayXd::Index min_index;
            eigensolver.eigenvalues().array().abs().minCoeff(&min_index);

            auto A_star = eigensolver.eigenvectors().col(min_index);
            A = Y.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(A_star);
        }
        else{
            A = V.col(3);
        }

        double A1 = A.array()(0); double A2 = A.array()(1); double A3 = A.array()(2); double A4 = A.array()(3);
        double a = -1.0 * A2/(2.0 * A1);
        double b = -1.0 * A3/(2.0* A1);
        double R2 = (A2*A2 + A3*A3 - 4.0*A1*A4)/(4.0 * A1*A1);

        Circle Circle(a+x_mean, b+y_mean, std::sqrt(R2));
        Circle_list.push_back(Circle);
    }
    return Circle_list;
}

nuslam::TurtleMap_xy nuslam::construct_turtlemap_xy_msg(const std::vector<rigid2d::Circle>& circles){
    nuslam::TurtleMap_xy msg;
    unsigned int landmark_id = 0;
    for (auto circle:circles){
        nuslam::Landmark_xy landmark;
        landmark.last_update = ros::Time::now().toSec();
        landmark.x = circle.x;
        landmark.y = circle.y;
        landmark.radius = circle.radius;
        landmark.landmark_id = landmark_id;
        ++landmark_id;
        msg.landmarks.push_back(landmark);
    }
    return msg;
}

//--------------------------------------------------------------EKF functions

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using rigid2d::Twist2D;

nuslam::TurtleMap nuslam::EKF::get_associated_landmarks(const nuslam::TurtleMap_xy msg){
    nuslam::TurtleMap ret_msg;
    unsigned int new_id = landmarks_num;
    double dist_thre = 9.0;         //TODO: to adjust
    unsigned int count = 0;
    // see if element in seen has been initialized. 
    // If encounters an element that's not yet been initialized, then, make it a new element. The new element id will be: new_id. 
    for (auto landmark_xy: msg.landmarks){
        //parse thru existing landmarks. calculate its mahalanobis distance

        std::cout<<"-----index: "<<count<<": ";

        vector<double> dist_vec(landmarks_num, 0.0);
        double x = this->mu(0); double y = this->mu(1); double theta = this->mu(2);
        double range = rigid2d::distance(rigid2d::Vector2D(x,y), rigid2d::Vector2D(landmark_xy.x, landmark_xy.y));
        double bearing = rigid2d::normalize_angle(rigid2d::angle(rigid2d::Vector2D(landmark_xy.x - x, landmark_xy.y - y))) - theta;

        nuslam::Landmark landmark_msg;
        landmark_msg.last_update = landmark_xy.last_update;
        landmark_msg.range = range;
        landmark_msg.bearing = bearing;

        for (unsigned int index = 0; index < landmarks_num; ++index){
            dist_vec.at(index) = this -> get_mahalanobis_dist(range, bearing, index);
        }

        unsigned int min_index = std::min_element(dist_vec.begin(),dist_vec.end()) - dist_vec.begin();
        if ( landmarks_num>0 && dist_vec.at(min_index) <= dist_thre){
            landmark_msg.landmark_id = min_index;
        }
        //if no matching, see this as a new landmark
        else{
                landmark_msg.landmark_id = new_id;
                ++new_id;
        }

        ret_msg.landmarks.push_back(landmark_msg);

        //----------------------TODO
        ++count;
        std::cout<<"choose: "<<min_index<<std::endl;
//        for (auto dist: dist_vec){
//            std::cout<<dist<<" ";
//        }
//        std::cout<<std::endl;
    }
    std::cout<<"-------------------------"<<std::endl;
    return ret_msg;
}

double nuslam::EKF::get_mahalanobis_dist(double range, double bearing, unsigned int landmark_index){
    double dist;
    Vector2d z_i, z_i_hat;
    z_i = Vector2d(range, bearing);
    double landmark_x = this -> mu(2*landmark_index + 3); double landmark_y = this -> mu(2*landmark_index + 4);
    double x = this->mu(0); double y = this->mu(1); double theta = this->mu(2);
    double landmark_range = rigid2d::distance(rigid2d::Vector2D(x,y), rigid2d::Vector2D(landmark_x, landmark_y));
    double landmark_bearing = rigid2d::normalize_angle(rigid2d::angle(rigid2d::Vector2D(landmark_x - x, landmark_y - y))) - theta;
    z_i_hat<< landmark_range, landmark_bearing;

    MatrixXd _sigma (5,5);
    _sigma.block<3,3>(0,0) = this -> Sigma.block<3,3>(0,0);
    _sigma.block<3,2>(0,3) = this -> Sigma.block<3,2>(0,2*landmark_index+3);
    _sigma.block<2,3>(3,0) = this -> Sigma.block<2,3>(2*landmark_index+3, 0);
    _sigma.block<2,2>(3,3) = this -> Sigma.block<2,2>(2*landmark_index+3,2*landmark_index+3);

    Vector2d z_i_diff = z_i - z_i_hat;
    double q = z_i_diff.squaredNorm();
    double delta_x = z_i_diff(0); double delta_y = z_i_diff(1);
    MatrixXd H_low(2, 5);
    H_low<< -1.0*delta_x/sqrt(q), -1.0*delta_y/sqrt(q), 0.0, delta_x/sqrt(q), delta_y/sqrt(q),
            delta_y/q,             -1.0*delta_x/q,      -1.0, -1.0*delta_y/q, delta_x/q;

    auto psi = H_low * _sigma * H_low.transpose() + this->Q;

    dist = z_i_diff.transpose() * psi.inverse() * z_i_diff;

    //TODO:
    std::cout<<"z_t_diff: "<<std::endl<<z_i_diff<<std::endl;
    return dist;
}

Eigen::MatrixXd nuslam::EKF::G(const Eigen::MatrixXd& Fk, unsigned int max_landmarks_num, const rigid2d::Twist2D& pose_twist, const rigid2d::Twist2D& old_pose_twist){
    auto I = MatrixXd::Identity(2 * max_landmarks_num + 3, 2 * max_landmarks_num + 3);
    double delta_y = pose_twist.y - old_pose_twist.y; double delta_x = pose_twist.x - old_pose_twist.x;

    MatrixXd g_t(3,3);
    g_t <<0, 0,-1.0 * delta_y,
          0, 0, delta_x,
          0, 0, 0;

    auto G = I + Fk.transpose() * g_t * Fk;
    return G;
}

Eigen::MatrixXd nuslam::EKF::R_t(const rigid2d::Twist2D velocity_twist){
    double theta = this -> mu(2);
    double halfRot = velocity_twist.theta / 2.0;
    double c = cos(theta + halfRot); double s = sin(theta + halfRot);

    MatrixXd M_t(2,2);
    double sigma_v = this->motion_stddev.at(0);
    double sigma_w = this->motion_stddev.at(1);

    M_t << sigma_v * sigma_v, 0.0,
            0.0, sigma_w * sigma_w;

    MatrixXd V_t(3,2);
    V_t << c, -0.5 * s,
            s, 0.5 * c,
            0, 1.0;

    auto R_t = V_t * M_t * (V_t.transpose());

    return R_t;
}

void nuslam::EKF::update_motion_model(const rigid2d::Twist2D velocity_twist){
    // update diff_drive with the most up-to-date mu
    diff_drive.reset(Twist2D(mu(2), mu(0), mu(1)));
    auto old_pose_twist = diff_drive.get_pose();
    //update diff_drive pose with wheel velocity twist only.
    diff_drive.feedforward(velocity_twist);
    auto pose_twist = diff_drive.get_pose();
    VectorXd miu(3);
    miu<<pose_twist.x, pose_twist.y, pose_twist.theta;
    this -> mu.head(3) = miu;

    auto G_t = this -> G(this->Fk, this->max_landmarks_num, pose_twist, old_pose_twist);
    auto R_t = this -> R_t(velocity_twist);

    this -> Sigma = G_t * this->Sigma * (G_t.transpose()) + this->Fk.transpose() * R_t * this->Fk;
}

rigid2d::Twist2D nuslam::EKF::get_ekf_pose(){
    double theta = this -> mu(2);
    double x = this -> mu(0);
    double y = this -> mu(1);
    return Twist2D(theta, x, y);
}

void nuslam::EKF::measurement_update(const nuslam::TurtleMap& msg){

    auto mu_pose_copy = this -> mu.head(3);

    for (auto landmark: msg.landmarks){

        //Step 0: initialize z_t_i
        //z_t_i = [r_i, phi_i, j_i]
        VectorXd z_t_i(2);
        z_t_i << landmark.range, landmark.bearing;
        // if the landmark has never seen before, initialize mu. If maximum number of landmarks have been reached, we don'tneed more landmarks
        if (landmark.landmark_id >= max_landmarks_num){
            continue;
        }
        if (this -> seen(landmark.landmark_id)==false){
            double x_dist = cos(landmark.bearing) * landmark.range;
            double y_dist = sin(landmark.bearing) * landmark.range;
            this -> mu(2 * landmark.landmark_id + 3) = mu_pose_copy(0) + x_dist;
            this -> mu(2 * landmark.landmark_id + 4) = mu_pose_copy(1) + y_dist;
            this -> seen(landmark.landmark_id ) = true;
            this -> landmarks_num += 1;
        }

        //Step 1: measurement estimate
        //q: dist(current_pose, landmark_pos)
        MatrixXd F_1 = MatrixXd::Identity(5, 3);
        MatrixXd F_2 = MatrixXd::Zero(5,2);
        F_2.block<2,2>(3,0) = MatrixXd::Identity(2,2);
        auto dist_vector = this -> mu.segment(2 * landmark.landmark_id + 3, 2) - mu_pose_copy.head(2);
        double q = (dist_vector).squaredNorm();
        double theta = rigid2d::normalize_angle(angle(rigid2d::Vector2D(dist_vector(0), dist_vector(1))) - mu(2));
        Vector2d z_t_i_hat;
        z_t_i_hat << sqrt(q), theta;

//        ------ Step 2: Linearize Measurement Model with Jacobian ------
//        F_x makes the landmark z_t_i hatbecome a state.
//                1 0 0  0 ...... 0   0 0   0 ...... 0
//                0 1 0  0 ...... 0   0 0   0 ...... 0
//         F_x =  0 0 1  0 ...... 0   0 0   0 ...... 0
//                0 0 0  0 ...... 0   1 0   0 ...... 0
//                0 0 0  0 ...... 0   0 1   0 ...... 0
//                        2*j           3+2*landmark_num - (3+2*j+2)
//          -delta_x/sqrt_q  -delta_y/sqrt_q  0  delta_x/sqrt_q  delta_y/q
//        H_low =   delta_y/q   -delta_x/q  -1  -delta_y/q  delta_x/q
//        delta_x is the x coordinate difference bw updated landmark and robot pose
        MatrixXd F_x(5, F_1.cols() + 2 * landmark.landmark_id + F_2.cols() + (3+2*max_landmarks_num) - (3 + 2*landmark.landmark_id + 2));
        F_x = MatrixXd::Zero(5, F_1.cols() + 2 * landmark.landmark_id + F_2.cols() + (3+2*max_landmarks_num) - (3 + 2*landmark.landmark_id + 2));
        F_x.block<5, 3>(0,0) = F_1;
        F_x.block<5,2>(0, 2*landmark.landmark_id + 3) = F_2;
        double delta_x = dist_vector(0); double delta_y = dist_vector(1);
        MatrixXd H_low(2, 5);
        H_low<< -1.0*delta_x/sqrt(q), -1.0*delta_y/sqrt(q), 0.0, delta_x/sqrt(q), delta_y/sqrt(q),
                delta_y/q,             -1.0*delta_x/q,      -1.0, -1.0*delta_y/q, delta_x/q;
        auto H_i_t = H_low * F_x;

//# ---------------- Step 3: Kalman gain update -----------------#
        auto S_i_t = H_i_t * this->Sigma * (H_i_t.transpose()) + this -> Q;
        auto K_i_t = this -> Sigma * (H_i_t.transpose()) * (S_i_t.inverse());

//# ------------------- Step 4: mean update ---------------------#
        Vector2d z_t_difference;
        z_t_difference = z_t_i - z_t_i_hat;
        z_t_difference(1) = rigid2d::normalize_angle(z_t_difference(1));

        auto innovation = K_i_t * z_t_difference;
        this -> mu += innovation;

        MatrixXd I(3+2*max_landmarks_num, 3+2*max_landmarks_num);
        I = MatrixXd::Identity(3+2*max_landmarks_num, 3+2*max_landmarks_num);
        this->Sigma = (I - K_i_t*H_i_t) * this->Sigma;
    }

//    std::cout<<"after observation, pose: "<<std::endl<<mu<<std::endl;

}
