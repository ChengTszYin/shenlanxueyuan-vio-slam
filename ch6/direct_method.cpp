#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>

using namespace std;

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics
// 内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// 基线
double baseline = 0.573;
// paths
string left_file = "./left.png";
string disparity_file = "./disparity.png";
boost::format fmt_others("./%06d.png");    // other files

// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21,
        bool multithread
);

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
);

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

int main(int argc, char **argv) {

    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 1000;
    int boarder = 40;
    VecVector2d pixels_ref;
    vector<double> depth_ref;

    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) {
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3 T_cur_ref;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (int i = 1; i < 6; i++) {  // 1~10
        cv::Mat img = cv::imread((fmt_others % i).str(), 0);
        // DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);    // first you need to test single layer
        DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref, false);
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_del1 = chrono::duration_cast<chrono::duration<double>> (t2 - t1);
    cout << "time used for single thread DirectPoseEstimationMultiLayer: " << time_del1.count() << " seconds." << endl;

    chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
    for (int i = 1; i < 6; i++) {  // 1~10
        cv::Mat img = cv::imread((fmt_others % i).str(), 0);
        // DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);    // first you need to test single layer
        DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref, true);
    }
    chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
    chrono::duration<double> time_def2 = chrono::duration_cast<chrono::duration<double>> (t4 - t3);
    cout << "time used for multithread DirectPoseEstimationMultiLayerMT: " << time_def2.count() << " seconds." << endl;
}

void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 100;

    double cost = 0, lastCost = 0;
    int nGood = 0;  // good projections
    VecVector2d goodProjection;

    for (int iter = 0; iter < iterations; iter++) {
        nGood = 0;
        goodProjection.clear();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias

        for (size_t i = 0; i < px_ref.size(); i++) {

            // compute the projection in the second image
            // TODO START YOUR CODE HERE
            Eigen::Vector3d point_ref = depth_ref[i] * Eigen::Vector3d(     //p1 convert tp P
                    (px_ref[i][0] - cx ) / fx,
                    (px_ref[i][1] - cy ) / fy,
                    1
            );
            Eigen::Vector3d q = T21 * point_ref;     //q = TP = (RP + t)
            if(q[2] < 0) continue;

            float u =fx * q[0] / q[2] + cx, v = fy * q[1] / q[2] + cy;
            if( u < half_patch_size || u > img2.cols - half_patch_size || v < half_patch_size || v > img2.rows - half_patch_size) continue; 
            nGood++;
            goodProjection.push_back(Eigen::Vector2d(u, v));
            double X = q[0], Y = q[1], Z = q[2], Z2 = Z * Z, Z_inv = 1.0 / Z, Z_inv2 = Z_inv * Z_inv;
            // and compute error and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    double error = GetPixelValue(img1, px_ref[i][0] + x, px_ref[i][1] + y) - GetPixelValue(img2, u + x, v + y);

                    Matrix26d J_pixel_xi;   // pixel to \xi in Lie algebra
                    Eigen::Vector2d J_img_pixel;    // image gradients
                    J_pixel_xi(0, 0) = fx * Z_inv;
                    J_pixel_xi(0, 1) = 0;
                    J_pixel_xi(0, 2) = -fx * X * Z_inv2;
                    J_pixel_xi(0, 3) = -fx * X * Y * Z_inv2;
                    J_pixel_xi(0, 4) = fx + fx * X * X * Z_inv2;
                    J_pixel_xi(0, 5) = -fx * Y * Z_inv;

                    J_pixel_xi(1, 0) = 0;
                    J_pixel_xi(1, 1) = fy * Z_inv;
                    J_pixel_xi(1, 2) = -fy * Y * Z_inv2;
                    J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z_inv2;
                    J_pixel_xi(1, 4) = fy * X * Y * Z_inv2;
                    J_pixel_xi(1, 5) = fy * X * Z_inv;

                    J_img_pixel = Eigen::Vector2d(
                        0.5 * (GetPixelValue(img2, u + x + 1, v + y) - GetPixelValue(img2, u + x - 1, v + y)),
                        0.5 * (GetPixelValue(img2, u + x, v + y + 1) - GetPixelValue(img2, u + x, v + y - 1))
                    );
 
                    // total jacobian
                    Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();
                    H += J * J.transpose();
                    b += -error * J;
                    cost += error * error;
                }
            // END YOUR CODE HERE
        }

        // solve update and put it into estimation
        // TODO START YOUR CODE HERE
        Vector6d update = H.ldlt().solve(b);
        T21 = Sophus::SE3::exp(update) * T21;
        // END YOUR CODE HERE

        cost /= nGood;

        if (isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            // cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        lastCost = cost;
        // cout << "cost = " << cost << ", good = " << nGood << endl;
    }
    //cout << "good projection: " << nGood << endl;
    //cout << "T21 = \n" << T21.matrix() << endl;

    // in order to help you debug, we plot the projected pixels here
    cv::Mat img1_show, img2_show;
    cv::cvtColor(img1, img1_show, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img2, img2_show, cv::COLOR_GRAY2BGR);
    for (auto &px: px_ref) {
        cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }
    for (auto &px: goodProjection) {
        cv::rectangle(img2_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }
    // cv::imshow("reference", img1_show);
    // cv::imshow("current", img2_show);
    // cv::waitKey();
}

void DirectPoseEstimationSingleLayerMT(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 100;

    double cost = 0, lastCost = 0;
    int nGood = 0;  // good projections
    VecVector2d goodProjection;

    for (int iter = 0; iter < iterations; iter++) {
        const int N = px_ref.size();
        vector<Matrix6d> H_per_point(N, Matrix6d::Zero());
        vector<Vector6d> b_per_point(N, Vector6d::Zero());
        vector<double> cost_per_point(N, 0.0);
        vector<bool> is_good(N, false);
        vector<Eigen::Vector2d> proj_per_point(N);

        cv::parallel_for_(cv::Range(0, static_cast<int>(px_ref.size())), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            Eigen::Vector3d point_ref(
                (px_ref[i][0] - cx) / fx * depth_ref[i],
                (px_ref[i][1] - cy) / fy * depth_ref[i],
                depth_ref[i]
            );

            Eigen::Vector3d q = T21 * point_ref;
            if (q[2] < 0.1) continue;

            float u = fx * q[0] / q[2] + cx;
            float v = fy * q[1] / q[2] + cy;

            if (u < half_patch_size || u >= img2.cols - half_patch_size ||
                v < half_patch_size || v >= img2.rows - half_patch_size) {
                continue;
            }

            // 标记这个点有效（用 is_good 数组，不要用 nGood++）
            is_good[i] = true;
            proj_per_point[i] = Eigen::Vector2d(u, v);

            // 关键：使用 q 的坐标
            double X = q[0], Y = q[1], Z = q[2];
            double Z_inv = 1.0 / Z;
            double Z_inv2 = Z_inv * Z_inv;

            Matrix6d H_local = Matrix6d::Zero();
            Vector6d b_local = Vector6d::Zero();
            double cost_local = 0.0;

            for (int x = -half_patch_size; x < half_patch_size; ++x) {
                for (int y = -half_patch_size; y < half_patch_size; ++y) {
                    double error = GetPixelValue(img1, px_ref[i][0] + x, px_ref[i][1] + y)
                                - GetPixelValue(img2, u + x, v + y);

                    Eigen::Matrix<double, 2, 6> J_pixel_xi;
                    J_pixel_xi << 
                        fx*Z_inv, 0, -fx*X*Z_inv2, -fx*X*Y*Z_inv2, fx + fx*X*X*Z_inv2, -fx*Y*Z_inv,
                        0, fy*Z_inv, -fy*Y*Z_inv2, -fy - fy*Y*Y*Z_inv2, fy*X*Y*Z_inv2, fy*X*Z_inv;

                    Eigen::Vector2d J_img_pixel(
                        0.5 * (GetPixelValue(img2, u+x+1, v+y) - GetPixelValue(img2, u+x-1, v+y)),
                        0.5 * (GetPixelValue(img2, u+x, v+y+1) - GetPixelValue(img2, u+x, v+y-1))
                    );

                    Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();

                    H_local += J * J.transpose();
                    b_local += -error * J;
                    cost_local += error * error;
                }
            }

            H_per_point[i] = H_local;
            b_per_point[i] = b_local;
            cost_per_point[i] = cost_local;
        }
    });  
        Matrix6d H = Matrix6d::Zero();
        Vector6d b = Vector6d::Zero();
        double cost = 0.0;
        int nGood = 0;
        goodProjection.clear();

        for (size_t i = 0; i < N; ++i) {
            if (!is_good[i]) continue;

            H += H_per_point[i];
            b += b_per_point[i];
            cost += cost_per_point[i];
            nGood++;
            goodProjection.push_back(proj_per_point[i]);
        }

        if (nGood == 0) {
            std::cout << "No good points!" << std::endl;
            break;
        }

        Vector6d update = H.ldlt().solve(b);
        T21 = Sophus::SE3::exp(update) * T21;

        cost /= nGood;

        if (isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            // cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        lastCost = cost;
        // cout << "cost = " << cost << ", good = " << nGood << endl;  
    }
    //std::cout << "good projection: " << goodProjection.size() << std::endl;
    //std::cout << "T21 = \n" << T21.matrix() << std::endl;
}

void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21,
        bool multithread = false
) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<cv::Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE
    for(int i = 0; i < pyramids; ++i)
    {
        if(i == 0)
        {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        }
        else{
            cv::Mat img1_o, img2_o;
            cv::resize(pyr1[i - 1] * pyramid_scale, img1_o, cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
            cv::resize(pyr2[i - 1] * pyramid_scale, img2_o, cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(img1_o);
            pyr2.push_back(img2_o);
        }
    }
    // END YOUR CODE HERE

    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values
    for (int level = pyramids - 1; level >= 0; level--) {
        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
        for (auto &px: px_ref) {
            px_ref_pyr.push_back(scales[level] * px);
        }

        // TODO START YOUR CODE HERE
        // scale fx, fy, cx, cy in different pyramid levels
        fx = fxG * scales[level];
        fy = fyG * scales[level];
        cx = cxG * scales[level];
        cy = cyG * scales[level];
        // END YOUR CODE HERE
        if(multithread){
            DirectPoseEstimationSingleLayerMT(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
        }
        else{
            DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
        }
        
    }

}

