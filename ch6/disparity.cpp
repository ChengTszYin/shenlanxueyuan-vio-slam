#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
using namespace std;
using namespace cv;

#define CV_GRAY2BGR COLOR_GRAY2BGR
// this program shows how to use optical flow

string file_1 = "./left.png";  // first image
string file_2 = "./right.png";  // second image

// TODO implement this funciton
/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

void OpticalFlowSingleLevelMT(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

// TODO implement this funciton
/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

void OpticalFlowMultiLevelMT(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return
 */
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

    // images, note they are CV_8UC1, not CV_8UC3
    Mat img1 = imread(file_1, 0);
    Mat img2 = imread(file_2, 0);

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);
    // now lets track these key points in the second image
    // first use single level LK in the validation picture

    // then test multi-level LK
    vector<KeyPoint> kp2_multi;
    vector<bool> success_multi;
    OpticalFlowMultiLevelMT(img1, img2, kp1, kp2_multi, success_multi, true);

    Mat disparity_gt = imread("disparity.png", 0);

    for (int i = 0; i < kp2_multi.size(); i++) {
        if (success_multi[i]) {
            double disparity = kp1[i].pt.x - kp2_multi[i].pt.x;

            double true_disparity = disparity_gt.at<uchar>(kp1[i].pt.y, kp1[i].pt.x);
            cout << "At kp " << i << ", est: " << disparity 
             << ", gt: " << true_disparity 
             << ", error: " << fabs(disparity - true_disparity) << endl;
        }
    }


    return 0;
}

void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    bool have_initial = !kp2.empty();

    for (size_t i = 0; i < kp1.size(); i++) {
        auto kp = kp1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (have_initial) {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        for (int iter = 0; iter < iterations; iter++) {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            cost = 0;

            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size) {   // go outside
                succ = false;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    // TODO START YOUR CODE HERE (~8 lines)
                    double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) - GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y);
                    Eigen::Vector2d J;  // Jacobian
                    if (inverse == false) {
                        // Forward Jacobian
                        J = -1.0 * Eigen::Vector2d(
                            0.5 * (GetPixelValue(img2, kp.pt.x + dx + x + 1, kp.pt.y + dy + y) - GetPixelValue(img2, kp.pt.x + dx + x - 1, kp.pt.y + dy + y)),
                            0.5 * (GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y + 1) - GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y - 1))
                        );
                    } else {
                        // Inverse Jacobian
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        J = -1.0 * Eigen::Vector2d(
                            0.5 * (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) - GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
                            0.5 * (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) - GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1))
                        );
                    }

                    // compute H, b and set cost;
                    H += J * J.transpose();
                    b += -error * J;
                    cost += error * error;
                    // TODO END YOUR CODE HERE
                }

            // compute update
            // TODO START YOUR CODE HERE (~1 lines)
            Eigen::Vector2d update;
            update = H.ldlt().solve(b);
            // TODO END YOUR CODE HERE

            if (isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if (iter > 0 && cost > lastCost) {
                cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;
        }

        success.push_back(succ);

        // set kp2
        if (have_initial) {
            kp2[i].pt = kp.pt + Point2f(dx, dy);
        } else {
            KeyPoint tracked = kp;
            tracked.pt += cv::Point2f(dx, dy);
            kp2.push_back(tracked);
        }
    }
}

void OpticalFlowSingleLevelMT(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
    ){
        int half_patch_size = 4;
        int iterations = 10;
        bool have_initial = !kp2.empty();

        success.resize(kp1.size());
        if (!have_initial) kp2.resize(kp1.size());
        cv::parallel_for_(cv::Range(0, kp1.size()), [&](const cv::Range& range)
        {
            for(int i = range.start; i < range.end; ++i)
            {
                auto kp = kp1[i];
                double dx = 0, dy = 0;
                if (have_initial) {
                    dx = kp2[i].pt.x - kp.pt.x;
                    dy = kp2[i].pt.y - kp.pt.y;
                }


                double cost = 0, lastCost = 0;
                bool succ = true;
                for(int iter = 0; iter < iterations; ++iter)
                {

                Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
                Eigen::Vector2d b = Eigen::Vector2d::Zero();
                cost = 0;

                if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                    kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size) {
                    succ = false;
                    break;
                }

                for(int x = -half_patch_size; x < half_patch_size; ++x)
                {
                    for(int y = -half_patch_size; y < half_patch_size; ++y)
                    {
                        double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) - GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y);
                        Eigen::Vector2d J;
                        if(!inverse)
                        {
                            J = -1.0 * Eigen::Vector2d(
                                0.5 * (GetPixelValue(img2, kp.pt.x + dx + x + 1, kp.pt.y + dy + y) -
                                       GetPixelValue(img2, kp.pt.x + dx + x - 1, kp.pt.y + dy + y)),
                                0.5 * (GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y + 1) -
                                       GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y - 1)));
                        }
                        else {
                            // inverse: 用 img1 的梯度
                            J = -1.0 * Eigen::Vector2d(
                                0.5 * (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) -
                                       GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
                                0.5 * (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) -
                                       GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1)));
                            }
                        H += J * J.transpose();
                        b += -error * J;
                        cost += error * error;
                    }
                }
                Eigen::Vector2d update = H.ldlt().solve(b);

                if (std::isnan(update[0])) {
                    succ = false;
                    break;
                }

                dx += update[0];
                dy += update[1];

                if (iter > 0 && cost > lastCost) break;
                lastCost = cost;
                }

            success[i] = succ;
            if (have_initial) {
                kp2[i].pt = kp.pt + Point2f(dx, dy);
            } else {
                kp2[i] = kp;
                kp2[i].pt += Point2f(dx, dy);
            }
        }
    });}

void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
) 
{

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE (~8 lines)
    for (int i = 0; i < pyramids; i++) {
        if(i == 0){
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        }
        else
        {
            Mat imgr1_ref = pyr1[i - 1];
            Mat imgr2_ref = pyr2[i - 1];

            Mat imgr1, imgr2;
            resize(imgr1_ref, imgr1, Size(imgr1_ref.cols * pyramid_scale, imgr1_ref.rows * pyramid_scale));
            resize(imgr2_ref, imgr2, Size(imgr2_ref.cols * pyramid_scale, imgr2_ref.rows * pyramid_scale));
            pyr1.push_back(imgr1);
            pyr2.push_back(imgr2);
        }
    }
    // TODO END YOUR CODE HERE

    // coarse-to-fine LK tracking in pyramids
    // TODO START YOUR CODE HERE
    vector<cv::KeyPoint> kp1_pyr, kp2_pyr;
    for(auto &kp: kp1)
    {
        auto kp_top = kp;
        kp_top.pt *= scales[pyramids - 1];
        kp1_pyr.push_back(kp_top);
        kp2_pyr.push_back(kp_top);
    }

    for(int level = 3; level >= 0; --level)
    {
        success.clear();
        OpticalFlowSingleLevel(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse);

        if(level > 0)
        {
            for(auto &kp: kp1_pyr){
                kp.pt /= pyramid_scale;
            }
            for(auto &kp: kp2_pyr){
                kp.pt /= pyramid_scale;
            }
        }

    }
    for(auto &kp: kp2_pyr)
    {
        kp2.push_back(kp);
    }
    // TODO END YOUR CODE HERE
    // don't forget to set the results into kp2
}

void OpticalFlowMultiLevelMT(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse)
{

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE (~8 lines)
    for (int i = 0; i < pyramids; i++) {
        if(i == 0){
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        }
        else
        {
            Mat imgr1_ref = pyr1[i - 1];
            Mat imgr2_ref = pyr2[i - 1];

            Mat imgr1, imgr2;
            resize(imgr1_ref, imgr1, Size(imgr1_ref.cols * pyramid_scale, imgr1_ref.rows * pyramid_scale));
            resize(imgr2_ref, imgr2, Size(imgr2_ref.cols * pyramid_scale, imgr2_ref.rows * pyramid_scale));
            pyr1.push_back(imgr1);
            pyr2.push_back(imgr2);
        }
    }
    // TODO END YOUR CODE HERE

    // coarse-to-fine LK tracking in pyramids
    // TODO START YOUR CODE HERE
    vector<cv::KeyPoint> kp1_pyr, kp2_pyr;
    for(auto &kp: kp1)
    {
        auto kp_top = kp;
        kp_top.pt *= scales[pyramids - 1];
        kp1_pyr.push_back(kp_top);
        kp2_pyr.push_back(kp_top);
    }

    for(int level = 3; level >= 0; --level)
    {
        success.clear();
        OpticalFlowSingleLevelMT(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse);

        if(level > 0)
        {
            for(auto &kp: kp1_pyr){
                kp.pt /= pyramid_scale;
            }
            for(auto &kp: kp2_pyr){
                kp.pt /= pyramid_scale;
            }
        }

    }
    for(auto &kp: kp2_pyr)
    {
        kp2.push_back(kp);
    }
    // TODO END YOUR CODE HERE
    // don't forget to set the results into kp2
}
