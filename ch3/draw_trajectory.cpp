#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;
typedef vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3 >> TrajectoryType;
TrajectoryType readPose(const string& path);
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses);
void DrawAllTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>(&a_poses)[3]);
// path to trajectory file
string trajectory_file = "./trajectory.txt";
string estimated_file = "./estimated.txt";
string groundtruth_file = "./groundtruth.txt";
// function for plotting trajectory, don't edit this code
// start point is red and end point is blue

int main(int argc, char **argv) {

    //TrajectoryType poses;

    /// implement pose reading code
    // start your code here (5~10 lines)
    TrajectoryType poses = readPose(trajectory_file);
    TrajectoryType estimated = readPose(estimated_file);
    TrajectoryType groundtruth = readPose(groundtruth_file);

    double rmse = 0.0;
    for(size_t i = 0; i < estimated.size(); ++i)
    {
	Sophus::SE3 est = estimated[i];
        Sophus::SE3 ground = groundtruth[i];
        double error = (ground.inverse() * est).log().norm();
        rmse += error * error;
    }
    rmse = rmse / double(estimated.size());
    rmse = sqrt(rmse);
    cout << "rmse: " << rmse << endl;
    // end your code here

    // draw trajectory in pangolin
    TrajectoryType poses_set[3] = {poses, estimated, groundtruth};
    DrawAllTrajectory(poses_set);
    //DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
TrajectoryType readPose(const string& path)
{
    ifstream fin(path);
    TrajectoryType trajectory;
    if(!fin)
    {
    	cerr << "trajectory " << path << " not found." << endl;
    	return trajectory;
    }
    while(!fin.eof())
    {
    	double time, tx, ty, tz, qx, qy, qz, qw;
    	fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    	Sophus::SE3 p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
    	trajectory.push_back(p1);
    }
    return trajectory;
}

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}

void DrawAllTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>(&a_poses)[3])
{
   for(int i=0; i < 3; ++i)
   {
   	if (a_poses[i].empty()) {
        cerr << "Trajectory " << i << " is empty!" << endl;
        return;
    }
   }
   // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	d_cam.Activate(s_cam);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	glLineWidth(2);
	for(size_t size=0; size < 3; ++size)
	{
	  for (size_t i = 0; i < a_poses[size].size() - 1; i++) {
	      glColor3f(1 - (float) i / a_poses[size].size(), 0.0f, (float) i / a_poses[size].size());
	      glBegin(GL_LINES);
	      auto p1 = a_poses[size][i], p2 = a_poses[size][i + 1];
	      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
	      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
	      glEnd();
	  }
	}
	pangolin::FinishFrame();
	usleep(5000);   // sleep 5 ms
    }
}
