#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

static int mode=0;
static int max_num_points=0;
const int &ORB_SLAM2::LocalMapping::MODE=mode;
const int &ORB_SLAM2::LocalMapping::MAX_NUM=max_num_points;
static float thR=0.05f;
const float &ORB_SLAM2::Tracking::theshR=thR;
static float thT=0.5f;
const float &ORB_SLAM2::Tracking::theshT=thT;


void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc < 10) {
        cerr << endl << "Usage: ./stereo_euroc [vocabulary_path] [settings_path] [cam0_seq_path] [cam1_seq_path] [timestamp_path] [result_path] [log_path] [thR] [thT] [mode] (max_num_param)\n" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamp;
    LoadImages(string(argv[3]), string(argv[4]), string(argv[5]), vstrImageLeft, vstrImageRight, vTimeStamp);

    if(vstrImageLeft.empty() || vstrImageRight.empty()) {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }

    if(vstrImageLeft.size()!=vstrImageRight.size()) {
        cerr << "ERROR: Different number of left and right images." << endl;
        return 1;
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);


    const int nImages = vstrImageLeft.size();

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ofstream resultFile;
    resultFile.open(argv[6], ios::app | ios::ate);

    thR = atof(argv[8]);
    thT = atof(argv[9]);
    
    stringstream logpath;

    if(atoi(argv[10])==0) {
        cout << "mode: original"  << endl;
        resultFile << "original, ";
        logpath << string(argv[7]) << "/original";
    } else if(atoi(argv[10])==1) {
        cout << "mode: cull" << endl;
        if (!argv[11]) {
            cerr << "max_num_points not given!" <<endl;
            return 1;
        }
        cout << "max_num_points is set to " << argv[11] << endl;
        mode = 1;
        max_num_points = atoi(argv[11]);
        resultFile << "cull(" << argv[11] << "), ";
        logpath << string(argv[7]) << "/cull(" << string(argv[11]) << ")";
    } else  {
        cerr << "invalid mode!" << endl;
        return 1;
    }

    cout << endl << "-------" << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    chrono::steady_clock::time_point st1 = chrono::steady_clock::now();
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;

    // Main loop
    for(int ni=0; ni<nImages; ni++)
    {
        imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED);

        if(imLeft.empty()) {
            cerr << endl << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imRight.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageRight[ni]) << endl;
            return 1;
        }

        chrono::steady_clock::time_point t_Start_Rect = chrono::steady_clock::now();
        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);
        chrono::steady_clock::time_point t_End_Rect = chrono::steady_clock::now();

        double t_rect = chrono::duration_cast<chrono::duration<double,milli> >(t_End_Rect - t_Start_Rect).count();
        SLAM.InsertRectTime(t_rect);
        double tframe = vTimeStamp[ni];

        SLAM.TrackStereo(imLeftRect,imRightRect,tframe);
    }
    chrono::steady_clock::time_point st2 = chrono::steady_clock::now();
    double totalRuntime = chrono::duration_cast<chrono::duration<double, milli> >(st2 - st1).count();

    resultFile << totalRuntime << ", " ;
    resultFile.close();
    SLAM.Shutdown();

    ///////////////////////////////////////////
    // SLAM.SaveMap(logpath.str().c_str());
    // SLAM.SaveMapPoints(logpath.str().c_str());
    ///////////////////////////////////////////
    SLAM.SaveStats(argv[6], logpath.str().c_str());

    // SLAM.SaveKeyFrameTrajectoryTUM(logpath.str() + "KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM(logpath.str() + "CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}
