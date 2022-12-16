#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>

#include<System.h>
#include <experimental/filesystem>
 
 
 
namespace fs = std::experimental::filesystem;

using namespace std;

static int mode=0;
static int max_num_points=0;
const int &ORB_SLAM2::LocalMapping::MODE=mode;
const int &ORB_SLAM2::LocalMapping::MAX_NUM=max_num_points;
static float thR=0.05f;
const float &ORB_SLAM2::Tracking::theshR=thR;
static float thT=0.5f;
const float &ORB_SLAM2::Tracking::theshT=thT;


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc < 6)
    {
        cerr << endl << "Usage: ./rgbd_tum [vocabulary_path] [settings_path] [sequence_path] [result_path] [log_path] [thR] [thT] [mode] (max_num_param)\n" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    LoadImages(argv[3], vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    // ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,false);


    ofstream resultFile;
    resultFile.open(argv[4], ios::app | ios::ate);

    thR = atof(argv[6]);
    thT = atof(argv[7]);
    
    stringstream logpath;
    stringstream mapsavepath;
    bool LocalMod = !SLAM.mapfile.empty();
    if (LocalMod)
    {
        vector<string> names;
        string temp;
        stringstream ss(SLAM.mapfile);
        while(getline(ss, temp, '/')){
            names.push_back(temp);
        }
        string name;
        name = names.back().substr(0, names.back().length() - 4);
        cout << name << endl;
        resultFile << "local_" << name << ", ";
        logpath << string(argv[5]) << "/" << "local_" << name;
    } else {
        if(atoi(argv[8])==0)
        {
            cout << "mode: original"  << endl;
            resultFile << "original, ";
            logpath << string(argv[5]) << "/original";
            mapsavepath << string(argv[5]) << "/map/original";
        }
        else if(atoi(argv[8])==1)
        {
            cout << "mode: cull" << endl;
            if (!argv[9])
            {
                cerr << "max_num_points not given!" <<endl;
                return 1;
            }
            cout << "max_num_points is set to " << argv[9] << endl;
            mode = 1;
            max_num_points = atoi(argv[9]);
            resultFile << "cull(" << argv[9] << "), ";
            logpath << string(argv[5]) << "/cull(" << string(argv[9]) << ")";
            mapsavepath << string(argv[5]) << "/map/cull(" << string(argv[9]) << ")";

        }
    }

    cout << endl << "-------" << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    chrono::steady_clock::time_point st1 = chrono::steady_clock::now();
    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED);
        imD = cv::imread(vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        cv::resize(imRGB, imRGB, cv::Size(640, 480));

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);
    }
    chrono::steady_clock::time_point st2 = chrono::steady_clock::now();
    double totalRuntime = chrono::duration_cast<chrono::duration<double, milli> >(st2 - st1).count();

    resultFile << totalRuntime << ", ";
    resultFile.close();
    SLAM.Shutdown();
  
    SLAM.SaveTrajectoryKITTI(logpath.str()+"CameraTrajectory.txt");

    if (!LocalMod) {
        cout << "hey~" << endl;
        SLAM.SaveMap(mapsavepath.str().c_str());
        SLAM.SaveStats(argv[4], logpath.str().c_str());
        // SLAM.SaveMapPoints(mapsavepath.str().c_str());
    }

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    auto dirIter = fs::directory_iterator(strPathToSequence+"/rgb/");

    int fileCount = std::count_if(
        begin(dirIter),
        end(dirIter),
        [](auto& entry) {return fs::is_regular_file(entry.status());}
    );

    string strPrefixRGB = strPathToSequence + "/rgb/";
    string strPrefixD = strPathToSequence + "/depth/";

    cout << "# images: " << fileCount << endl;    
    vTimestamps.resize(fileCount);
    vstrImageFilenamesRGB.resize(fileCount);
    vstrImageFilenamesD.resize(fileCount);

    for(int i=0; i<fileCount; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenamesRGB[i] = strPrefixRGB + "frame-" + ss.str() + ".color.jpg";
        vstrImageFilenamesD[i] = strPrefixD + "frame-" + ss.str() + ".depth.pgm";
        vTimestamps[i] = stod(ss.str());
    }
}
