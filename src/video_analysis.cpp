#include <fstream>
#include <iostream>
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <string>
#include <vector>
#include <map>

#include <signal.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/CompressedImage.h>
#include <boost/program_options.hpp>

#include "json.hpp"

#define STR_EXPAND(tok) #tok
#define STR(tok) STR_EXPAND(tok)

using namespace std;
using namespace cv;
using namespace nlohmann; // json
namespace po = boost::program_options;

// defines a couple of colours, for feature rendering
const Scalar WHITE(255,255,255);
const Scalar GREEN ( 62,131, 43);
const Scalar RED ( 57, 68,174);

//const string BAG_FILE ("freeplay.bag");
//const string POSES_FILE ("freeplay.poses.json");
const string BAG_FILE ("visual_tracking.bag");
const string POSES_FILE ("visual_tracking.poses.json");

const float FEATURE_LOW_CONFIDENCE_THRESHOLD = 0.05; // skeleton features with lower threshold are not reliable enough
const float PUPILS_CONFIDENCE_THRESHOLD = 0.8;

const uint NB_SKEL_FEATURES = 18; // 18 skeleton features
const uint NB_FACE_FEATURES = 70; // 70 facial landmarks (including pupils)
const uint NB_HAND_FEATURES = 21; // 21 x 2 hands features

const float SANDTRAY_LENGTH=600.; //mm
const float SANDTRAY_WIDTH=340.; //mm


// signal handler to stop the program with ctrl+c
bool interrupted = false;
void my_handler(int s){
    cout << "Caught signal " << s << endl;
    interrupted = true; 
}

/**
 * Iterates over the skeleton features, and draw a point on each of them.
 */
cv::Mat drawSkeleton(cv::Mat image, const json& skel) {

    auto w = image.size().width;
    auto h = image.size().height;

    // even though it should not happen, more than one skeleton might have been
    // detected, so we iterate over all of them.
    for(uint skel_idx = 1; skel_idx <= skel.size(); skel_idx++) { for(uint i =
            1; i < NB_SKEL_FEATURES; i++) {

        // Feature coordinates are normalized floats: 0.0 =< x|y =< 1.0
        auto x = skel[to_string(skel_idx)][i][0].get<float>();
        auto y = skel[to_string(skel_idx)][i][1].get<float>();
        auto confidence = skel[to_string(skel_idx)][i][2].get<float>();

        if (confidence < FEATURE_LOW_CONFIDENCE_THRESHOLD) continue;

        Point2f p(w * x, h * y);
        cv::circle(image, p, 3, WHITE, -1, cv::LINE_AA);
    }
    }

    return image;
}

/**
 * Iterates over the facial landmarks, and draw a point on each of them.
 */
cv::Mat drawFace(cv::Mat image, const json& face) {

    auto w = image.size().width;
    auto h = image.size().height;

    for(uint face_idx = 1; face_idx <= face.size(); face_idx++) {

        // facial landmarks, except pupils
        for(uint i = 0; i < NB_FACE_FEATURES - 2; i++) {

            auto x = face[to_string(face_idx)][i][0].get<float>();
            auto y = face[to_string(face_idx)][i][1].get<float>();
            auto confidence = face[to_string(face_idx)][i][2].get<float>();

            if (confidence < FEATURE_LOW_CONFIDENCE_THRESHOLD) continue;

            Point2f p(w * x, h * y);
            cv::circle(image, p, 2, GREEN, -1, cv::LINE_AA);
            // display as well the landmark ID -- useful to know for Action Units for instance
            cv::putText(image, " " + to_string(i), p, FONT_HERSHEY_DUPLEX, 0.2, WHITE);
        }

        // pupils
        for(uint i = 68; i < NB_FACE_FEATURES; i++) {

            auto x = face[to_string(face_idx)][i][0].get<float>();
            auto y = face[to_string(face_idx)][i][1].get<float>();
            auto confidence = face[to_string(face_idx)][i][2].get<float>();

            if (confidence < PUPILS_CONFIDENCE_THRESHOLD) continue;

            Point2f p(w * x, h * y);
            cv::circle(image, p, 3, RED, -1, cv::LINE_AA);
        }
    }

    return image;
}

/**
 * Iterates over the hands features, and draw a point on each of them.
 */
cv::Mat drawHands(cv::Mat image, const json& hand) {

    auto w = image.size().width;
    auto h = image.size().height;

    for(uint hand_idx = 1; hand_idx <= hand.size(); hand_idx++) {
        for(auto handeness : {"left", "right"}) {
            for(uint i = 0; i < NB_HAND_FEATURES; i++) {
                auto x = hand[to_string(hand_idx)][handeness][i][0].get<float>();
                auto y = hand[to_string(hand_idx)][handeness][i][1].get<float>();
                auto confidence = hand[to_string(hand_idx)][handeness][i][2].get<float>();

                if (confidence < FEATURE_LOW_CONFIDENCE_THRESHOLD) continue;

                Point2f p(w * x, h * y);
                cv::circle(image, p, 2, RED, -1, cv::LINE_AA);
            }
        }
    }

    return image;
}


/**
 * Calls one after the other the 3 drawing functions (skeleton, face, hands)
 */
cv::Mat drawPose(cv::Mat image, const json& frame) {

    image = drawSkeleton(image, frame["poses"]);
    image = drawHands(image, frame["hands"]);
    image = drawFace(image, frame["faces"]);

    return image;

}



int main(int argc, char **argv) {

    ////////////////////////////////////////////////////////////////////// 
    // configure the signal handler to be able to interrupt the tool with
    // ctrl+c

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    ////////////////////////////////////////////////////////////////////// 



    ////////////////////////////////////////////////////////////////////// 
    // Command-line program options
    //
    po::positional_options_description p;
    p.add("path", 1);

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produces help message")
        ("version,v", "shows version and exits")
        ("path", po::value<string>(), "record path (must contain freeplay.bag and freeplay.poses.json)")
        ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv)
            .options(desc)
            .positional(p)
            .run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << argv[0] << " " << STR(FREEPLAY_ANALYSIS_VERSION) << "\n\n" << desc << "\n";
        return 1;
    }

    if (vm.count("version")) {
        cout << argv[0] << " " << STR(FREEPLAY_ANALYSIS_VERSION) << "\n";
        return 0;
    }

    if (!vm.count("path")) {
        cerr << "You must provide a record path.\n";
        return 1;
    }
    ////////////////////////////////////////////////////////////////////// 


    // those are the RGB video topics that we want to read from the bag file
    std::vector<std::string> topics {"camera_purple/rgb/image_raw/compressed", 
                                     "camera_yellow/rgb/image_raw/compressed"};


    int total_nb_frames = 0;

    ////////////////////////////////////////////////////////////////////// 
    // Loads freeplay.bag and freeplay.poses.json
    //
    rosbag::Bag bag;
    rosbag::View view;

    cerr << "Opening " << vm["path"].as<string>() << "/" << BAG_FILE << " (this might take up to a few minutes, depending on your hard-drive)..." << endl;
    bag.open(vm["path"].as<string>() + "/" + BAG_FILE, rosbag::bagmode::Read);
    view.addQuery(bag, rosbag::TopicQuery(topics));
    total_nb_frames = view.size();
    if (total_nb_frames == 0) {
        cerr << "Found no image messages for given topic in " << BAG_FILE << ". Aborting." << endl;
        exit(1);
    }

    // freeplay.poses.json
    json root;

    auto start = std::chrono::system_clock::now();

    cerr << "Opening " << POSES_FILE << "..." << flush;
    std::ifstream file(vm["path"].as<string>() + "/" + POSES_FILE);
    file >> root;

    auto end = std::chrono::system_clock::now();
    auto elapsed = end - start;

    cerr << "done (took " << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() << "s)" << endl << endl;

    for (const auto& topic : topics) {
        auto nb_frames = root[topic]["frames"].size();
        if (nb_frames == 0) {
            cerr << "Found no frames for topic " << topic << " in " << POSES_FILE << ". Aborting." << endl;
            exit(1);
        }
        cerr << "Found " << nb_frames << " poses to render for topic " << topic << endl << endl;
    }
    ////////////////////////////////////////////////////////////////////// 

    // Keeps track of how many frames we have seen on each topic
    // This is needed to match the ROS video frame with the corresponding entry
    // in freeplay.poses.json
    map<string, size_t> topicsIndices;

    Size windowSize(960 * topics.size(), 540);
    Mat image(windowSize, CV_8UC3, Scalar(0,0,0));

    int idx = 0;
    int last_percent = 0;

    // iterate over each of the ROS messages in 'view' (ie, the 2 RGB video streams)
    for(rosbag::MessageInstance const m : view)
    {
        idx++;

        // iterate over the topics of interest
        for (size_t t_idx = 0; t_idx < topics.size(); t_idx++) {

            auto topic = topics[t_idx];

            if (m.getTopic() == topic || ("/" + m.getTopic() == topic)) {

                auto compressed_rgb = m.instantiate<sensor_msgs::CompressedImage>();

                if (compressed_rgb != NULL) {

                    topicsIndices[topic] += 1;

                    // the ROS video stream are compressed as PNG. Simply
                    // calling OpenCV's imdecode on the raw, compressed data,
                    // returns the uncompressed image.
                    auto camimage = imdecode(compressed_rgb->data,1);
                    Rect roi( Point( 960 * t_idx, 0 ), camimage.size() );


                    // draw the skeleton, facial landmarks and hand on top of
                    // this frame
                    camimage = drawPose(camimage, root[topic]["frames"][topicsIndices[topic]]);

                    // insert the resulting image into final displayed image
                    camimage.copyTo(image(roi));
                }

            }

        }

        // once we've seen at least one frame per topic, we update the display
        if (idx % topics.size() == 0) {
            imshow("Video replay -- press Space to pause, Esc to quit", image);

            auto k = waitKey(30) & 0xFF;
            if (k == 27) interrupted = true;
            if (k == 32) { // space to pause
                while (true) {
                    if ((waitKey(30) & 0xFF) == 32) break;
                }
            }
        }

        int percent = idx * 100 / total_nb_frames;
        if (percent != last_percent) {
            cerr << "\x1b[FDone " << percent << "% (" << idx << " images)" << endl;
            last_percent = percent;
        }

        if(interrupted) {
            cerr << "Interrupted." << endl;
            break;
        }
    }


    bag.close();
}
