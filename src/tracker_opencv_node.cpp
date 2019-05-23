#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
    // copies of images
    cv::Mat frame, frameRoi;
    bool gotFirstImage, okTrackerUpdate;
    // bounding box selects the template
    cv::Rect2d bbox;
    // List of all tracker types
    const std::string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    std::string trackerType;

    cv::Ptr<cv::Tracker> tracker;
    //std::vector<cv::Ptr<cv::Tracker>> vectorOfTrackers;
public:

    bool okTrackerInit;
    uint trackingFailedCounter;

    ImageConverter() {
        cv::namedWindow(OPENCV_WINDOW);
        gotFirstImage = false;
        okTrackerInit = false;
        okTrackerUpdate = false;
        //set default RoI for a tracker
        bbox = cv::Rect2d(0, 0, 0, 0);
        // select tracker type
        trackerType = trackerTypes[1];
        trackingFailedCounter = 0;
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }


    void createOpenCVMat(const sensor_msgs::ImageConstPtr &msg) {

        cv_bridge::CvImageConstPtr cv_ptr;
        uint32_t currSeqNumber;
        try {
            if (sensor_msgs::image_encodings::isColor(msg->encoding)) {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
                gotFirstImage = true;
            } else {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
                gotFirstImage = true;
            }
            currSeqNumber = cv_ptr->header.seq;
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Process cv_ptr->image using OpenCV
        //frame = cv_ptr->image.clone();
        cv_ptr->image.copyTo(frame);

        //frameRoi = cv_ptr->image.clone();
        cv_ptr->image.copyTo(frameRoi);

        // Display seq number of the Image topic
        putText(frame, "Seq: " + std::to_string(currSeqNumber),
                cv::Point(frame.cols - frame.cols / 4, frame.rows - frame.rows / 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 70, 50), 2);

    }

    void createTracker() {
        if (trackerType == "BOOSTING")
            tracker = cv::TrackerBoosting::create();
        else if (trackerType == "MIL")
            tracker = cv::TrackerMIL::create();
        else if (trackerType == "KCF")
            tracker = cv::TrackerKCF::create();
            //TODO try mode colour?
        else if (trackerType == "TLD")
            tracker = cv::TrackerTLD::create();
        else if (trackerType == "MEDIANFLOW")
            tracker = cv::TrackerMedianFlow::create();
        else if (trackerType == "GOTURN")
            tracker = cv::TrackerGOTURN::create();
        else if (trackerType == "MOSSE")
            tracker = cv::TrackerMOSSE::create();
        else if (trackerType == "CSRT")
            tracker = cv::TrackerCSRT::create();
        else {
            std::cerr << "Incorrect tracker name!" << std::endl;
            return;
        }
        //vectorOfTrackers.emplace_back(cv::TrackerMedianFlow::create());
        std::cout << trackerType << " tracker was created" << std::endl;

    }

    void destroyTracker() {
        tracker.release();
        //tracker.reset();
        delete tracker;
        okTrackerInit = false;
        std::cout << trackerType << " tracker was destroyed." << std::endl;
    }

    void initTracker() {
        // get new bbox
        bbox = cv::selectROI(OPENCV_WINDOW, frameRoi, false, false);

        // init tracker if there is an image, tracker was createdBefore(or it failed) and RoI exist
        if (gotFirstImage && !okTrackerInit && isBboxValid()) {
            okTrackerInit = tracker->init(frameRoi, bbox);
            trackingFailedCounter = 0;
            //okTrackerInit = vectorOfTrackers.back()->init(frameRoi, bbox);
            std::cout << "Was tracker initialized successfully? " << std::boolalpha << okTrackerInit << std::endl;
        } else {
            std::cout << "Can not initialize tracker since ROI is 0" << std::endl;
        }
    }

    void updateTracker() {
        okTrackerUpdate = tracker->update(frameRoi, bbox);
        //okTrackerUpdate = vectorOfTrackers.back()->update(frameRoi, bbox);
        if (okTrackerUpdate) {
            // Tracking success : Draw the tracked object
            rectangle(frame, bbox, cv::Scalar(255, 0, 0), 2, 1);
        } else {
            // Tracking failure detected : notification is shown
            putText(frame, "Tracking failure detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75,
                    cv::Scalar(0, 0, 255), 2);
            trackingFailedCounter++;
        }
        // Display tracker type on frame
        putText(frame, trackerType + " Tracker", cv::Point(100, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75,
                cv::Scalar(50, 170, 50), 2);

    }

    bool isBboxValid() {
        if (bbox.x >= 0 && bbox.y >= 0 && bbox.height > 0 && bbox.width > 0) {
            std::cout << "Bbox looks valid: " << bbox.x << " " << bbox.y << " " << bbox.height << " " << bbox.width
                      << std::endl;
            return true;
        } else {
            std::cerr << "Bbox does not look valid: " << bbox.x << " " << bbox.y << " " << bbox.height << " "
                      << bbox.width << std::endl;
            return false;
        }
    }
    bool isBboxEmpty() {
        return bbox.x == 0 && bbox.y == 0 && bbox.height == 0 && bbox.width == 0;
    }

    // Update GUI Window
    void updateGui(){
        cv::imshow(OPENCV_WINDOW, frame);
        cv::waitKey(1);
    }


};

int main(int argc, char **argv) {

    ros::init(argc, argv, "tracker_demo");

    rosbag::Bag bag;
    bag.open("/home/pangercic/Documents/ros_bags/2019-05-10-14-28-57.bag", rosbag::bagmode::Read);

    ImageConverter ic;

    std::string cam_image = "/camera/color/image_raw";
    std::vector<std::string> topics;
    topics.push_back(cam_image);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        sensor_msgs::ImageConstPtr imageTmpPtr = m.instantiate<sensor_msgs::Image>();
        if (imageTmpPtr != nullptr) {
            // get new image from a bag
            ic.createOpenCVMat(imageTmpPtr);

            // create bbox for the first time
            if (ic.isBboxEmpty()) {
                ic.createTracker();
                ic.initTracker();
            }

            char key = (char) cv::waitKey(1);
            if (key == 'n') {
                ic.destroyTracker();
                ic.createTracker();
                ic.initTracker();
            }

            if (key == 27) {
                break;
            }

            // reset tracking after 15 frames
            if (ic.trackingFailedCounter > 15) {
                std::cerr << "Tracking failed more than 15 times..." << std::endl;
                ic.destroyTracker();
                ic.createTracker();
                ic.initTracker();
            }
            if (ic.okTrackerInit) {
                ic.updateTracker();
            } else {
                std::cerr << "Tracker initialization failed" << std::endl;
            }

        }
        ic.updateGui();
    }

    bag.close();
    std::cout << "Stopping the program" << std::endl;
    return 0;
}