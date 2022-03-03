#include <ecn_common/color_detector.h>
#include <sstream>
#include <sensor_msgs/Image.h>
#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <tinyxml.h>


using namespace std;
//using ecn::ColorDetector;

cv::Mat im;

bool im_ok;
ros::Time stamp;

void readImage(const sensor_msgs::ImageConstPtr msg)
{
    im_ok = true;
    im = cv_bridge::toCvCopy(msg, "bgr8")->image;
    std_msgs::Header h = msg->header;
    stamp = h.stamp.now();
}

int main(int argc, char** argv)
{

    TiXmlDocument doc("../../../../../src/integ_description/urdf/arm.urdf.xacro");
    if(!doc.LoadFile()){
        cerr << "erreur lors du chargement" << endl;
        cerr << "error #" << doc.ErrorId() << " : " << doc.ErrorDesc() << endl;
        return 1;
    }


    // subscribe to images
    ros::init(argc, argv, "color_detector");

    im_ok = false;

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imsub = it.subscribe("/robot/camera1/image_raw", 1, &readImage);
    ros::Publisher impub = nh.advertise<geometry_msgs::PointStamped>("masscenter", 1000);
    // init color detector
    geometry_msgs::PointStamped mass_center;
    int r = 255, g = 0, b = 0;
    if(argc == 4)
    {
        r = atoi(argv[1]);
        g = atoi(argv[2]);
        b = atoi(argv[3]);
    }
    ecn::ColorDetector cd(r, g, b);
    cd.showSegmentation();  // also gives trackbars for saturation / value
    cd.showOutput();
    cd.fitCircle();

    ros::Rate loop(10);

    while(ros::ok())
    {
        if(im_ok)
        {
            mass_center.header.stamp = stamp;
            mass_center.point = cd.process(im);
            impub.publish(mass_center);
        }

        loop.sleep();
        ros::spinOnce();
    }
}
