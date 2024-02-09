#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include <include/backend.h>
#include <include/imageitem.h>

#define CV_LOAD_IMAGE_COLOR 0

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  //ros::NodeHandle nh;

  // image_transport::ImageTransport it(nh);
  // image_transport::Publisher pub = it.advertise("camera/image", 1);
  

  #if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  #endif
  #if (QT_VERSION >= QT_VERSION_CHECK(5, 9, 0))
      QCoreApplication::setAttribute(Qt::AA_DisableShaderDiskCache);
  #endif

//   image_transport::ImageTransport it(nh);
//   image_transport::Publisher pub = it.advertise("camera/image", 1);
//   cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
//   cv::waitKey(30);
//   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
#if 0
  ros::Rate loop_rate(5);
  while (nh.ok()) {
    std::cout << "Hello\n";
    //pub.publish(msg);
    ros::spinOnce();
    //loop_rate.sleep();
  }

#endif

    QGuiApplication app(argc, argv);

    qRegisterMetaType<QImage*>("QImage *");
    qmlRegisterType<BackEnd>("BackEnd", 1, 0, "BackEnd");
    qmlRegisterType<ImageItem>("ImageItem", 1, 0, "ImageItem");
    
    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();


}