#ifndef SerialLine_QNODE_HPP_
#define SerialLine_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include "QtSerialPort/QSerialPortInfo"
#include "std_msgs/ByteMultiArray.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/ByteMultiArray.h"
#include "packetgenerator.h"
#include <QTime>
#include <QVector>
#include <QList>
#include "DataStructure.h"

namespace serial_line
{
class QNode : public QThread
{
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();
    void publishScueRead();
    std_msgs::ByteMultiArray scueSetMsg;
    MobileMasterDataStructure scueRead;


Q_SIGNALS:
    void rosShutdown();
    void serialWrite(QByteArray packet);
    void translateJointStateMsg(sensor_msgs::JointState msg);
    void processScueSet(PacketGenerator::DataChange d);

private:
    ros::Publisher robotReadPub;
    ros::Subscriber operatorRobotSetSub;
    ros::Subscriber jointSetSub;
    ros::Subscriber scueSetSub;
    void scueSetCallback(const std_msgs::ByteMultiArray &msg);
    void operatorMsgCallback(const sensor_msgs::JointState &msg);
    int init_argc;
    char** init_argv;
};

}
#endif
