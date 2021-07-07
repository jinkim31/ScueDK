/**
 * @file /include/SerialLine/main_window.hpp
 *
 * @brief Qt based gui for SerialLine.
 *
 * @date November 2010
 **/
#ifndef SerialLine_MAIN_WINDOW_H
#define SerialLine_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QTime>
#include "QtSerialPort/QSerialPort"
#include "QtSerialPort/QSerialPortInfo"
#include "packetgenerator.h"
#include "packettranslator.h"
#include "device.h"
#include "QTimer"
#include "string.h"
#include "sensor_msgs/JointState.h"
#include "DataStructure.h"

namespace serial_line{

enum InitSequence
{
  INIT_DXL_3,
  INIT_DXL_6,
  INIT_DXL_7,
  INIT_DXL_8,
  INIT_DXL_9,
  INIT_DXL_3_ACC,
  INIT_DXL_6_ACC,
  INIT_DXL_7_ACC,
  INIT_DXL_8_ACC,
  INIT_DXL_9_ACC,
  INIT_DONE,
};

class MainWindow : public QMainWindow {
Q_OBJECT

public:
        MainWindow(int argc, char** argv, QWidget *parent = 0);
        ~MainWindow();
Q_SIGNALS:
        void publishRobotRead();
public Q_SLOTS:
        void timerCallback();
        void serialReadCallback();
        void serialWrite(QByteArray packet);
        void translateJointStateMsg(sensor_msgs::JointState msg);
        void processScueSet(PacketGenerator::DataChange d);

private:
        InitSequence initSequence;
        PacketGenerator packetGenerator;
        PacketTranslator packetTranslator;
        enum CommStatus
        {
            IDLE,
            CONNECTING,
            CONNECTED,
        };

        enum ConversationStatus
        {
          CONVERSATION_IDLE,
          CONVERSATION_REQUEST,
          CONVERSATION_RESPONSE
        };

        ConversationStatus conversationStatus;
        CommStatus commStatus;

        void moderateComm();
        void init();
        Ui::MainWindowDesign ui;
        QNode qnode;
        QSerialPort serial;
        QTimer* timer;
        MobileMasterDataStructure m;
        int timeoutCnt = 0;
};

}  // namespace SerialLine

#endif // SerialLine_MAIN_WINDOW_H
