#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <string>
#include <QDebug>
#include "../include/serial_line/config.h"
#include "../include/serial_line/main_window.hpp"

#define DEG2RAD(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define RAD2DEG(angleRadians) ((angleRadians) * 180.0 / M_PI)

namespace serial_line
{


using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent): QMainWindow(parent), qnode(argc,argv)
{
  ui.setupUi(this);
  qnode.init();
  setWindowIcon(QIcon(":/images/icon.png"));

  init();
}

MainWindow::~MainWindow()
{
}

void MainWindow::init()
{
  qRegisterMetaType<sensor_msgs::JointState>("sensor_msgs::JointState");

  //init timer, connect to callback slot
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(timerCallback()));
  timer->start(1);

  //init serial, connect non-blocking callback
  commStatus = CommStatus::IDLE;
  conversationStatus = ConversationStatus::CONVERSATION_IDLE;
  serial.setPortName(PORT_NAME);
  serial.setBaudRate(BAUD_RATE);
  serial.setDataBits(QSerialPort::Data8);
  serial.setParity(QSerialPort::NoParity);
  serial.setStopBits(QSerialPort::OneStop);
  serial.setFlowControl(QSerialPort::NoFlowControl);
  connect(&serial, SIGNAL(readyRead()), this, SLOT(serialReadCallback()));

  qRegisterMetaType<QByteArray>("QByteArray");
  qRegisterMetaType<PacketGenerator::DataChange>("PacketGenerator::DataChange");
  qRegisterMetaType<std_msgs::ByteMultiArray>("std_msgs::ByteMultiArray");
  connect(&qnode, SIGNAL(serialWrite(QByteArray)), this, SLOT(serialWrite(QByteArray)));
  connect(&qnode, SIGNAL(processScueSet(PacketGenerator::DataChange)), this, SLOT(processScueSet(PacketGenerator::DataChange)));
  connect(&qnode, SIGNAL(translateJointStateMsg(sensor_msgs::JointState)), this, SLOT(translateJointStateMsg(sensor_msgs::JointState)));
  qDebug()<<"SerialLine"<<VERSION;

  m = createMobileMasterStructure();
}

void MainWindow::timerCallback()
{
  moderateComm();
  if(commStatus != CONNECTED) return;

  switch(conversationStatus)
  {
  case CONVERSATION_IDLE:
  {
    conversationStatus = CONVERSATION_REQUEST;
    break;
  }
  case CONVERSATION_REQUEST:
  {
    serialWrite(packetGenerator.generatePacket());
    conversationStatus = CONVERSATION_RESPONSE;
    timeoutCnt = 0;
    break;
  }
  case CONVERSATION_RESPONSE:
  {
    if(timeoutCnt++ > 60)
    {
      conversationStatus = CONVERSATION_REQUEST;
      qDebug()<<"Timeout!"<<endl;
    }
    break;
  }
  }
}

void MainWindow::serialReadCallback()
{
  QByteArray byteArray = serial.readAll();
  PacketTranslator::PacketInfo packetInfo;
  for(uint8_t byte : byteArray)
  {
    //qDebug()<<byte;
    if(packetTranslator.pushByte(packetInfo, byte))
    {
      qDebug()<<packetInfo.parameter.size()<<" / "<<sizeof(MobileMasterDataStructure)<<"received";
      if(packetInfo.parameter.size() == sizeof(MobileMasterDataStructure))
      {
        memcpy(&qnode.scueRead, (unsigned char*)&packetInfo.parameter[0], sizeof(MobileMasterDataStructure));
        qnode.publishScueRead();
        conversationStatus = CONVERSATION_REQUEST;
      }
    }
  }
}

void MainWindow::processScueSet(PacketGenerator::DataChange d)
{
  packetGenerator.addDataChange(d);

  //qDebug()<<"adding data change : address : "<<d.address;
}

void MainWindow::serialWrite(QByteArray packet)
{
  serial.write(packet);
}

void MainWindow::translateJointStateMsg(sensor_msgs::JointState msg)
{
}

void MainWindow::moderateComm()
{
  switch(commStatus)
  {
  case CommStatus::IDLE:
  {
    commStatus = CommStatus::CONNECTING;
    break;
  }
  case CommStatus::CONNECTING:
  {
    if(serial.open(QIODevice::ReadWrite))
    {
      commStatus = CommStatus::CONNECTED;
      qDebug()<<"Port opened.";
    }
    else
    {
      qDebug()<<"Port open failed.";
    }
    break;
  }
  case CommStatus::CONNECTED:
  {
    break;
  }
  }
}

}
