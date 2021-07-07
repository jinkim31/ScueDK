#ifndef DEVICE_H
#define DEVICE_H

#include "QVector"
#include "QDebug"

namespace serial_line
{

class Device
{
private:
  QVector<float> mDesiredParameterArray;
  QVector<bool> mChangeFlagArray;
  int mParameterArraySize;
  int mId;
  bool mIsChanged;
  QString mName;
public:
  Device(int id, int parameterArraySize);
  Device(QString name,int id, int parameterArraySize);
  void changeParameter(int index, float value);
  bool isChanged();
  void clear();
  int id();
  QString name();
  QVector<float> desiredParameterArray();
  QVector<bool> changeFlagArray();

};
}

#endif
