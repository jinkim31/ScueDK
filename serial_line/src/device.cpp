#include "../include/serial_line/device.h"
#include <QString>

using namespace serial_line;

Device::Device(int id, int parameterArraySize)
{
  mDesiredParameterArray.resize(parameterArraySize);
  mChangeFlagArray.resize(parameterArraySize);
  mChangeFlagArray.fill(false);
  mId = id;
  mIsChanged = false;
  mName = "unknown";
}


Device::Device(QString name, int id, int parameterArraySize)
{
  mDesiredParameterArray.resize(parameterArraySize);
  mChangeFlagArray.resize(parameterArraySize);
  mChangeFlagArray.fill(false);
  mId = id;
  mIsChanged = false;
  mName = name;
}

void Device::changeParameter(int index, float value)
{
  mDesiredParameterArray[index] = value;
  mChangeFlagArray[index] = true;
  mIsChanged = true;
}

bool Device::isChanged()
{
  return mIsChanged;
}

void Device::clear()
{
  mChangeFlagArray.fill(false);
  mIsChanged = false;
}

int Device::id()
{
  return mId;
}

QString Device::name()
{
  return mName;
}

QVector<float> Device::desiredParameterArray()
{
  return mDesiredParameterArray;
}

QVector<bool> Device::changeFlagArray()
{
  return mChangeFlagArray;
}


