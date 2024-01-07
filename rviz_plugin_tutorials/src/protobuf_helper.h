#pragma once

#include <google/protobuf/descriptor.h>
#include <google/protobuf/reflection.h>
#include <map>
#include <QString>
#include <QStringList>
#include <qvariant.h>
#include "student.pb.h"

using spMessage = std::shared_ptr<google::protobuf::Message>;
using spPerson = std::shared_ptr<demo::Person>;

extern std::map<size_t, spMessage> g_messages;

void InitPersons();
void PrintProtoMsg2(const google::protobuf::Message &message);
QStringList GetFildNames(const google::protobuf::Message &message, QString const &prefix = "");
QVariant ReflectVaule(const google::protobuf::Message &message, google::protobuf::Descriptor const *descriptor, google::protobuf::Reflection const *reflection,
                      google::protobuf::FieldDescriptor const *single_filed);
QVariant GetValueByHeaderName(const google::protobuf::Message &message, QString const &name);