#pragma once

#include <google/protobuf/descriptor.h>
#include <google/protobuf/reflection.h>
#include <google/protobuf/dynamic_message.h>
#include <map>
#include <QString>
#include <QStringList>
#include <qvariant.h>
#include "student.pb.h"

#include <boost/multi_index/global_fun.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index_container.hpp>


// using spMessage = std::shared_ptr<google::protobuf::Message>;
using spPerson = std::shared_ptr<demo::Person>;
using sp_cPbMsg = std::shared_ptr<google::protobuf::Message const>;
using spPbMsg = std::shared_ptr<google::protobuf::Message>;
// extern std::map<size_t, spMessage> g_messages;

void InitPersons();


using namespace boost::multi_index;


namespace inner {
struct t0 {};
} // namespace inner

size_t ExactT0_ms(sp_cPbMsg msg);

using DataContainer = boost::multi_index_container<
    sp_cPbMsg,
    indexed_by<random_access<>, ordered_unique<tag<inner::t0>, global_fun<sp_cPbMsg, size_t, ExactT0_ms>>>>;


spPbMsg CreateMessageByName(std::string const& name);
double GetHeaderT0(google::protobuf::Message const &msg, QString const &name);
google::protobuf::Message const &GetHeader(google::protobuf::Message const &msg, QString const &name);
void PrintProtoMsg2(const google::protobuf::Message &message);
QStringList GetFildNames(const google::protobuf::Message &message, QString const &prefix = "");
QVariant ReflectVaule(const google::protobuf::Message &message, google::protobuf::Descriptor const *descriptor, google::protobuf::Reflection const *reflection,
                      google::protobuf::FieldDescriptor const *single_filed);
QVariant GetValueByHeaderName(const google::protobuf::Message &message, QString const &name);