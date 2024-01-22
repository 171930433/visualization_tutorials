#include "protobuf_helper.h"
#include <iostream>
#include <qmath.h>
#include <QDebug>

// std::map<size_t, spMessage> g_messages;
bool g_demo_data_inited = false;

void InitPersons()
{
  if (g_demo_data_inited)
  {
    return;
  }

  int n = 1 * 1000; // number of points in graph
  double xScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double yScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double xOffset = (std::rand() / (double)RAND_MAX - 0.5) * 4;
  double yOffset = (std::rand() / (double)RAND_MAX - 0.5) * 10;
  double r1 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r2 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r3 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r4 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double current_time = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
  for (int i = 0; i < n; i++)
  {
    size_t time_index = (current_time + i * 0.01) * 1e3;
    spPerson person = std::make_shared<demo::Person>();
    person->set_name("1");
    auto header = person->mutable_header();
    header->set_t0(current_time + i * 0.01);
    header->set_index(i);
    auto pos = person->mutable_pos();
    pos->set_x((i / (double)n - 0.5) * 10.0 * xScale + xOffset);
    pos->set_y((qSin(pos->x() * r1 * 5) * qSin(qCos(pos->x() * r2) * r4 * 3) + r3 * qCos(qSin(pos->x()) * r4 * 2)) * yScale + yOffset);
    // vel
    auto vel = person->mutable_vel();
    vel->set_x(5 * sin(M_PI * 2 / n * i)), vel->set_y(5 * cos(M_PI * 2 / n * i)), vel->set_z(i * 1.0 / n);
    // att
    auto att = person->mutable_att();
    att->set_x(3 * sin(M_PI * 2 / n * i)), att->set_y(3 * cos(M_PI * 2 / n * i)), att->set_z(i * 1.0 / n);

    // g_messages[time_index] = person;
    //
    spPerson person2 = std::make_shared<demo::Person>(*person);
    person2->set_name("2");
    person2->mutable_pos()->set_y(person2->mutable_pos()->y() + 1);

    // 添加到cacher
    g_cacher_->push_back("/demo/trj1", header->t0(), person);
    g_cacher_->push_back("/demo/trj2", header->t0(), person2);
  }
  g_demo_data_inited = true;
}

spPbMsg CreateMessageByName(std::string const &name)
{
  using namespace google::protobuf;
  spPbMsg message = nullptr;
  auto *descriptor = DescriptorPool::generated_pool()->FindMessageTypeByName(name);
  // std::cout << descriptor->DebugString() << "\n";
  if (descriptor)
  {
    auto *prototype = MessageFactory::generated_factory()->GetPrototype(descriptor);
    if (prototype)
    {
      message.reset(prototype->New());
    }
  }
  return message;
}

QStringList GetFildNames(const google::protobuf::Message &message, QString const &prefix)
{
  QStringList result;
  using namespace google::protobuf;
  auto *descriptor = message.GetDescriptor();
  auto *reflection = message.GetReflection();
  int fields = descriptor->field_count();
  for (int i = 0; i < fields; ++i)
  {
    auto *single_filed = descriptor->field(i);
    QString parent = (prefix == "" ? "" : prefix + "-");
    if (single_filed->type() == FieldDescriptor::TYPE_MESSAGE)
    {
      auto const &msg = reflection->GetMessage(message, single_filed);
      result << GetFildNames(msg, parent + QString::fromStdString(single_filed->name()));
    }
    else
    {
      result << parent + QString::fromStdString(single_filed->name());
    }
  }
  return result;
}

QVariant ReflectVaule(const google::protobuf::Message &message, google::protobuf::Descriptor const *descriptor, google::protobuf::Reflection const *reflection,
                      google::protobuf::FieldDescriptor const *single_filed)
{
  using namespace google::protobuf;

  QVariant result;
  int repeated_counts = single_filed->is_repeated() ? reflection->FieldSize(message, single_filed) : 0;

  switch (single_filed->type())
  {
  case FieldDescriptor::TYPE_INT32:
  case FieldDescriptor::TYPE_SINT32:
  case FieldDescriptor::TYPE_FIXED32:
  {
    if (repeated_counts)
    {
      for (int j = 0; j < repeated_counts; ++j)
      {
        std::cout << " int32 repeated " << reflection->GetRepeatedInt32(message, single_filed, j) << "\n";
      }
    }
    else
    {
      result = QVariant::fromValue(reflection->GetInt32(message, single_filed));
    }
  }
  break;
  case FieldDescriptor::TYPE_INT64:
  case FieldDescriptor::TYPE_SINT64:
  case FieldDescriptor::TYPE_FIXED64:
  {
    if (repeated_counts)
    {
      for (int j = 0; j < repeated_counts; ++j)
      {
        std::cout << " int32 repeated " << reflection->GetRepeatedInt64(message, single_filed, j) << "\n";
      }
    }
    else
    {
      result = QVariant::fromValue(reflection->GetInt64(message, single_filed));
    }
  }
  break;
  case FieldDescriptor::TYPE_FLOAT:
  case FieldDescriptor::TYPE_DOUBLE:
  {
    if (repeated_counts)
    {
      for (int j = 0; j < repeated_counts; ++j)
      {
        std::cout << " int32 repeated " << reflection->GetRepeatedDouble(message, single_filed, j) << "\n";
      }
    }
    else
    {
      result = QVariant::fromValue(reflection->GetDouble(message, single_filed));
    }
  }
  break;
  case FieldDescriptor::TYPE_STRING:
  case FieldDescriptor::TYPE_BYTES:
  {
    if (repeated_counts)
    {
      for (int j = 0; j < repeated_counts; ++j)
      {
        std::cout << " string repeated " << reflection->GetRepeatedString(message, single_filed, j) << "\n";
      }
    }
    else
    {
      result = QVariant::fromValue(QString::fromStdString(reflection->GetString(message, single_filed)));
    }
  }
  break;

  case FieldDescriptor::TYPE_ENUM:
  {
    if (repeated_counts)
    {
      for (int j = 0; j < repeated_counts; ++j)
      {
        auto *single_enum = reflection->GetRepeatedEnum(message, single_filed, j);
        std::cout << " string repeated enum " << single_enum->full_name() << "\n";
      }
    }
    else
    {
      auto *single_enum = reflection->GetEnum(message, single_filed);
      result = QVariant::fromValue(QString::fromStdString(single_enum->full_name()));
    }
  }
  break;
  case FieldDescriptor::TYPE_MESSAGE:
  {
  }
  break;
  default:
    break;
  }
  return result;
}

QVariant GetValueByHeaderName(const google::protobuf::Message &msg, QString const &name)
{
  QVariant result;
  QStringList parts = name.split("-");

  google::protobuf::Message const *message = &msg;

  for (int i = 0; i < parts.size(); ++i)
  {
    auto *descriptor = message->GetDescriptor();
    auto *reflection = message->GetReflection();
    auto *single_filed = descriptor->FindFieldByName(parts[i].toStdString());

    if (!single_filed)
    {
      return QVariant();
    }
    // 如果已经到底
    if (i == parts.size() - 1)
    {
      result = ReflectVaule(*message, descriptor, reflection, single_filed);
    }
    else
    {
      message = &reflection->GetMessage(*message, single_filed);
    }
  }
  return result;
}

void PrintProtoMsg2(const google::protobuf::Message &message)
{
  using namespace google::protobuf;
  auto *descriptor = message.GetDescriptor();
  auto *reflection = message.GetReflection();
  // 遍历所有字段
  int fields = descriptor->field_count();
  for (int i = 0; i < fields; ++i)
  {
    auto *single_filed = descriptor->field(i);
    // 根据类型筛选

    int repeated_counts = single_filed->is_repeated() ? reflection->FieldSize(message, single_filed) : 0;

    switch (single_filed->type())
    {
    case FieldDescriptor::TYPE_INT32:
    case FieldDescriptor::TYPE_SINT32:
    case FieldDescriptor::TYPE_FIXED32:
    {
      if (repeated_counts)
      {
        for (int j = 0; j < repeated_counts; ++j)
        {
          std::cout << " int32 repeated " << reflection->GetRepeatedInt32(message, single_filed, j) << "\n";
        }
      }
      else
      {
        std::cout << " int32 " << reflection->GetInt32(message, single_filed) << "\n";
      }
    }
    break;
    case FieldDescriptor::TYPE_STRING:
    case FieldDescriptor::TYPE_BYTES:
    {
      if (repeated_counts)
      {
        for (int j = 0; j < repeated_counts; ++j)
        {
          std::cout << " string repeated " << reflection->GetRepeatedString(message, single_filed, j) << "\n";
        }
      }
      else
      {
        std::cout << " string " << reflection->GetString(message, single_filed) << "\n";
      }
    }
    break;

    case FieldDescriptor::TYPE_ENUM:
    {
      if (repeated_counts)
      {
        for (int j = 0; j < repeated_counts; ++j)
        {
          auto *single_enum = reflection->GetRepeatedEnum(message, single_filed, j);
          std::cout << " string repeated enum " << single_enum->full_name() << "\n";
        }
      }
      else
      {
        auto *single_enum = reflection->GetEnum(message, single_filed);
        std::cout << " string repeated enum " << single_enum->full_name() << "\n";
      }
    }
    break;
    case FieldDescriptor::TYPE_MESSAGE:
    {
      if (repeated_counts)
      {
        for (int j = 0; j < repeated_counts; ++j)
        {
          auto const &msg = reflection->GetRepeatedMessage(message, single_filed, j);
          std::cout << " repeated msg " << j << "\n";
          PrintProtoMsg2(msg);
        }
      }
      else
      {
        auto const &msg = reflection->GetMessage(message, single_filed);
        std::cout << " msg "
                  << "\n";
        PrintProtoMsg2(msg);
      }
    }
    break;
    default:
      break;
    }
  }
}