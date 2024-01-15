#include "protobuf_helper.h"
#include <iostream>
#include <qmath.h>
std::map<size_t, spMessage> g_messages;

void InitPersons()
{
  if (g_messages.size() != 0)
  {
    return;
  }

  int n = 1000 * 1000; // number of points in graph
  double xScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double yScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double xOffset = (std::rand() / (double)RAND_MAX - 0.5) * 4;
  double yOffset = (std::rand() / (double)RAND_MAX - 0.5) * 10;
  double r1 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r2 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r3 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r4 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  for (int i = 0; i < n; i++)
  {
    size_t time_index = (i + 1e8) * 1e3;
    spPerson person = std::make_shared<demo::Person>();
    auto header = person->mutable_header();
    header->set_t0(i + 1e8);
    header->set_index(i);
    auto pos = person->mutable_pos();
    pos->set_x((i / (double)n - 0.5) * 10.0 * xScale + xOffset);
    pos->set_y((qSin(pos->x() * r1 * 5) * qSin(qCos(pos->x() * r2) * r4 * 3) + r3 * qCos(qSin(pos->x()) * r4 * 2)) * yScale + yOffset);
    g_messages[time_index] = person;
  }
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
    if (single_filed->type() == FieldDescriptor::TYPE_MESSAGE)
    {
      auto const &msg = reflection->GetMessage(message, single_filed);
      result << GetFildNames(msg, QString::fromStdString(single_filed->name()));
    }
    else
    {
      QString parent = (prefix == "" ? "" : prefix + "-");
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

  google::protobuf::Message const* message = &msg;

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