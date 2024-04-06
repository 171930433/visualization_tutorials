#pragma once
#include "src/properties/cyber_channel_property.h"
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>

//
#include <cyber/cyber.h>

#include <boost/serialization/singleton.hpp>

class SingletonProxy : public boost::serialization::singleton<SingletonProxy> {
protected:
  SingletonProxy() {
    apollo::cyber::Init("rviz_cyber");
    node_ = apollo::cyber::CreateNode("rviz_cyber");
  }
  std::unique_ptr<apollo::cyber::Node> node_;
};

Q_DECLARE_METATYPE(std::shared_ptr<const void>)

/** @brief Helper superclass for CyberTopicDisplay, needed because
 * Qt's moc and c++ templates don't work nicely together.  Not
 * intended to be used directly. */
class _CyberTopicDisplay : public rviz_common::Display {
  Q_OBJECT

public:
  _CyberTopicDisplay() {
    qRegisterMetaType<std::shared_ptr<const void>>();

    topic_property_ = new CyberChannelProperty("Topic", "", "", this, SLOT(updateTopic()));
  }

  /**
   * When overriding this method, the onInitialize() method of this superclass has to be called.
   * Otherwise, the ros node will not be initialized.
   */
  void onInitialize() override {
    connect(reinterpret_cast<QObject *>(context_->getTransformationManager()),
            SIGNAL(transformerChanged(std::shared_ptr<rviz_common::transformation::FrameTransformer>)),
            this,
            SLOT(transformerChangedCallback()));

    // Useful to _CyberTopicDisplay subclasses to ensure GUI updates
    // are performed by the main thread only.
    connect(this,
            SIGNAL(typeErasedMessageTaken(std::shared_ptr<const void>)),
            this,
            SLOT(processTypeErasedMessage(std::shared_ptr<const void>)),
            // Force queued connections regardless of QObject thread affinity
            Qt::QueuedConnection);
  }

Q_SIGNALS:
  void typeErasedMessageTaken(std::shared_ptr<const void> type_erased_message);

protected Q_SLOTS:
  virtual void processTypeErasedMessage(std::shared_ptr<const void> type_erased_message) { (void)type_erased_message; }

  virtual void transformerChangedCallback() {}
  virtual void updateMessageQueueSize() {}
  virtual void updateTopic() = 0;

protected:
  CyberChannelProperty *topic_property_;
};


template <class InnerProtoClass> class CyberTopicDisplay : public _CyberTopicDisplay {
public:
  typedef CyberTopicDisplay<InnerProtoClass> RTDClass;

  CyberTopicDisplay() : messages_received_(0) {
    QString message_type = QString::fromStdString(InnerProtoClass::descriptor()->full_name());
    topic_property_->setMessageType(message_type);
    topic_property_->setDescription(message_type + " topic to subscribe to.");
  }

  ~CyberTopicDisplay() override { unsubscribe(); }

  void reset() override {
    Display::reset();
    messages_received_ = 0;
  }

  void setTopic(const QString &topic, const QString &datatype) override {
    (void)datatype;
    topic_property_->setString(topic);
  }

protected:
  void updateTopic() override {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
  }

  virtual void subscribe() {
    if (inited_) { return; }

    if (topic_property_->isEmpty()) {
      setStatus(properties::StatusProperty::Error, "Topic", QString("Error subscribing: Empty topic name"));
      return;
    }

    try {
      subscription_start_time_ = rclcpp::Clock::now();

      apollo::cyber::ReaderConfig cfg;
      cfg.channel_name = topic_property_->getStdString();
      cfg.pending_queue_size = 200;
      SingletonProxy::get_mutable_instance().node_->CreateReader<InnerProtoClass>(
          cfg, [this](auto msg) { this->incomingMessage(msg); });
      // 确保不会重复接收
      inited_ = true;
      setStatus(properties::StatusProperty::Ok, "Topic", "OK");
    } catch (rclcpp::exceptions::InvalidTopicNameError &e) {
      setStatus(properties::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
  }

  virtual void unsubscribe() {}

  void onEnable() override { subscribe(); }

  void onDisable() override {
    unsubscribe();
    reset();
  }

  void fixedFrameChanged() override { reset(); }

  /** @brief Incoming message callback.  Checks if the message pointer
   * is valid, increments messages_received_, then calls
   * processMessage(). */
  void incomingMessage(std::shared_ptr<InnerProtoClass const> msg) {
    if (!msg) { return; }

    ++messages_received_;
    QString topic_str = QString::number(messages_received_) + " messages received";

    if (1) {
      const double duration = (rclcpp::Clock::now() - subscription_start_time_).seconds();
      const double subscription_frequency = static_cast<double>(messages_received_) / duration;
      topic_str += " at " + QString::number(subscription_frequency, 'f', 1) + " hz.";
    }
    setStatus(properties::StatusProperty::Ok, "Topic", topic_str);

    processMessage(msg);
  }

  virtual void processMessage(std::shared_ptr<InnerProtoClass const> msg) = 0;

  rclcpp::Time subscription_start_time_;
  uint32_t messages_received_;
  bool inited_ = false;
};