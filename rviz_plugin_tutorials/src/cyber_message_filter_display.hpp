
// #pragma once
// #include <tf2_ros/message_filter.h>

// #include <memory>

// #include <rviz_common/properties/int_property.hpp>
// #include <rviz_common/ros_topic_display.hpp>

// #include "cyber_channel_display.hpp"

// /// Display subclass using a rclcpp::subscription and tf2_ros::MessageFilter.
// /**
//  * This class handles subscribing and unsubscribing to a ROS node using the
//  * message filter when the display is enabled or disabled.
//  * This class is templated on the ROS message type.
//  */
// template <class InnerProtoClass> class CyberMessageFilterDisplay : public _CyberTopicDisplay {
//   // No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.

// public:
//   /// Convenience typedef so subclasses don't have to use
//   /// the long templated class name to refer to their super class.
//   typedef CyberMessageFilterDisplay<InnerProtoClass> MFDClass;

//   CyberMessageFilterDisplay() : tf_filter_(nullptr), messages_received_(0) {
//     QString message_type = QString::fromStdString(InnerProtoClass::descriptor()->full_name());
//     topic_property_->setMessageType(message_type);
//     topic_property_->setDescription(message_type + " topic to subscribe to.");

//     message_queue_property_ = new properties::IntProperty("Filter size",
//                                                           10,
//                                                           "Set the filter size of the Message Filter Display.",
//                                                           topic_property_,
//                                                           SLOT(updateMessageQueueSize()),
//                                                           this,
//                                                           1,
//                                                           INT_MAX);
//   }

//   /**
//    * When overriding this method, the onInitialize() method of this superclass has to be called.
//    * Otherwise, the ros node will not be initialized.
//    */
//   void onInitialize() override { _CyberTopicDisplay::onInitialize(); }

//   ~CyberMessageFilterDisplay() override { unsubscribe(); }

//   void reset() override {
//     Display::reset();
//     if (tf_filter_) { tf_filter_->clear(); }
//     messages_received_ = 0;
//   }

//   void setTopic(const QString &topic, const QString &datatype) override {
//     (void)datatype;
//     topic_property_->setString(topic);
//   }

// protected:
//   void updateTopic() override { resetSubscription(); }

//   virtual void subscribe() {
//     if (inited_) { return; }

//     if (topic_property_->isEmpty()) {
//       setStatus(properties::StatusProperty::Error, "Topic", QString("Error subscribing: Empty topic name"));
//       return;
//     }

//     try {
//       rclcpp::Node::SharedPtr node = rviz_ros_node_.lock()->get_raw_node();

//       //
//       sp_pub_ = node->create_publisher<rviz_plugin_tutorials::Wrapper>(topic_property_->getStdString(), 10);
//       // cyber收到数据后立即发送出去
//       apollo::cyber::ReaderConfig cfg;
//       cfg.channel_name = topic_property_->getStdString();
//       cfg.pending_queue_size = 200;
//       SingletonProxy::get_mutable_instance().node_->CreateReader<InnerProtoClass>(cfg, [this](auto msg) {
//         rviz_plugin_tutorials::Wrapper ros_msg_wrapper;
//         ros_msg_wrapper.header.frame_id = msg_channel_name_base_;
//         ros_msg_wrapper.header.stamp = ros::Time{0};
//         this->sp_pub_->publish(ros_msg_wrapper);
//       });
//       //
//       subscription_ = std::make_shared<message_filters::Subscriber<rviz_plugin_tutorials::Wrapper>>(
//           node, topic_property_->getTopicStd(), qos_profile.get_rmw_qos_profile());
//       subscription_start_time_ = node->now();

//       tf_filter_ = std::make_shared<tf2_ros::MessageFilter<rviz_plugin_tutorials::Wrapper, transformation::FrameTransformer>>(
//           *context_->getFrameManager()->getTransformer(),
//           fixed_frame_.toStdString(),
//           static_cast<uint32_t>(message_queue_property_->getInt()),
//           node);
//       tf_filter_->connectInput(*subscription_);
//       tf_filter_->registerCallback(
//           std::bind(&CyberMessageFilterDisplay<InnerProtoClass>::messageTaken, this, std::placeholders::_1));
//       setStatus(properties::StatusProperty::Ok, "Topic", "OK");

//     } catch (rclcpp::exceptions::InvalidTopicNameError &e) {
//       setStatus(properties::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
//     }
//   }

//   void updateMessageQueueSize() {
//     if (tf_filter_) { tf_filter_->setQueueSize(static_cast<uint32_t>(message_queue_property_->getInt())); }
//   }

//   void transformerChangedCallback() override { resetSubscription(); }

//   void resetSubscription() {
//     unsubscribe();
//     reset();
//     subscribe();
//     context_->queueRender();
//   }

//   virtual void unsubscribe() {
//     tf_filter_.reset();
//     subscription_.reset();
//   }

//   void onEnable() override { subscribe(); }

//   void onDisable() override {
//     unsubscribe();
//     reset();
//   }

//   void fixedFrameChanged() override {
//     if (tf_filter_) { tf_filter_->setTargetFrame(fixed_frame_.toStdString()); }
//     reset();
//   }

//   void messageTaken(typename rviz_plugin_tutorials::Wrapper::ConstSharedPtr msg) {
//     if (!msg) { return; }

//     // Do not process message right away, tf2_ros::MessageFilter may be
//     // calling back from tf2_ros::TransformListener dedicated thread.
//     // Use type erased signal/slot machinery to ensure messages are
//     // processed in the main thread.
//     Q_EMIT typeErasedMessageTaken(std::static_pointer_cast<const void>(msg));
//   }

//   void processTypeErasedMessage(std::shared_ptr<const void> type_erased_msg) override {
//     auto msg = std::static_pointer_cast<const rviz_plugin_tutorials::Wrapper>(type_erased_msg);

//     ++messages_received_;
//     QString topic_str = QString::number(messages_received_) + " messages received";
//     // Append topic subscription frequency if we can lock rviz_ros_node_.
//     std::shared_ptr<ros_integration::RosNodeAbstractionIface> node_interface = rviz_ros_node_.lock();
//     if (node_interface != nullptr) {
//       const double duration = (node_interface->get_raw_node()->now() - subscription_start_time_).seconds();
//       const double subscription_frequency = static_cast<double>(messages_received_) / duration;
//       topic_str += " at " + QString::number(subscription_frequency, 'f', 1) + " hz.";
//     }
//     setStatus(properties::StatusProperty::Ok, "Topic", topic_str);

//     // processMessage(msg);
//   }

//   /// Implement this to process the contents of a message.
//   /**
//    * This is called by incomingMessage().
//    */
//   virtual void processMessage(std::shared_ptr<InnerProtoClass const> msg) = 0;

//   typename std::shared_ptr<message_filters::Subscriber<rviz_plugin_tutorials::Wrapper>> subscription_;
//   rclcpp::Time subscription_start_time_;
//   std::shared_ptr<tf2_ros::MessageFilter<rviz_plugin_tutorials::Wrapper, transformation::FrameTransformer>> tf_filter_;
//   uint32_t messages_received_;
//   rviz_common::properties::IntProperty *message_queue_property_;
//   bool inited_ = false;
//   rclcpp::PublisherBase::SharedPtr sp_pub_;
// };
