#pragma once

#include <deque>
#include <google/protobuf/message.h>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "cacher/utils.h"

// using namespace zhito::zloc;
// using namespace zhito::convert;

// const C; reference R;sp shared_ptr
#define cr(T) T const &
#define sp(T) std::shared_ptr<T>
#define up(T) std::unique_ptr<T>
#define crsp_c(T) std::shared_ptr<T const> const &
#define sp_c(T) std::shared_ptr<T const>
#define crsp(T) std::shared_ptr<T> const &

#define DEFINE_EXTEND_TYPE(T)                                                                                          \
  using cr##T = T const &;                                                                                             \
  using sp##T = std::shared_ptr<T>;                                                                                    \
  using up##T = std::unique_ptr<T>;                                                                                    \
  using crsp_c##T = std::shared_ptr<T const> const &;                                                                  \
  using sp_c##T = std::shared_ptr<T const>;                                                                            \
  using crsp##T = std::shared_ptr<T> const &

using sp_cPbMsg = std::shared_ptr<google::protobuf::Message const>;
using spPbMsg = std::shared_ptr<google::protobuf::Message>;

class Cacher {
public:
  Cacher() {}
  ~Cacher() {}

  void push_back(const std::string &channel_name, double time, sp_cPbMsg msg) {
    // 加锁
    {
      std::lock_guard<std::mutex> lock(mtx_named_buff_);
      size_t t0_ms = ::s2ms(time);
      auto &single_buff = named_buff_[channel_name];
      single_buff[t0_ms] = msg;
    }
  }

  void Reset() {
    {
      std::lock_guard<std::mutex> lock(mtx_named_buff_);
      named_buff_.clear();
    }
  }

  template <typename MT>
  std::shared_ptr<MT const> GetProtoMsgWithChannleTime(const std::string &channle_name, double time) {
    std::lock_guard<std::mutex> lock(mtx_named_buff_);
    const auto &single_buff = named_buff_[channle_name];
    auto it = single_buff.find(::s2ms(time));
    if (it != single_buff.end()) {
      auto frame = std::dynamic_pointer_cast<MT const>(it->second);
      return frame;
    }
    return nullptr;
  }

  // template <typename T>
  // sp_cZFrame GetFrameWithChannleTime(const std::string &channle_name, double time)
  // {
  //   std::lock_guard<std::mutex> lock(mtx_named_buff_);
  //   const auto &single_buff = named_buff_[channle_name];
  //   auto it = single_buff.find(::s2ms(time));
  //   if (it != single_buff.end())
  //   {
  //     auto frame = std::dynamic_pointer_cast<const T>(it->second);
  //     return zhito::convert::Convert(frame);
  //   }
  //   return nullptr;
  // }

  std::map<size_t, sp_cPbMsg> GetProtoWithChannleName(const std::string &channle_name, double time = 0) {
    size_t time_ms = ::s2ms(time);
    std::map<size_t, sp_cPbMsg> single_buff;
    auto it = named_buff_.find(channle_name);
    if (it == named_buff_.end()) { return single_buff; }
    {
      std::lock_guard<std::mutex> lock(mtx_named_buff_);
      single_buff =
          std::map<size_t, sp_cPbMsg>(named_buff_[channle_name].upper_bound(time_ms), named_buff_[channle_name].end());
      // single_buff = named_buff_[channle_name];
    }
    return single_buff;
  }

  std::list<std::string> GetChannelNames() {
    std::list<std::string> result;
    std::lock_guard<std::mutex> lock(mtx_named_buff_);
    for (auto const &kv : named_buff_) {
      result.push_back(kv.first);
    }
    return result;
  }

  std::string GetTypeNameWithChannelName(std::string const &name) {
    std::map<std::string, std::string> result;
    std::lock_guard<std::mutex> lock(mtx_named_buff_);
    return named_buff_[name].begin()->second->GetTypeName();
  }

private:
  std::unordered_map<std::string, std::map<size_t, sp_cPbMsg>> named_buff_;
  std::mutex mtx_named_buff_;
};
DEFINE_EXTEND_TYPE(Cacher);

using CacherBuffer = std::unordered_map<std::string, std::map<size_t, sp_cPbMsg>>;

extern spCacher g_cacher_;