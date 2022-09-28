#ifndef hiros_opensim_ik_solver_Queue_h
#define hiros_opensim_ik_solver_Queue_h

#include <condition_variable>
#include <mutex>

namespace hiros {
namespace opensim_ik {

template <typename T_in, typename T_out>
class Queue {
 public:
  Queue()
      : head_(std::make_shared<Node>()),
        tail_(head_),
        next_to_consume_(head_) {}

  Queue(const Queue& other) = delete;
  Queue& operator=(const Queue& other) = delete;

  size_t size() {
    if (!head_) {
      return 0;
    }

    size_t size = 0;
    auto tmp_ptr = head_;

    while (tmp_ptr != tail_) {
      ++size;
      tmp_ptr = tmp_ptr->next;
    }

    return size;
  }

  void push(T_in data_in) {
    auto new_tail = std::make_shared<Node>();
    auto data_in_ptr = std::make_shared<T_in>(std::move(data_in));

    {
      std::unique_lock<std::mutex> tail_lock(tail_mutex_);
      tail_->next = new_tail;
      tail_->data_in = std::move(data_in_ptr);
      tail_ = std::move(new_tail);
    }

    new_element_present_.notify_one();
  }

  void takeNextToConsume(std::shared_ptr<T_in>& data_in_ptr,
                         std::shared_ptr<T_out>& data_out_ptr,
                         std::shared_ptr<bool>& processed_ptr) {
    std::unique_lock<std::mutex> next_to_consume_lock(next_to_consume_mutex_);

    {
      std::unique_lock<std::mutex> tail_lock(tail_mutex_);

      while (next_to_consume_ == tail_) {
        new_element_present_.wait(tail_lock);
      }
    }

    data_in_ptr = next_to_consume_->data_in;
    data_out_ptr = next_to_consume_->data_out;
    processed_ptr = next_to_consume_->processed;

    next_to_consume_ = next_to_consume_->next;
  }

  void notifyOutputReady(std::shared_ptr<bool> processed_ptr) {
    *processed_ptr = true;
    output_ready_.notify_one();
  }

  T_out pop() {
    std::unique_lock<std::mutex> head_lock(head_mutex_);

    while (!*(head_->processed)) {
      output_ready_.wait(head_lock);
    }

    auto data_out_ptr = std::move(head_->data_out);
    head_ = std::move(head_->next);

    return *data_out_ptr;
  }

 private:
  struct Node {
    std::shared_ptr<T_in> data_in;
    std::shared_ptr<T_out> data_out;
    std::shared_ptr<bool> processed;

    std::shared_ptr<Node> next;

    Node(std::shared_ptr<T_in> data_in = std::make_shared<T_in>(),
         std::shared_ptr<T_out> data_out = std::make_shared<T_out>(),
         std::shared_ptr<bool> processed = std::make_shared<bool>(false))
        : data_in(std::move(data_in)),
          data_out(std::move(data_out)),
          processed(std::move(processed)),
          next(nullptr) {}
  };

  std::shared_ptr<Node> head_{};
  std::shared_ptr<Node> tail_{};
  std::shared_ptr<Node> next_to_consume_{};

  std::mutex head_mutex_{};
  std::mutex tail_mutex_{};
  std::mutex next_to_consume_mutex_{};

  std::condition_variable new_element_present_{};
  std::condition_variable output_ready_{};
};

}  // namespace opensim_ik
}  // namespace hiros

#endif
