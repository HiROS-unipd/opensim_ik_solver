#ifndef hiros_opensim_ik_solver_Queue_h
#define hiros_opensim_ik_solver_Queue_h

#include <condition_variable>
#include <mutex>

namespace hiros {
  namespace opensim_ik {

    template <typename T_in, typename T_out>
    class Queue
    {
    public:
      Queue()
        : m_head(std::make_shared<Node>())
        , m_tail(m_head)
        , m_next_to_consume(m_head)
      {}

      Queue(const Queue& t_other) = delete;
      Queue& operator=(const Queue& t_other) = delete;

      size_t size()
      {
        if (!m_head) {
          return 0;
        }

        size_t size = 0;
        auto tmp_ptr = m_head;

        while (tmp_ptr != m_tail) {
          ++size;
          tmp_ptr = tmp_ptr->next;
        }

        return size;
      }

      void push(T_in t_data_in)
      {
        auto new_tail = std::make_shared<Node>();
        auto data_in_ptr = std::make_shared<T_in>(std::move(t_data_in));

        {
          std::unique_lock<std::mutex> tail_lock(m_tail_mutex);
          m_tail->next = new_tail;
          m_tail->data_in = std::move(data_in_ptr);
          m_tail = std::move(new_tail);
        }

        m_new_element_present.notify_one();
      }

      void takeNextToConsume(std::shared_ptr<T_in>& t_data_in_ptr,
                             std::shared_ptr<T_out>& t_data_out_ptr,
                             std::shared_ptr<bool>& t_processed_ptr)
      {
        std::unique_lock<std::mutex> next_to_consume_lock(m_next_to_consume_mutex);

        {
          std::unique_lock<std::mutex> tail_lock(m_tail_mutex);

          while (m_next_to_consume == m_tail) {
            m_new_element_present.wait(tail_lock);
          }
        }

        t_data_in_ptr = m_next_to_consume->data_in;
        t_data_out_ptr = m_next_to_consume->data_out;
        t_processed_ptr = m_next_to_consume->processed;

        m_next_to_consume = m_next_to_consume->next;
      }

      void notifyOutputReady(std::shared_ptr<bool> t_processed_ptr)
      {
        *t_processed_ptr = true;
        m_output_ready.notify_one();
      }

      T_out pop()
      {
        std::unique_lock<std::mutex> head_lock(m_head_mutex);

        while (!*(m_head->processed)) {
          m_output_ready.wait(head_lock);
        }

        auto data_out_ptr = std::move(m_head->data_out);
        m_head = std::move(m_head->next);

        return *data_out_ptr;
      }

    private:
      struct Node
      {
        std::shared_ptr<T_in> data_in;
        std::shared_ptr<T_out> data_out;
        std::shared_ptr<bool> processed;

        std::shared_ptr<Node> next;

        Node(std::shared_ptr<T_in> t_data_in = std::make_shared<T_in>(),
             std::shared_ptr<T_out> t_data_out = std::make_shared<T_out>(),
             std::shared_ptr<bool> t_processed = std::make_shared<bool>(false))
          : data_in(std::move(t_data_in))
          , data_out(std::move(t_data_out))
          , processed(std::move(t_processed))
          , next(nullptr)
        {}
      };

      std::shared_ptr<Node> m_head;
      std::shared_ptr<Node> m_tail;
      std::shared_ptr<Node> m_next_to_consume;

      std::mutex m_head_mutex;
      std::mutex m_tail_mutex;
      std::mutex m_next_to_consume_mutex;

      std::condition_variable m_new_element_present;
      std::condition_variable m_output_ready;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
