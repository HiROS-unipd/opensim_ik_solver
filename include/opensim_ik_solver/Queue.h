#ifndef hiros_opensim_ik_solver_Queue_h
#define hiros_opensim_ik_solver_Queue_h

#include <condition_variable>
#include <mutex>
//#include <string>

template <typename T_in, typename T_out>
class Queue
{
private:
  struct Node
  {
    std::shared_ptr<T_in> data_in;
    std::shared_ptr<T_out> data_out;
    std::shared_ptr<bool> processed;
    std::shared_ptr<Node> next;
    Node(std::shared_ptr<T_in> data_in_, std::shared_ptr<T_out> data_out_, std::shared_ptr<bool> processed_)
      : data_in(std::move(data_in_))
      , data_out(std::move(data_out_))
      , processed(std::move(processed_))
    {}
  };
  std::shared_ptr<Node> head;
  std::shared_ptr<Node> tail;
  std::shared_ptr<Node> next_to_consume;
  std::mutex headMutex;
  std::mutex tailMutex;
  std::mutex next_to_consumeMutex;
  std::condition_variable new_element_present;
  std::condition_variable output_ready;

public:
  Queue()
    : head(std::make_shared<Node>(
      Node(std::make_shared<T_in>(T_in{}), std::make_shared<T_out>(T_out{}), std::make_shared<bool>(false))))
    , tail(head)
    , next_to_consume(head)
  {}

  Queue(const Queue& other) = delete;
  Queue& operator=(const Queue& other) = delete;

  std::shared_ptr<T_out> pop()
  {
    std::unique_lock<std::mutex> headLock(headMutex);

    while (!*(head->processed)) {
      output_ready.wait(headLock);
    }
    auto output = head->data_out;
    head = std::move(head->next);
    return output;
  }

  void notify_output_ready(std::shared_ptr<bool>& ptr_processed)
  {
    (*ptr_processed) = true;
    output_ready.notify_one();
  }

  void push(T_in data_in_)
  {
    auto dummyNode = std::make_shared<Node>(
      Node(std::make_shared<T_in>(T_in{}), std::make_shared<T_out>(T_out{}), std::make_shared<bool>(false)));
    auto newTail = dummyNode;

    auto data_in_ptr = std::make_shared<T_in>(std::move(data_in_));
    auto data_out_ptr = std::make_shared<T_out>(T_out{});

    {
      std::unique_lock<std::mutex> tailLock(tailMutex);
      tail->next = std::move(dummyNode);
      tail->data_in = std::move(data_in_ptr);
      tail->data_out = std::move(data_out_ptr);
      tail = newTail;
    }

    new_element_present.notify_one();
  }

  void take_next_to_consume(std::shared_ptr<T_in>& ptr_in,
                            std::shared_ptr<T_out>& ptr_out,
                            std::shared_ptr<bool>& ptr_processed)
  {
    std::unique_lock<std::mutex> next_to_consumeLock(next_to_consumeMutex);
    {
      std::unique_lock<std::mutex> tailLock(tailMutex);
      while (next_to_consume == tail) {
        new_element_present.wait(tailLock);
      }
    }

    ptr_in = next_to_consume->data_in;
    ptr_out = next_to_consume->data_out;
    ptr_processed = next_to_consume->processed;

    next_to_consume = next_to_consume->next;
  }

  /*std::string print_queue()
  {
      std::shared_ptr<Node> temp_ptr=head;
      std::string return_value{""};

      while (temp_ptr!=tail)
      {
          return_value+="\ndata_in  :";
          return_value.append(temp_ptr->data_in->toString());
          return_value+="\ndata_out :";
          return_value.append(temp_ptr->data_out->toString());
          return_value+="\nprocessed:";
          if(*(temp_ptr->processed))
          {
              return_value+="true";
          }
          else
          {
              return_value+="false";
          }
          return_value+="\n-----------------------------------------";
          temp_ptr=temp_ptr->next;
      }
      return return_value;
  }*/
};

#endif
