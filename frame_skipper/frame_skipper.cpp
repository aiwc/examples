#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>
#include <boost/random/random_device.hpp>

#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <random>
#include <thread>

class my_ai
  : public aiwc::ai_base
{
public:
  my_ai(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath)
    : aiwc::ai_base(std::move(server_ip), port, std::move(realm), std::move(key), std::move(datapath))
  {
  }

private:
  void init()
  {
    behavior_thread = std::thread([&]() { frame_skipper(); });
  }

  void update(const aiwc::frame& f)
  {
    std::unique_lock<std::mutex> lck(frames.m);
    frames.q.push_back(f);
    lck.unlock();
    frames.cv.notify_one();
  }

  void frame_skipper()
  {
    for(;;) {
      std::unique_lock<std::mutex> lck(frames.m);
      frames.cv.wait(lck, [&]() { return !frames.q.empty(); });

      std::vector<aiwc::frame> local_queue;
      local_queue.swap(frames.q);
      lck.unlock();

      // you can ignore all frames but the most recent one,
      // or keep only resetting frames,
      // or do whatever you want.

      // this example keeps only the most recent frame.
      choose_behavior_which_takes_really_long_time(local_queue.back());

      if(local_queue.back().reset_reason == aiwc::GAME_END) {
        break;
      }
    }
  }

  void choose_behavior_which_takes_really_long_time(const aiwc::frame& f)
  {
    if(f.reset_reason == aiwc::GAME_END) {
      return;
    }

    // heavy operations
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::mt19937 rng{boost::random_device{}()};
    std::uniform_real_distribution<double> dist(-info.max_linear_velocity, info.max_linear_velocity);

    std::array<double, 10> wheels;
    for(auto& s : wheels) {
      s = dist(rng);
    }
    set_wheel(wheels);
  }

  void finish()
  {
    // You have less than 30 seconds before it's killed.
    std::ofstream ofs(datapath + "/result.txt");
    ofs << "my_result" << std::endl;
  }

private: // member variable
  std::thread behavior_thread;

  struct {
    std::vector<aiwc::frame> q;
    std::mutex m;
    std::condition_variable cv;
  } frames;
};

int main(int argc, char *argv[])
{
  if(argc < 6) {
    std::cerr << "Usage " << argv[0] << " server_ip port realm key datapath" << std::endl;
    return -1;
  }

  const auto& server_ip = std::string(argv[1]);
  const auto& port      = boost::lexical_cast<std::size_t>(argv[2]);
  const auto& realm     = std::string(argv[3]);
  const auto& key       = std::string(argv[4]);
  const auto& datapath  = std::string(argv[5]);

  my_ai ai(server_ip, port, realm, key, datapath);

  ai.run();

  return 0;
}
