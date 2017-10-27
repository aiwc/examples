#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>

#include <fstream>
#include <iostream>
#include <random>

class my_ai
  : public aiwc::ai_base
{
public:
  my_ai(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath)
    : aiwc::ai_base(std::move(server_ip), port, std::move(realm), std::move(key), std::move(datapath))
  {
    // you don't have any information of the game here
  }

private:
  void init()
  {
    // now you have information of the game

    // double field_x = info.field[0];
    // double field_y = info.field[1];
    // double resolution_x = info.resolution[0];
    // double resolution_y = info.resolution[1];
  }

  void update(const aiwc::frame& f)
  {
    if(f.reset_reason == aiwc::GAME_START) {
      std::cout << "Game started : " << f.time << std::endl;
    }
    if(f.reset_reason == aiwc::SCORE_MYTEAM) {
      // yay! my team scored!
      std::cout << "Myteam scored : " << f.time << std::endl;
    }
    else if(f.reset_reason == aiwc::SCORE_OPPONENT) {
      // T_T what have you done
      std::cout << "Opponent scored : " << f.time << std::endl;
    }
    else if(f.reset_reason == aiwc::GAME_END) {
      // game is finished. finish() will be called after you return.
      // now you have about 30 seconds before this process is killed.
      std::cout << "Game ended : " << f.time << std::endl;
      return;
    }

    if(f.opt_coordinates) { // if the optional coordinates are given,
      // const auto& myteam   = std::get<MYTEAM>  (*f.opt_coordinates);
      // const auto& opponent = std::get<OPPONENT>(*f.opt_coordinates);
      // const auto& ball     = std::get<BALL>    (*f.opt_coordinates);

      // const auto& myteam0 = myteam[0]; // id=0 robot

      // const auto& myteam0_x      = std::get<X>(myteam0);
      // const auto& myteam0_y      = std::get<Y>(myteam0);
      // const auto& myteam0_th     = std::get<TH>(myteam0);
      // const auto& myteam0_active = std::get<ACTIVE>(myteam0); // if false, the robot is currently out of the field and deactivated
    }
    else { // given no coordinates, you need to get coordinates from image
    }

    std::array<double, 10> wheels;
    for(auto& w : wheels) {
      w = info.max_linear_velocity;
    }
    set_wheel(wheels); // every robot will go ahead with maximum velocity
  }

  void finish()
  {
    // You have less than 30 seconds before it's killed.
    std::fstream ofs(datapath + "/result.txt");
    ofs << "my_result" << std::endl;
  }

private: // member variable
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
