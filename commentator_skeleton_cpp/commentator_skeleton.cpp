// Author(s):         Inbae Jeong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>

#include <fstream>
#include <iostream>

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
      if(!f.half_passed)
        commentate("Game has begun");
      else
        commentate("Second half has begun");
      return;
    }
    else if(f.reset_reason == aiwc::DEADLOCK) {
      commentate("Position is reset since no one touched the ball");
      return;
    }
    else if(f.reset_reason == aiwc::GOALKICK) {
      commentate("A goal kick of Team " + std::string(f.ball_ownership ? "Red" : "Blue"));
    }
    else if(f.reset_reason == aiwc::CORNERKICK) {
      commentate("A corner kick of Team " + std::string(f.ball_ownership ? "Red" : "Blue"));
    }
    else if(f.reset_reason == aiwc::PENALTYKICK) {
      commentate("A penalty kick of Team " + std::string(f.ball_ownership ? "Red" : "Blue"));
    }
    else if(f.reset_reason == aiwc::HALFTIME) {
      if(f.score[0] > f.score[1])
        commentate("The halftime has met. Team Red is leading the game with score " + std::to_string(f.score[0]) + " : " + std::to_string(f.score[1]));
      else if(f.score[0] < f.score[1])
        commentate("The halftime has met. Team Blue is leading the game with score " + std::to_string(f.score[1]) + " : " + std::to_string(f.score[0]));
      else
        commentate("The halftime has met. The game is currently a tie with score " + std::to_string(f.score[0]) + " : " + std::to_string(f.score[1]));
    }
    else if(f.reset_reason == aiwc::GAME_END) {
      if(f.score[0] > f.score[1])
        commentate("Team Red won the game with score " + std::to_string(f.score[0]) + " : " + std::to_string(f.score[1]));
      else if(f.score[0] < f.score[1])
        commentate("Team Blue won the game with score " + std::to_string(f.score[1]) + " : " + std::to_string(f.score[0]));
      else
        commentate("Game ended in a tie with score "  + std::to_string(f.score[0]) + " : " + std::to_string(f.score[1]));
      return;
    }

    if(f.opt_coordinates) {
      if((*f.opt_coordinates).ball.x >= info.field[X] / 2 && std::abs((*f.opt_coordinates).ball.y) <= info.goal[Y] / 2) {
        commentate("Team Red scored!!");
      }
      else if((*f.opt_coordinates).ball.x <= -info.field[X] / 2 && std::abs((*f.opt_coordinates).ball.y) <= info.goal[Y] / 2) {
        commentate("Team Blue scored!!");
      }

      // const auto& ateam0_x      = (*f.opt_coordinates).robots[ATEAM][0].x;
      // const auto& bteam0_active = (*f.opt_coordinates).robots[BTEAM][0].active;
    }
    else { // given no coordinates, you need to get coordinates from image
    }
  }

  void finish()
  {
    // You have less than 30 seconds before it's killed.
    std::ofstream ofs(datapath + "/result.txt");
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
