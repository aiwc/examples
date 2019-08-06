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
    // NOTICE:
    // you can send as many reports as you want, but ONLY THE LAST ONE will be preserved for the evaluation.

    if(f.reset_reason == aiwc::GAME_END) {
      // game is finished. finish() will be called after you return.
      // now you have about 30 seconds before this process is killed.

      std::vector<std::string> paragraphs;

      if(f.score[ATEAM] > f.score[BTEAM]) {
        paragraphs.emplace_back("Team Red won the game with score "
                            + std::to_string(f.score[ATEAM]) + ":" + std::to_string(f.score[BTEAM]));
      }
      else if(f.score[ATEAM] < f.score[BTEAM]) {
        paragraphs.emplace_back("Team Blue team won the game with score "
                            + std::to_string(f.score[ATEAM]) + ":" + std::to_string(f.score[BTEAM]));
      }
      else {
        paragraphs.emplace_back("Nhe game ended in a tie with score "
                            + std::to_string(f.score[ATEAM]) + ":" + std::to_string(f.score[BTEAM]));
      }

      paragraphs.emplace_back("It was really a great match!");

      // each element of report is a paragraph.
      report(paragraphs);

      return;
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
