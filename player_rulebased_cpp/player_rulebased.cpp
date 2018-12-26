// Author(s):         Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

class my_ai
  : public aiwc::ai_base
{
  static constexpr double PI = 3.1415926535;

public:
  my_ai(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath)
    : aiwc::ai_base(std::move(server_ip), port, std::move(realm), std::move(key), std::move(datapath))
    , robot_wheels{}
  {
    std::cout << "I am ready." << std::endl;
  }

private:
  void init()
  {
  }

  double d2r(double deg) {
    return deg * PI / 180;
  }

  double r2d(double rad) {
    return rad * 180 / PI;
  }

  double dist(double x1, double y1, double x2, double y2)
  {
    const auto dx = x1 - x2;
    const auto dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
  }

  void velocity(std::size_t id, double l, double r)
  {
    if (std::abs(l) > info.max_linear_velocity[id] || std::abs(r) > info.max_linear_velocity[id]) {
      double multiplier;
      if (std::abs(l) > std::abs(r))
        multiplier = info.max_linear_velocity[id] / std::abs(l);
      else
        multiplier = info.max_linear_velocity[id] / std::abs(r);

      l *= multiplier;
      r *= multiplier;
    }

    robot_wheels[id] = {l, r};
  }

  void position(std::size_t id, double x, double y, double damping = 0.35)
  {
    const double mult_lin = 5.0;
    const double mult_ang = 0.4;

    const double dx = x - cur_posture[id][X];
    const double dy = y - cur_posture[id][Y];

    const double d_e = std::sqrt(dx * dx + dy * dy);

    const double desired_th = (dx == 0 && dy == 0) ? (PI / 2) : std::atan2(dy, dx);

    double d_th = desired_th - cur_posture[id][TH];
    while(d_th > PI) d_th -= 2 * PI;
    while(d_th < -PI) d_th += 2 * PI;

    double ka;
    if(d_e > 1) {        ka = 17; }
    else if(d_e > 0.5) { ka = 19; }
    else if(d_e > 0.3) { ka = 21; }
    else if(d_e > 0.2) { ka = 23; }
    else               { ka = 25; }
    ka /= 90;

    int sign = 1;

    if(d_th > d2r(95)) {
      d_th -= PI;
      sign = -1;
    }
    else if(d_th < d2r(-95)) {
      d_th += PI;
      sign = -1;
    }

    if(std::abs(d_th) > d2r(85)) {
      velocity(id, -mult_ang * d_th, mult_ang * d_th);
    }
    else {
      if(d_e < 5.0 && abs(d_th) < d2r(40)) {
	      ka = 0.1;
      }
      ka *= 4;
      velocity(id,
	       sign * (mult_lin * (1 / (1 + std::exp(-3 * d_e)) - damping) - mult_ang * ka * d_th),
	       sign * (mult_lin * (1 / (1 + std::exp(-3 * d_e)) - damping) + mult_ang * ka * d_th));
    }
  }

  void kick(std::size_t id, double x, double y)
  {
    position(id, x, y, 0);
  }

  void goalie(std::size_t id)
  {
	const double x = -info.field[X] / 2 + info.robot_size[id] / sqrt(2) + 0.05;
    const double y = std::max(std::min(cur_ball[Y],
				       info.goal[Y] / 2 - info.robot_size[id] / sqrt(2)),
			      -info.goal[Y] / 2 + info.robot_size[id] / sqrt(2));
    //std::cout << "Target Pos: " << x << "," << y << std::endl;
    position(id, x, y);
  }

  void defend(std::size_t id, std::size_t idx, double offset_y)
  {
    const double ox = 0.1;
    const double oy = 0.075;

    const double min_x = -info.field[X]/2 + info.robot_size[id] / sqrt(2) + 0.05;

    // If ball is on offense
    if(cur_ball[X] > 0) {
	    // If ball is in the upper part of the field (y>0)
	    if(cur_ball[Y] > 0){
        position(id, (cur_ball[X]-info.field[X]/2)/2, (std::min(cur_ball[Y],info.field[Y]/3))+offset_y);
	    }
	    // If ball is in the lower part of the field (y<0)
      else {
	      position(id, (cur_ball[X]-info.field[X]/2)/2, (std::max(cur_ball[Y],-info.field[Y]/3))+offset_y);
      }
    }
    else {
      // If robot is in front of the ball
	    if(cur_posture[id][X] > cur_ball[X] - ox) {
		    // if this defender is the nearest defender from the ball
		    if (id == idx) {
			    position(id,
					  (cur_ball[X] - ox),
					  (cur_posture[id][Y] < 0)?(cur_ball[Y] + oy):(cur_ball[Y] - oy));
		    }
	      else {
          position(id,
				    std::max((cur_ball[X]-0.03),min_x),
				    (cur_posture[id][Y] < 0)?(cur_posture[id][Y]+0.03):(cur_posture[id][Y]-0.03));
		    }
      }
	    // If robot is behind the ball
      else {
        if (id==idx) {
          position(id,
				    cur_ball[X],
				    (cur_posture[id][Y] < 0)?(cur_ball[Y]):(cur_ball[Y]));
		    }
	      else {
		      position(id,
				    std::max((cur_ball[X]-0.03),min_x),
				    (cur_posture[id][Y] < 0)?(cur_posture[id][Y]+0.03):(cur_posture[id][Y]-0.03));
		    }
      }
    }
  }

  void midfielder(std::size_t id, std::size_t idx, double offset_y)
  {
    const double ox = 0.1;
    const double oy = 0.075;
	  const double threshold = 0.7;

    const double ball_dist = dist(cur_posture[id][X], cur_posture[id][Y], cur_ball[X], cur_ball[Y]);
    const double goal_dist = dist(cur_posture[id][X], cur_posture[id][Y], info.field[X] / 2, 0);

    if (id==idx) {
      if(ball_dist < 0.04) {
		    // if near the ball and near the opposite team goal
		    if(goal_dist < 1.0) {
          position(id, info.field[X] / 2, 0);
		    }
		    else {
		      // if near the ball bur in front of the ball
		      if(cur_ball[X] < cur_posture[id][X] - 0.075) {
		        double x_suggest = std::max(cur_ball[X] - 0.075, -info.field[X] / 6);
			      position(id, x_suggest, cur_ball[Y]);
			    }
		      // if near the ball and behind the ball
		      else {
		        position(id, info.field[X] + info.goal[X], -info.goal[Y] / 2);
	        }
	      }
	    }
      else {
        if (cur_ball[X] < cur_posture[id][X]) {
		      if (cur_ball[Y] > 0)
		        position(id, cur_ball[X] - ox, std::min((cur_ball[Y] - oy), 0.45*info.field[Y]));
		      else
		        position(id, cur_ball[X] - ox, std::max((cur_ball[Y] + oy), -0.45*info.field[Y]));
		    }
		    else {
		      position(id, cur_ball[X], cur_ball[Y]);
	      }
	    }
	  }
	  else {
		  position(id, std::max(cur_ball[X]-0.1,-0.3*info.field[Y]), cur_ball[Y]+offset_y);
	  }
  }

  int find_closest_robot(void)
  {
    //Check for distances between each non-goallie robot and the ball, return the index of robot closest to the ball.
    int min_idx = 0;
    double min_dist = 9999.99;
    for (int i = 0; i < info.number_of_robots - 1; i++) {
      auto measured_dist = dist(cur_ball[X], cur_ball[Y], cur_posture[i][X], cur_posture[i][Y]); //calculate the distance
      if (measured_dist < min_dist) { //if this robot's distance is shorter, replace. (robot with smaller idx has priority over one with larger idx when two dists are equal)
        min_dist = measured_dist;
        min_idx = i;
      }
    }
    return min_idx;
  }

  auto get_coord(const aiwc::frame& f)
  {
    decltype(cur_posture) cur;
	std::array<double, 2> pos_ball = { (*f.opt_coordinates).ball.x, (*f.opt_coordinates).ball.y };

    for(std::size_t id = 0; id < 5; ++id) {
      cur[id][X]  = (*f.opt_coordinates).robots[MYTEAM][id].x;
      cur[id][Y]  = (*f.opt_coordinates).robots[MYTEAM][id].y;
      cur[id][TH] = (*f.opt_coordinates).robots[MYTEAM][id].th;
    }

    return std::make_pair(cur, pos_ball);
  }

  void update(const aiwc::frame& f)
  {
	  frames.push_back(f);
    if(f.reset_reason == aiwc::GAME_START) {
      previous_frame = f;
      std::tie(cur_posture, cur_ball) = get_coord(f);
      return;
    }
    else if(f.reset_reason == aiwc::HALFTIME) {
      // halftime is met - from next frame, received_frame.half_passed will be set to True
      // although the simulation switches sides,
      // coordinates and images given to your AI soccer algorithm will stay the same
      // that your team is red and located on left side whether it is 1st half or 2nd half

      // this example does not do anything at halftime
    }
    else if(f.reset_reason == aiwc::GAME_END) {
      return;
    }

    // update current robot and ball postures
    std::tie(cur_posture, cur_ball) = get_coord(f);

    // array for holding wheel speeds
    std::array<double, 10> ws;

    // act differently based on the current game state
    switch(f.game_state) {
      case aiwc::STATE_DEFAULT:
        {
          int idx = find_closest_robot();

          // Robots Functions
          goalie(0);
          defend(1, idx, 0.2);
          defend(2, idx, -0.2);
          midfielder(3, idx, 0.15);
          midfielder(4, idx, -0.15);
        }
        break;
      case aiwc::STATE_BACKPASS:
        {
          // If the ball belongs to my team, initiate backpass
          if (f.ball_ownership)
            position(4, 0, 0);
        }
        break;
      default:
        break;
    }

    // hand over current frame as previous frame to next step
    prev_ball = cur_ball;
    previous_frame = f;

    for(std::size_t id = 0; id < 5; ++id) {
      // std::cout << "Robot " << id << ":[" << robot_wheels[id][0] << "," << robot_wheels[id][1] << "]" << std::endl; //print robots info
      ws[2*id    ] = robot_wheels[id][0]; // left
      ws[2*id + 1] = robot_wheels[id][1]; // right
    }

    // send wheel speed data to the simulator
    set_wheel(ws); // this function is defined at ai_base.cpp
  }

  void finish()
  {
    // You have less than 30 seconds before it's killed.
    std::ofstream ofs(datapath + "/result.txt");

    // print header
    ofs << "time, a.score, b.score, ";
    ofs << "ball.x, ball.y" << std::endl;

    // print data
    for(const auto& f : frames) {
      std::array<double, 2> ball;

	    ball = { (*f.opt_coordinates).ball.x, (*f.opt_coordinates).ball.y };

      ofs << f.time << ", " << f.score[0] << ", " << f.score[1] << ", ";

      ofs << ball[X] << ", " << ball[Y] << std::endl;
    }
  }

private: // member variable
  aiwc::frame previous_frame;

  std::array<std::array<double, 3>, 5> cur_posture;
  std::array<double, 2> prev_ball;
  std::array<double, 2> cur_ball;

  std::array<std::array<double, 2>, 5> robot_wheels;

  std::vector<aiwc::frame> frames;
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
