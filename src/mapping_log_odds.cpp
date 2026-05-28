#include "phi_p3dx_mapping/mapping_node.hpp"
#include <cmath>
#include <algorithm>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tuple>
#include <set> // Adicionado para evitar duplicatas e conflitos de feixes

class MappingExample : public MappingNode
{
public:
  MappingExample() : MappingNode("mapping_example_cpp") {
    // IMPORTANTE: Certifique-se de que map_msg_.info.width e height já estão preenchidos aqui
    logOddsMap_.assign(map_msg_.info.width * map_msg_.info.height, l_0_);
  }

protected:
  void on_odom() override
  {
    publish_map();
  }

private:
  float l_0_ = 0.0;      
  float l_occ_ = 1.0986;   // p_occ = 0.75
  float l_free_ = -1.0986; // p_free = 0.25

  float L_MIN = -2.5;
  float L_MAX = 2.5;

  std::vector<float> laser_ranges_;
  std::vector<float> logOddsMap_;
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "laser_scan", 10,
    std::bind(&MappingExample::laser_callback, this, std::placeholders::_1));
    
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    laser_ranges_.clear();
    int width_cells = map_msg_.info.width;
    int height_cells = map_msg_.info.height;

    double current_x = x_;
    double current_y = y_;
    double current_theta = theta_;

    // Se o mapa do ROS ainda não foi configurado pela classe base, ignora o scan
    if (width_cells == 0 || height_cells == 0) {
        return;
    }

    // Garante que o vetor de Log-Odds tenha exatamente o tamanho do mapa no começo da execução
    if (logOddsMap_.size() != static_cast<size_t>(width_cells * height_cells)) {
        logOddsMap_.assign(width_cells * height_cells, l_0_);
    }

    for (float r : msg->ranges) {
      if (msg->range_min <= r && r <= msg->range_max) {
        laser_ranges_.push_back(r);
      } else {
        laser_ranges_.push_back(std::numeric_limits<float>::infinity());
      }
    }

    std::set<int> free_cells_set;
    std::vector<int> occupied;

    auto [gx_, gy_] = meters_to_cells(current_x, current_y);

    // Se o próprio robô estiver fora do mapa por erro de odom, ignora o scan
    if (gx_ < 0 || gx_ >= width_cells || gy_ < 0 || gy_ >= height_cells) {
        return;
    }

    for (size_t i = 0; i < laser_ranges_.size(); ++i) {
      float r = laser_ranges_[i];
      //pega angulo exato do feixe no mapa grid
      double angle = msg->angle_min + (i * msg->angle_increment) + current_theta;

      double end_x, end_y;
      std::tuple<int, int> end_g;

      if (std::isfinite(r)) {
        //se o feixe bateu em algo
          end_x = current_x + r * std::cos(angle);
          end_y = current_y + r * std::sin(angle);

          end_g = meters_to_cells(end_x, end_y);
          int egx = std::get<0>(end_g);
          int egy = std::get<1>(end_g);

          // Verifica se o obstáculo está dentro do mapa
          if (egx >= 0 && egx < width_cells && egy >= 0 && egy < height_cells) {
              occupied.push_back(egy * width_cells + egx);
              // Traça o raio 
              rayTrace(gx_, gy_, egx, egy, free_cells_set);
          }
      } 
      else {
          // Se o feixe for infinito
          end_x = current_x + r * std::cos(angle);
          end_y = current_y + r * std::sin(angle);
          
          end_g = meters_to_cells(end_x, end_y);
          int egx = std::get<0>(end_g);
          int egy = std::get<1>(end_g);

          // Traça o raio 
          if (egx >= 0 && egx < width_cells && egy >= 0 && egy < height_cells) {
            rayTrace(gx_, gy_, egx, egy, free_cells_set);
          }
      }
    }

  
    // Remove do conjunto de células LIVRES qualquer índice que tenha sido detectado como OCUPADO
    for (int occ_idx : occupied) {
        free_cells_set.erase(occ_idx);
    }

    // células livres
    for (int idx : free_cells_set) {
        logOddsMap_[idx] += l_free_ - l_0_;
        if (logOddsMap_[idx] < L_MIN) logOddsMap_[idx] = L_MIN;
    }

    //células ocupadas
    for (int idx : occupied) {
        logOddsMap_[idx] += l_occ_ - l_0_;
        if (logOddsMap_[idx] > L_MAX) logOddsMap_[idx] = L_MAX;
    }

    // grid do ROS
    for (size_t i = 0; i < logOddsMap_.size(); ++i) {
        float l = logOddsMap_[i];
        
        if (l == 0.0) {
            map_msg_.data[i] = -1; 
        } 
        else {
            float p = log2Prob(l);
            int ros_value = p*100;
            
            if (ros_value < 0)   ros_value = 0;
            if (ros_value > 100) ros_value = 100;

            if(l >= (L_MAX - 0.1)) ros_value == 100;
            
            map_msg_.data[i] = ros_value;
        }
    }
  }

  float log2Prob(float l) {
    return 1.0f / (1.0f + std::exp(-l));
  }

  // rasteriza todas as células atravessadas pelo raio
  void rayTrace(int x0, int y0, int x1, int y1, std::set<int>& free_cells) {
    int width_cells = map_msg_.info.width;
    int height_cells = map_msg_.info.height;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;

    while (true) {
        if (x == x1 && y == y1) break;

        //insere apenas se dentro do grid
        if (x >= 0 && x < width_cells && y >= 0 && y < height_cells) {
            free_cells.insert(y * width_cells + x);
        } else {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingExample>());
  rclcpp::shutdown();
  return 0;
}