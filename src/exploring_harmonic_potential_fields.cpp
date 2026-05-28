#include "phi_p3dx_mapping/exploration_node.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <cstdlib>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

/**
 * @brief Exemplo simples de nodo de exploração.
 *
 * Realiza uma caminhada aleatória: anda para frente até encontrar 
 * um obstáculo usando os dados do laser de varredura. Ao detectar,
 * gira para evitar colisão e escolhe uma direção aleatória por tempo.
 * O mapa é recebido via callback apenas para demonstrar o consumo,
 * mas não é ativamente usado para decidir o caminho na lógica básica.
 */
class ExplorationExample : public ExplorationNode
{
public:
  ExplorationExample() : ExplorationNode("exploration") {
    // Semente para direção aleatória
    srand(time(NULL));
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    potential_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/potential_map", map_qos);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&ExplorationExample::odom_callback, this, std::placeholders::_1));
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr potential_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;   
  

  // campo potencial 
  std::vector<float> potential_map_;
  double x_, y_, theta_;
  double resolution_ = 0.05;

  const int N = 100; // Número de iterações
  double kp_angular = 1;
  double kp_linear = 1;
  double angle_alpha = 0.005;
  double GRADIENT_LIMIT = 0.001;
  int distant_look = 25;
  int width_ = 0;
  int height_ = 0;
  double last_arrow = 0;
  bool first_arrow = true;

  int toIndex(int x, int y){
    return y * width_ + x;
  }

protected:

  std::tuple<int, int> meters_to_cells(double wx, double wy) const
  {
    int mx = (int)((wx - map_msg_->info.origin.position.x) / resolution_);
    int my = (int)((wy - map_msg_->info.origin.position.y) / resolution_);
    return {mx, my};
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta_);


  }

  void control_loop() override
  {
    if (laser_ranges_.empty()) {
      return; // Aguarda dados do laser
    }

    auto [rx, ry] = meters_to_cells(x_, y_);
    
    float now_cel = potential_map_[toIndex(rx, ry)];

    //Em vez de pegar apenas os pontos mais próximos, faz uma média de vários valores para ter uma melhor noção do gradiente
    //caso encontre uma parede, interrompa a busca
    float sum_right = now_cel;
    int count_right = 1;
    for (int i = 1; i <= distant_look; ++i) {
      if (rx + i >= width_ - 1) break;
      int idx = toIndex(rx + i, ry);
      sum_right += potential_map_[idx];
      count_right++;
      if (map_msg_->data[idx] >= 90) break; 
    }
    float val_right = sum_right / count_right;

    float sum_left = now_cel;
    int count_left = 1;
    for (int i = 1; i <= distant_look; ++i) {
      if (rx - i <= 0) break;
      int idx = toIndex(rx - i, ry);
      sum_left += potential_map_[idx];
      count_left++;
      if (map_msg_->data[idx] >= 90) break;
    }
    float val_left = sum_left / count_left;

    float sum_up = now_cel;
    int count_up = 1;
    for (int i = 1; i <= distant_look; ++i) {
      if (ry + i >= height_ - 1) break;
      int idx = toIndex(rx, ry + i);
      sum_up += potential_map_[idx];
      count_up++;
      if (map_msg_->data[idx] >= 90) break;
    }
    float val_up = sum_up / count_up;

    float sum_down = now_cel;
    int count_down = 1;
    for (int i = 1; i <= distant_look; ++i) {
      if (ry - i <= 0) break;
      int idx = toIndex(rx, ry - i);
      sum_down += potential_map_[idx];
      count_down++;
      if (map_msg_->data[idx] >= 90) break;
    }
    float val_down = sum_down / count_down;

    //Equação de laplace e descida de gradiente
    double grad_x = (val_right - val_left) / 2.0;
    double grad_y = (val_up - val_down) / 2.0;

    double desc_x = -grad_x;
    double desc_y = -grad_y;
    
    //Se a magnitude do gradiente for baixa o suficiente, pare a exploração
    double hypot = std::hypot(desc_x, desc_y);
    // std::cout << hypot << std::endl;
    if(hypot < GRADIENT_LIMIT){
      RCLCPP_INFO(this->get_logger(), "fim do gradiente");
      stop();
      return;
    }

    //calcula o angulo do gradiente
    double yaw = std::atan2(desc_y, desc_x);
    
    //só muda de direção caso a diferença da direção antiga com a nova seja >=0.1
    if(first_arrow){
      first_arrow = false;
    } else{
      if(std::abs(yaw - last_arrow) < 0.1){
        yaw = last_arrow;
      }
    }
    
    last_arrow = yaw;
    
    geometry_msgs::msg::PoseStamped arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = this->now();
    arrow.pose.position.x = x_;
    arrow.pose.position.y = y_;    
    arrow.pose.orientation.x = 0.0;
    arrow.pose.orientation.y = 0.0;
    arrow.pose.orientation.z = std::sin(yaw / 2.0);
    arrow.pose.orientation.w = std::cos(yaw / 2.0);
    target_pose_pub_->publish(arrow);
    
    double angle = yaw - theta_;
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;

    geometry_msgs::msg::Twist cmd_vel;
    
    //limito as velocidades
    double angular = std::clamp(kp_angular * angle, -0.2, 0.2);
    double linear = std::clamp(kp_linear * std::abs(std::cos(angle)), 0.0, 0.15);

    /////////////////////////////////////////////////////////////////////////////////
    //rotina para evitar que o robô bata em obstaculo
    double front_dist = get_front_distance(15.0); // 20 graus para cada lado
  
    // Distância de segurança (parar e girar se menor que isso)
    const double MIN_DIST = 0.5;
  
    if (front_dist < MIN_DIST) {
      // Obstáculo detectado, girar
      linear = 0;
      angular = -0.3;
      // publish_velocity(0.0, 0.1);
    }
    ////////////////////////////////////////////////////////////////////////////////////
    // std::cout << "linear: " << linear << " , angular: " << angular << std::endl;
    publish_velocity(linear, angular);
  }
  
  void on_map() override
  {
    //inicialização, só roda uma vez
    if (width_ != map_msg_->info.width || height_ != map_msg_->info.height) {
      width_ = map_msg_->info.width;
      height_ = map_msg_->info.height;
      potential_map_.assign(width_ * height_, 0);
    }
    
    //preenchendo com obstaculos e pontos desconhecidos
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        int idx = toIndex(x, y);
        int value = map_msg_->data[idx];

        if (value >= 90) potential_map_[idx] = 1;   //as vezes o nó do mapa não está conseguindo fixar em 100
        else if(value <= 0) potential_map_[idx] = 0;
      }
    }

    // GaussSeidel
    for (int i = 0; i < N; ++i) {
      for (int y = 1; y < height_ - 1; ++y) {
        for (int x = 1; x < width_ - 1; ++x) {
          int idx = toIndex(x, y);
          int value = map_msg_->data[idx];

          if (value >= 0 and value < 90) {
            float sum = potential_map_[toIndex(x + 1, y)] + 
                                  potential_map_[toIndex(x - 1, y)] + 
                                  potential_map_[toIndex(x, y + 1)] + 
                                  potential_map_[toIndex(x, y - 1)];
            
            potential_map_[idx] = sum / 4;
          }
        }
      }
    }

    nav_msgs::msg::OccupancyGrid potencial;
    potencial.header = map_msg_->header;
    potencial.info = map_msg_->info;   
    potencial.data.resize(width_ * height_);

    for (int i = 0; i < width_ * height_; ++i) {
      if (map_msg_->data[i] == -1) potencial.data[i] = -1;
      else {
        potencial.data[i] = potential_map_[i] * 100;    // potential [0,1] to ROS[0,100]
      }
    }

    potential_pub_->publish(potencial);


  }
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorationExample>());
  rclcpp::shutdown();
  return 0;
}
