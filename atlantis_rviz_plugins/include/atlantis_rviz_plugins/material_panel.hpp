#ifndef ATLANTIS_RVIZ_PLUGINS__MATERIAL_PANEL_HPP_
#define ATLANTIS_RVIZ_PLUGINS__MATERIAL_PANEL_HPP_

#include <mutex>
#include <vector>

#include <QTableWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <material_handler_msgs/msg/material_stock.hpp>
#include <material_handler_msgs/msg/material_stock_array.hpp>

namespace atlantis_rviz_plugins
{

class MaterialPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MaterialPanel(QWidget * parent = nullptr);

  void onInitialize() override;

Q_SIGNALS:
  void stockReceived();

private Q_SLOTS:
  void updateTable();

private:
  void stockCallback(const material_handler_msgs::msg::MaterialStockArray::SharedPtr msg);

  QTableWidget * table_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<material_handler_msgs::msg::MaterialStockArray>::SharedPtr subscription_;
  std::vector<material_handler_msgs::msg::MaterialStock> stocks_;
  std::mutex mutex_;
};

}  // namespace atlantis_rviz_plugins

#endif  // ATLANTIS_RVIZ_PLUGINS__MATERIAL_PANEL_HPP_