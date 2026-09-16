#include "atlantis_rviz_plugins/material_panel.hpp"

#include <algorithm>

#include <QHeaderView>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace atlantis_rviz_plugins
{

MaterialPanel::MaterialPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  table_ = new QTableWidget(this);
  table_->setColumnCount(3);
  table_->setHorizontalHeaderLabels({"Material", "Location", "Amount"});
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table_->verticalHeader()->setVisible(false);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(table_);
  setLayout(layout);

  connect(this, &MaterialPanel::stockReceived, this, &MaterialPanel::updateTable);
}

void MaterialPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  subscription_ = node_->create_subscription<material_handler_msgs::msg::MaterialStockArray>(
    "material_stock", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&MaterialPanel::stockCallback, this, std::placeholders::_1));
}

void MaterialPanel::stockCallback(
  const material_handler_msgs::msg::MaterialStockArray::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stocks_ = msg->stocks;
    std::sort(
      stocks_.begin(), stocks_.end(),
      [](const auto & a, const auto & b) {
        if (a.material == b.material) {
          return a.location < b.location;
        }
        return a.material < b.material;
      });
  }
  Q_EMIT stockReceived();
}

void MaterialPanel::updateTable()
{
  std::vector<material_handler_msgs::msg::MaterialStock> stocks;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stocks = stocks_;
  }

  table_->setRowCount(static_cast<int>(stocks.size()));
  for (size_t i = 0; i < stocks.size(); ++i) {
    const int row = static_cast<int>(i);
    table_->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(stocks[i].material)));
    table_->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(stocks[i].location)));
    table_->setItem(row, 2, new QTableWidgetItem(QString::number(stocks[i].amount, 'f', 2)));
  }
}

}  // namespace atlantis_rviz_plugins

PLUGINLIB_EXPORT_CLASS(atlantis_rviz_plugins::MaterialPanel, rviz_common::Panel)