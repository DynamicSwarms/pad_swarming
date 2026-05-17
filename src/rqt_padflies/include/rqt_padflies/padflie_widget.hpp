#pragma once

#include "ui_padflie.h"
#include <QPainter>
#include <QTimer>
#include "rqt_padflies/padflie_ros_connection.hpp"
#include "rqt_padflies/control_modal.hpp"


#include <QMetaType>
#include <QPointer>
#include <lifecycle_msgs/msg/state.hpp>

namespace rqt_padflies
{

inline const QPixmap& circlePixmap(const QColor& color)
{
    static std::unordered_map<int, QPixmap> cache;

    int key = color.rgba();

    auto it = cache.find(key);
    if (it != cache.end())
        return it->second;

    QPixmap pixmap(16, 16);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setBrush(color);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(0, 0, 16, 16);

    cache[key] = pixmap;
    return cache[key];
}

inline void set_lifecycle_button(
  QPushButton * button,
  bool active,
  bool clickable)
{
  static QPalette defaultPalette;
  static bool paletteInitialized = false;
  if (!paletteInitialized) {
    defaultPalette = button->palette();
    paletteInitialized = true;
  }

  QPalette pal = defaultPalette;

  if (active) {
      pal.setColor(QPalette::Button, QColor("#4CAF50"));
      pal.setColor(QPalette::ButtonText, Qt::black);

      button->setEnabled(false); // current state not clickable
  }
  else if (clickable) {
      pal.setColor(QPalette::Button, defaultPalette.color(QPalette::Button));
      pal.setColor(QPalette::ButtonText, Qt::black);

      button->setEnabled(true);
  }
  else {
      pal.setColor(QPalette::Button,
                    defaultPalette.color(QPalette::Button).darker(120));
      pal.setColor(QPalette::ButtonText, QColor("#9E9E9E"));

      button->setEnabled(false);
  }

  button->setPalette(pal);
  button->setAutoFillBackground(true);
}

class PadflieWidget : public QWidget
{
  Q_OBJECT
  public:
    PadflieWidget(
        QWidget *parent, int id,
        std::shared_ptr<PadflieROSConnection> ros_connection)
    : QWidget(parent)
    , m_ros_connection(ros_connection)
    {
      qRegisterMetaType<lifecycle_msgs::msg::State>("lifecycle_msgs::msg::State");
      m_ros_connection->get_lifecycle_connection()->add_lifecycle_state_callback(
        [this](lifecycle_msgs::msg::State current_state) {emit current_lifecycle_changed(current_state);}
      );

      m_ui.setupUi(this);
      m_ui.label_id->setText(QString("0x%1").arg(QString::number(id, 16).toUpper()));
      m_ui.label_id->setToolTip(QString::number(id));

      m_ui.label->setFixedSize(16, 16);
      m_ui.label->setScaledContents(true);
      m_ui.label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
      m_ui.label->setPixmap(circlePixmap(Qt::red));

      connect(
        m_ui.flight_control_modal,
        &QPushButton::clicked,
        this,
        [this]()
        {
          auto *modal =
            new ControlModal(m_ros_connection->get_control_connection(), this);

          modal->setAttribute(Qt::WA_DeleteOnClose);
          modal->setWindowModality(Qt::WindowModal); // optional "overlay feel"
          modal->show();
        });

      m_availability_timeout_timer.setInterval(1000); // 1 second timeout for availability check
      m_availability_timeout_timer.start();
      m_availability_timeout_timer.callOnTimeout([this]() {
        QSignalBlocker blocker(this->m_ui.label);
        this->m_ui.label->setPixmap(circlePixmap(Qt::red));
        this->m_availability_timeout_timer.stop();
      });      

      connect(m_ui.active_button, &QPushButton::pressed, this, &PadflieWidget::on_active_button_clicked);
      connect(m_ui.inactive_button, &QPushButton::pressed, this, &PadflieWidget::on_inactive_button_clicked);
      connect(this, &PadflieWidget::current_lifecycle_changed, this, &PadflieWidget::set_current_lifecycle_state);
    
      set_lifecycle_button(m_ui.active_button, false, false);
      set_lifecycle_button(m_ui.inactive_button, false, false);
      set_lifecycle_button(m_ui.unconfigured_button, false, false);
      m_ui.flight_control_modal->setEnabled(false);
    }
    ~PadflieWidget() {
      m_ros_connection->get_lifecycle_connection()->add_lifecycle_state_callback(nullptr); // Unregister callback
    }

    int getHeight() const {
      return m_ui.battery_progress->height() * 1.1;
    }

 

  private:
    QTimer m_availability_timeout_timer;
    Ui::Padflie m_ui;
    std::shared_ptr<PadflieROSConnection> m_ros_connection;


  private: 
      //this->setEnabled(m_lifecycle_connection->padflie_is_available());
      
    void lifecycle_state_changed_callback(lifecycle_msgs::msg::State current_state) {
      
    }

    void set_current_lifecycle_state(lifecycle_msgs::msg::State current_state)
    {
      if (current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        m_ui.flight_control_modal->setEnabled(true);
      } else {
        m_ui.flight_control_modal->setEnabled(false);
      }
      // _lifecycle_button(button, active, clickable)
      set_lifecycle_button(m_ui.active_button, false, false);
      set_lifecycle_button(m_ui.inactive_button, false, false);
      set_lifecycle_button(m_ui.unconfigured_button, false, false);

      if (current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        set_lifecycle_button(m_ui.active_button, true, false);
        set_lifecycle_button(m_ui.inactive_button, false, true);
      }
      if (current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      {
        set_lifecycle_button(m_ui.inactive_button, true, false);
        set_lifecycle_button(m_ui.active_button, false, true);
        m_ros_connection->get_control_connection()->reset_target();
      }
      if (current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
      {
        set_lifecycle_button(m_ui.unconfigured_button, true, false);
        set_lifecycle_button(m_ui.inactive_button, false, true);
        m_ros_connection->get_control_connection()->reset_target();
      }

    }

  public slots: 
    void set_available() {
      m_ui.label->setPixmap(circlePixmap(Qt::green));
      m_availability_timeout_timer.start();
    }

  private slots:
    void on_active_button_clicked() {
      m_ros_connection->get_lifecycle_connection()->activate_padflie();    
    }

    void on_inactive_button_clicked() {
      m_ros_connection->get_lifecycle_connection()->deactivate_padflie();
    }

  signals:
    void current_lifecycle_changed(lifecycle_msgs::msg::State current_state);

};

}  // namespace rqt_padflies