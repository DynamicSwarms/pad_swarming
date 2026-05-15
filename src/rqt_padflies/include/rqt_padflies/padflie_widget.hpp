#pragma once

#include "ui_padflie.h"
#include <QPainter>
#include <QTimer>
#include "padflie_message_parser.hpp"


namespace rqt_padflies
{

inline QPixmap makeCircle(const QColor & color, int size = 16)
{
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);

    painter.setBrush(color);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(0, 0, size, size);

    return pixmap;
}

class PadflieWidget : public QWidget
{
  Q_OBJECT
  public:
    PadflieWidget(QWidget *parent, int id, std::shared_ptr<PadflieMessageParser> message_parser)
    : QWidget(parent)
    , m_message_parser(message_parser)
    {
      m_ui.setupUi(this);
      m_ui.label->setText(QString::number(id));

      m_ui.label->setPixmap(makeCircle(Qt::red));

      m_update_timer.setInterval(1000); // 1 second timeout for availability check
      m_update_timer.start();
      m_update_timer.callOnTimeout([this]() {
        m_ui.label->setPixmap(makeCircle(Qt::red));
        m_update_timer.stop();
      });

      m_message_parser->set_availability_callback([this]() {
        this->set_available();
      });
    }
    ~PadflieWidget() = default;

    int getHeight() const {
      return m_ui.battery_progress->height() * 1.1;
    }

    void set_available() {
      m_ui.label->setPixmap(makeCircle(Qt::green ));
      m_update_timer.start();
    }

  private:
    std::shared_ptr<PadflieMessageParser> m_message_parser;
    QTimer m_update_timer;
    Ui::Padflie m_ui;

};

}  // namespace rqt_padflies