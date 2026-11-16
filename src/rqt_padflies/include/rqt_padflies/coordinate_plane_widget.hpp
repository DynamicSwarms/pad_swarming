#pragma once

#include <QWidget>
#include <QPainter>
#include <QPen>
#include <QColor>
#include <QMouseEvent>
#include <QPointF>
#include <QSlider>
#include <QVBoxLayout>

#include <iostream>
#include <ostream>
class CoordinatePlaneWidget : public QWidget
{
    Q_OBJECT

public:
    CoordinatePlaneWidget(QWidget *parent = nullptr)
    {
        (void)parent;
        setMouseTracking(true);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        setSizeIncrement(1, 1);
        setFixedSize(300, 300);
    }

    QPointF selectedPoint() const { return m_selected_point; }
    QPointF hoverPoint() const { return screenToWorld(m_hover_point); }

    void set_scale(double scale) { m_scale = scale; update(); }

    void set_target(double x, double y) {
        m_selected_point = QPointF(x, y);
        m_has_selected_point = true;
        update();
    }
signals:
    void pointSelected(QPointF point);
    void pointHovered(QPointF point, bool valid);
    void wheelScrolled(double delta);
private: 
    bool hasHeightForWidth() const override
    {
        return true;
    }

    int heightForWidth(int w) const override
    {
        return w;
    }

    void resizeEvent(QResizeEvent *event) override
    {
        QWidget::resizeEvent(event);
        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override 
    {
        (void)event;
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), QColor(30, 30, 30));

        painter.setPen(QPen(QColor(70, 70, 70), 1));

        int lines = 10;
        for (int i = 0; i <= lines; i++) {
            int x = i * width() / lines;
            int y = i * height() / lines;

            painter.drawLine(x, 0, x, height());
            painter.drawLine(0, y, width(), y);
        }

        // center axes
        painter.setPen(QPen(Qt::white, 2));
        painter.drawLine(width()/2, 0, width()/2, height());
        painter.drawLine(0, height()/2, width(), height()/2);

        // markings
        painter.setPen(QPen(QColor(180, 180, 180), 1));

        
        painter.drawText(width() - 40, height()/2 - 5,
                        QString::number(m_scale, 'f', 1));

        painter.drawText(5, height()/2 - 5,
                        QString::number(-m_scale, 'f', 1));

        // Y-axis max labels (middle vertical axis)
        painter.drawText(width()/2 + 5, 15,
                        QString::number(m_scale, 'f', 1));

        painter.drawText(width()/2 + 5, height() - 5,
                        QString::number(-m_scale, 'f', 1));

        // hover point
        if (m_has_hover) {
            QPointF s = m_hover_point;
            painter.setPen(QPen(Qt::yellow, 2));
            painter.drawEllipse(s, 5, 5);
        }

        // selected point
        if(m_has_selected_point) {
            QPointF s = worldToScreen(m_selected_point);
            painter.setPen(QPen(Qt::red, 3));
            painter.drawEllipse(s, 6, 6);
        }
    }
    void mouseMoveEvent(QMouseEvent *event) override
    {
        m_hover_point = event->pos();
        m_has_hover = true;

        emit pointHovered(screenToWorld(m_hover_point), true);
        update();
    }
    void mousePressEvent(QMouseEvent *event) override
    {    
        m_selected_point = screenToWorld(event->pos());
        m_has_selected_point = true;

        emit pointSelected(m_selected_point);
        update();
    }

    void wheelEvent(QWheelEvent *event) override
    {
        double delta = event->angleDelta().y() / 120.0; // 120 is the standard for one notch
        
        emit pointHovered(screenToWorld(m_hover_point), true);
        emit wheelScrolled(delta);
        update();        
    }

    

    void leaveEvent(QEvent *event) override
    {
        (void)event;
        m_has_hover = false;
        update();
    }


private:
    QPointF screenToWorld(const QPointF &p) const
    {
        double nx = (p.x() / width()) * 2.0 - 1.0;
        double ny = 1.0 - (p.y() / height()) * 2.0;

        return QPointF(ny * m_scale, -nx * m_scale) ;
    }
    QPointF worldToScreen(const QPointF &p) const
    {        
        double nx = p.x() / m_scale;
        double ny = p.y() / m_scale;

        double x = (-ny + 1.0) * 0.5 * width();
        double y = (1.0 - (nx + 1.0) * 0.5) * height();

        return QPointF(x, y);
    }



private:
    QPointF m_hover_point; // in screen coordinates
    QPointF m_selected_point; // in world coordinates
    bool m_has_hover = false;
    bool m_has_selected_point = false;

private:
    double m_scale = 1.0; // world radius in meters
};