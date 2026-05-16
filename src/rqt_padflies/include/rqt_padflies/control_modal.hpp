#include "rqt_padflies/padflie_control_connection.hpp"
#include "ui_padflie_control_modal.h"
#include <Eigen/Dense>

#include "rqt_padflies/coordinate_plane_widget.hpp"



namespace rqt_padflies
{

inline double slider_value_to_height(int slider_value)
{
        return slider_value / 10.0 * 1.0; // Slider is 0-49 -> 0-5m
}

class ControlModal : public QDialog
{
    Q_OBJECT
public:
    ControlModal(
        std::shared_ptr<PadflieControlConnection> control_connection,
        QWidget* parent = nullptr)
        : QDialog(parent)
        , m_control_connection(control_connection)
    {
        m_ui.setupUi(this);
        
        // Fix right slider
        m_ui.gridLayout->setColumnMinimumWidth(3, 0);
        m_ui.gridLayout->setColumnStretch(3, 0);
        m_ui.height_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);



        connect(m_ui.takeoff_button, &QPushButton::pressed, this, &ControlModal::on_takeoff_button_clicked);
        connect(m_ui.land_button, &QPushButton::pressed, this, &ControlModal::on_land_button_clicked);
        
        double height = slider_value_to_height(m_ui.z_slider->value());
        m_ui.height_label->setText(QString("%1 m").arg(height, 0, 'f', 1));

        m_coordinate_widget = new CoordinatePlaneWidget(nullptr);
        m_ui.gridLayout->addWidget(m_coordinate_widget, 0, 1, 1, 2);         
        m_ui.gridLayout->setRowStretch(0, 1);
        m_ui.gridLayout->setColumnStretch(1, 1);   
        
        connect(m_ui.scale_slider, &QSlider::valueChanged, this, [this](int value) {
            double scale = value / 10.0;
            m_coordinate_widget->set_scale(scale);
        });
        emit m_ui.scale_slider->valueChanged(m_ui.scale_slider->value());


        connect(m_coordinate_widget,
                &CoordinatePlaneWidget::pointSelected,
                this,
                [this](QPointF p)
        {
            double z_height = slider_value_to_height(m_ui.z_slider->value());
            Eigen::Vector3d target(p.x(), p.y(), z_height);
            
            m_ui.current_target_label->setText(QString("Current Target:(%1, %2, %3)").arg(target.x(), 0, 'f', 1).arg(target.y(), 0, 'f', 1).arg(target.z(), 0, 'f', 1));
            set_target(target);
        });


        connect(m_coordinate_widget,
                &CoordinatePlaneWidget::pointHovered,
                this,
                [this](QPointF p, bool valid)
        {
            if (valid) {
                double z_height = slider_value_to_height(m_ui.z_slider->value());
                Eigen::Vector3d target(p.x(), p.y(), z_height);
                m_ui.target_label->setText(QString("(%1, %2, %3)").arg(target.x(), 0, 'f', 1).arg(target.y(), 0, 'f', 1).arg(target.z(), 0, 'f', 1));
            }
        });

        connect(
            m_coordinate_widget,
            &CoordinatePlaneWidget::wheelScrolled,
            this,
            [this](int delta)
            {
                int current = m_ui.scale_slider->value();
                int step = (delta > 0) ? 1 : -1;
                m_ui.scale_slider->setValue(current + step);
            });

        connect(m_ui.z_slider, &QSlider::valueChanged, this, [this](int value) {
            double height = slider_value_to_height(value);
            m_ui.height_label->setText(QString("%1 m").arg(height, 0, 'f', 1));
            m_ui.target_label->setText(QString("(%1, %2, %3)").arg(m_coordinate_widget->hoverPoint().x(), 0, 'f', 1).arg(m_coordinate_widget->hoverPoint().y(), 0, 'f', 1).arg(height, 0, 'f', 1));
            set_target(Eigen::Vector3d(m_coordinate_widget->selectedPoint().x(), m_coordinate_widget->selectedPoint().y(), height));
        });


        Eigen::Vector3d current_target;
        if (m_control_connection->get_target(current_target)) {
            m_coordinate_widget->set_target(current_target.x(), current_target.y());
            m_ui.z_slider->setValue(static_cast<int>(current_target.z() * 10.0));
        }
    }
    ~ControlModal() = default;

private slots:
    void on_takeoff_button_clicked() {
        m_control_connection->takeoff();
    }

    void on_land_button_clicked() {
        m_control_connection->land();
    }
private: 
    void set_target(Eigen::Vector3d target){
        m_control_connection->set_target(target);
    }

private: 
    Ui::PadflieControlModal m_ui;
    std::shared_ptr<PadflieControlConnection> m_control_connection;

    CoordinatePlaneWidget* m_coordinate_widget;
};

}