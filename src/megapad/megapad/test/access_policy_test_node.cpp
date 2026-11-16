#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <QApplication>
#include <QBrush>
#include <QColor>
#include <QFont>
#include <QGraphicsItem>
#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "megapad/megapad_access_policy.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const QColor kBackground{24, 28, 36};
const QColor kNeutral{91, 102, 120};
const QColor kGreen{46, 204, 113};
const QColor kRed{231, 76, 60};
const QColor kOrange{243, 156, 18};
const QColor kSelected{241, 196, 15};

enum class EvaluationState
{
    HOLDER,
    ELIGIBLE,
    BLOCKED
};

class MetricScene : public QGraphicsScene
{
public:
    using QGraphicsScene::QGraphicsScene;

protected:
    void drawBackground(QPainter * painter, const QRectF & rectangle) override
    {
        painter->fillRect(rectangle, kBackground);

        constexpr double minor_spacing = 50.0;   // 0.20 m
        constexpr double major_spacing = 250.0;  // 1.00 m
        const auto draw_grid = [painter, &rectangle](double spacing, const QPen & pen) {
            painter->setPen(pen);
            const double left = std::floor(rectangle.left() / spacing) * spacing;
            const double top = std::floor(rectangle.top() / spacing) * spacing;
            for (double x = left; x <= rectangle.right(); x += spacing) {
                painter->drawLine(QLineF(x, rectangle.top(), x, rectangle.bottom()));
            }
            for (double y = top; y <= rectangle.bottom(); y += spacing) {
                painter->drawLine(QLineF(rectangle.left(), y, rectangle.right(), y));
            }
        };

        draw_grid(minor_spacing, QPen(QColor(43, 49, 61), 1.0));
        draw_grid(major_spacing, QPen(QColor(64, 73, 89), 2.0));
    }
};

class PadItem : public QGraphicsRectItem
{
public:
    PadItem(int id, const QRectF & rectangle, std::function<void(int)> on_clicked)
    : QGraphicsRectItem(rectangle), m_id(id), m_on_clicked(std::move(on_clicked))
    {
        setPen(QPen(QColor(130, 143, 164), 2.0));
        setBrush(QBrush(kNeutral));
        setToolTip(QString("Pad %1").arg(id));

        auto * label = new QGraphicsSimpleTextItem(QString::number(id), this);
        label->setBrush(Qt::white);
        QFont font;
        font.setBold(true);
        label->setFont(font);
        const QRectF label_bounds = label->boundingRect();
        label->setPos(
            rectangle.center().x() - label_bounds.width() / 2.0,
            rectangle.center().y() - label_bounds.height() / 2.0);
    }

    int id() const { return m_id; }

    QPointF center() const { return rect().center(); }

    void set_evaluation(bool used, EvaluationState state)
    {
        if (!used) {
            setBrush(kNeutral);
        } else if (state == EvaluationState::HOLDER) {
            setBrush(kGreen);
        } else if (state == EvaluationState::ELIGIBLE) {
            setBrush(kOrange);
        } else {
            setBrush(kRed);
        }
    }

    void set_selected(bool selected)
    {
        setPen(QPen(selected ? kSelected : QColor(130, 143, 164), selected ? 4.0 : 2.0));
        setZValue(selected ? 2.0 : 0.0);
    }

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent * event) override
    {
        m_on_clicked(m_id);
        QGraphicsRectItem::mousePressEvent(event);
    }

private:
    int m_id;
    std::function<void(int)> m_on_clicked;
};

class QuadcopterItem : public QGraphicsItem
{
public:
    QuadcopterItem(
        int id,
        std::function<void(int)> on_selected,
        std::function<void(int)> on_double_clicked,
        std::function<void()> on_released)
    : m_id(id)
    , m_on_selected(std::move(on_selected))
    , m_on_double_clicked(std::move(on_double_clicked))
    , m_on_released(std::move(on_released))
    {
        setFlags(ItemIsMovable | ItemIsSelectable | ItemSendsGeometryChanges);
        setCursor(Qt::OpenHandCursor);
        setToolTip(QString("Crazyflie %1 — drag to move").arg(id));
    }

    QRectF boundingRect() const override { return {-31.0, -31.0, 62.0, 62.0}; }

    void paint(QPainter * painter, const QStyleOptionGraphicsItem *, QWidget *) override
    {
        painter->setRenderHint(QPainter::Antialiasing);
        const QColor state_color = m_state == EvaluationState::HOLDER ? kGreen :
            (m_state == EvaluationState::ELIGIBLE ? kOrange : kRed);
        painter->setPen(QPen(isSelected() ? kSelected : state_color, 3.0));
        painter->setBrush(QBrush(QColor(44, 51, 64)));

        painter->drawLine(QPointF(-17, -17), QPointF(17, 17));
        painter->drawLine(QPointF(-17, 17), QPointF(17, -17));
        for (const QPointF rotor : std::array<QPointF, 4>{
                 QPointF{-19, -19}, QPointF{19, -19},
                 QPointF{-19, 19}, QPointF{19, 19}})
        {
            painter->drawEllipse(rotor, 8.0, 8.0);
        }
        painter->setBrush(state_color);
        painter->drawRoundedRect(QRectF(-12, -8, 24, 16), 4.0, 4.0);
        painter->setPen(Qt::white);
        painter->drawText(QRectF(-12, -8, 24, 16), Qt::AlignCenter, QString::number(m_id));
    }

    void set_evaluation(EvaluationState state)
    {
        m_state = state;
        update();
    }

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent * event) override
    {
        m_on_selected(m_id);
        setCursor(Qt::ClosedHandCursor);
        QGraphicsItem::mousePressEvent(event);
    }

    void mouseReleaseEvent(QGraphicsSceneMouseEvent * event) override
    {
        setCursor(Qt::OpenHandCursor);
        QGraphicsItem::mouseReleaseEvent(event);
        m_on_released();
    }

    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event) override
    {
        m_on_double_clicked(m_id);
        QGraphicsItem::mouseDoubleClickEvent(event);
    }

private:
    int m_id;
    EvaluationState m_state{EvaluationState::BLOCKED};
    std::function<void(int)> m_on_selected;
    std::function<void(int)> m_on_double_clicked;
    std::function<void()> m_on_released;
};

class AccessPolicyWindow : public QMainWindow
{
public:
    AccessPolicyWindow()
    {
        setWindowTitle("MegaPad 2D Access Policy Test Node");
        resize(1120, 820);

        auto * central = new QWidget;
        auto * layout = new QHBoxLayout(central);
        m_scene = new MetricScene(-430, -390, 860, 780, this);
        auto * view = new QGraphicsView(m_scene);
        view->setRenderHint(QPainter::Antialiasing);
        view->setDragMode(QGraphicsView::NoDrag);
        view->setMinimumSize(870, 790);
        layout->addWidget(view, 1);

        auto * information = new QWidget;
        information->setFixedWidth(230);
        information->setStyleSheet(
            "background:#232934;color:#e9edf3;border-radius:8px;padding:8px;");
        auto * info_layout = new QVBoxLayout(information);
        auto * title = new QLabel("Access policy");
        QFont title_font;
        title_font.setPointSize(16);
        title_font.setBold(true);
        title->setFont(title_font);
        info_layout->addWidget(title);

        auto * instructions = new QLabel(
            "Drag a Crazyflie to update its position.\n\n"
            "Press a Crazyflie to inspect its path and marked pads. The selection clears on release.\n\n"
            "Orange means access can be granted. Double-click an orange Crazyflie to make it a holder.\n\n"
            "Double-click a green holder to release it.");
        instructions->setWordWrap(true);
        info_layout->addWidget(instructions);
        m_selection_label = new QLabel("Selected: none");
        m_selection_label->setWordWrap(true);
        info_layout->addWidget(m_selection_label);
        m_result_label = new QLabel;
        m_result_label->setWordWrap(true);
        info_layout->addWidget(m_result_label);
        info_layout->addStretch();

        auto * legend = new QLabel(
            "<span style='color:#2ecc71'>●</span> granted<br>"
            "<span style='color:#f39c12'>●</span> can be granted<br>"
            "<span style='color:#e74c3c'>●</span> blocked<br>"
            "<span style='color:#f1c40f'>●</span> selected");
        info_layout->addWidget(legend);
        layout->addWidget(information);
        setCentralWidget(central);

        create_pads();
        create_quadcopters();
        m_selection_line = m_scene->addLine(QLineF(), QPen(kSelected, 2.5, Qt::DashLine));
        m_selection_line->setZValue(1.0);
        evaluate();

        auto * timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, [this]() { evaluate(); });
        timer->start(50);
    }

private:
    static Eigen::Vector2d to_policy_position(const QPointF & point)
    {
        constexpr double pixels_per_meter = 250.0;
        return {point.x() / pixels_per_meter, -point.y() / pixels_per_meter};
    }

    void create_pads()
    {
        // 250 px/m makes each 50 px pad exactly 0.20 m wide.
        constexpr double size = 50.0;
        constexpr double gap = 2.0;
        constexpr double pitch = size + gap;
        constexpr double origin = -2.0 * pitch - size / 2.0;
        for (int row = 0; row < 5; ++row) {
            for (int column = 0; column < 5; ++column) {
                const int id = row * 5 + column;
                const QRectF rectangle(
                    origin + column * pitch, origin + row * pitch, size, size);
                auto * pad = new PadItem(
                    id, rectangle,
                    [this](int pad_id) { assign_selected_to(pad_id); });
                m_scene->addItem(pad);
                m_pads.push_back(pad);
            }
        }
    }

    void create_quadcopters()
    {
        constexpr int count = 25;
        constexpr double radius = 340.0;
        m_associated_pad.reserve(count);
        for (int index = 0; index < count; ++index) {
            const int cf_id = index;
            const double angle = -std::numbers::pi / 2.0 +
                2.0 * std::numbers::pi * static_cast<double>(index) / count;
            auto * quadcopter = new QuadcopterItem(
                cf_id,
                [this](int selected_cf_id) { select_quadcopter(selected_cf_id); },
                [this](int selected_cf_id) { toggle_holder(selected_cf_id); },
                [this]() { clear_selection(); });
            quadcopter->setPos(radius * std::cos(angle), radius * std::sin(angle));
            m_scene->addItem(quadcopter);
            m_quadcopters.push_back(quadcopter);
            m_associated_pad.push_back(index);
            m_is_holder.push_back(false);
        }
    }

    void select_quadcopter(int id)
    {
        m_selected_id = id;
        for (std::size_t index = 0; index < m_quadcopters.size(); ++index) {
            m_quadcopters[index]->setSelected(index == static_cast<std::size_t>(id));
            m_quadcopters[index]->setZValue(index == static_cast<std::size_t>(id) ? 3.0 : 0.0);
        }
        update_selection_label();
        update_selection_graphics();
    }

    void assign_selected_to(int pad_id)
    {
        if (m_selected_id < 0) {
            return;
        }
        m_associated_pad.at(static_cast<std::size_t>(m_selected_id)) = pad_id;
        update_selection_label();
        update_selection_graphics();
        evaluate();
    }

    void clear_selection()
    {
        m_selected_id = -1;
        for (auto * quadcopter : m_quadcopters) {
            quadcopter->setSelected(false);
            quadcopter->setZValue(0.0);
        }
        m_selection_label->setText("Selected: none");
        update_selection_graphics();
        evaluate();
    }

    void update_selection_graphics()
    {
        if (m_selected_id < 0) {
            for (auto * pad : m_pads) {
                pad->set_selected(false);
            }
            m_selection_line->setVisible(false);
            return;
        }

        m_selection_line->setVisible(true);
        const auto selected_index = static_cast<std::size_t>(m_selected_id);
        const auto pad_index = static_cast<std::size_t>(m_associated_pad[selected_index]);
        for (std::size_t index = 0; index < m_pads.size(); ++index) {
            m_pads[index]->set_selected(index == pad_index);
        }
        m_selection_line->setLine(QLineF(
            m_quadcopters[selected_index]->scenePos(), m_pads[pad_index]->center()));
    }

    void toggle_holder(int id)
    {
        const auto index = static_cast<std::size_t>(id);
        if (m_is_holder[index]) {
            m_is_holder[index] = false;
        } else if (m_evaluation[index] == EvaluationState::ELIGIBLE) {
            m_is_holder[index] = true;
        }
        evaluate();
    }

    void update_selection_label()
    {
        m_selection_label->setText(QString("Selected: CF %1 → pad %2")
            .arg(m_selected_id)
            .arg(m_associated_pad.at(static_cast<std::size_t>(m_selected_id))));
    }

    megapad::AccessGeometry2D geometry_for(int id) const
    {
        const int pad_id = m_associated_pad.at(static_cast<std::size_t>(id));
        return {
            to_policy_position(m_quadcopters.at(static_cast<std::size_t>(id))->scenePos()),
            to_policy_position(m_pads.at(static_cast<std::size_t>(pad_id))->center())};
    }

    std::size_t pad_index_for_position(const Eigen::Vector2d & position) const
    {
        for (std::size_t index = 0; index < m_pads.size(); ++index) {
            if (to_policy_position(m_pads[index]->center()).isApprox(position)) {
                return index;
            }
        }
        return m_pads.size();
    }

    void evaluate()
    {
        update_selection_graphics();
        std::vector<megapad::AccessGeometry2D> holders;
        std::vector<megapad::AccessGeometry2D> all_accesses;
        all_accesses.reserve(m_quadcopters.size());
        for (std::size_t index = 0; index < m_quadcopters.size(); ++index) {
            all_accesses.push_back(geometry_for(static_cast<int>(index)));
            if (m_is_holder[index]) {
                holders.push_back(all_accesses.back());
            }
        }

        std::array<EvaluationState, 25> pad_state{};
        std::array<bool, 25> pad_used{};
        m_evaluation.resize(m_quadcopters.size());

        for (std::size_t index = 0; index < m_quadcopters.size(); ++index) {
            const auto geometry = geometry_for(static_cast<int>(index));
            const EvaluationState state = m_is_holder[index] ? EvaluationState::HOLDER :
                (m_policy.can_grant(geometry, holders, all_accesses) ?
                    EvaluationState::ELIGIBLE : EvaluationState::BLOCKED);
            m_evaluation[index] = state;
            m_quadcopters[index]->set_evaluation(state);

            std::vector<Eigen::Vector2d> visible_positions;
            if (state == EvaluationState::HOLDER) {
                visible_positions =
                    m_policy.marked_associated_positions(geometry, all_accesses);
            } else if (state == EvaluationState::BLOCKED) {
                visible_positions =
                    m_policy.blocked_associated_positions(
                        geometry, holders, all_accesses);
            }
            for (const auto & marked_position : visible_positions) {
                const auto pad_index = pad_index_for_position(marked_position);
                if (pad_index == m_pads.size()) {
                    continue;
                }
                pad_used[pad_index] = true;
                if (static_cast<int>(state) > static_cast<int>(pad_state[pad_index])) {
                    pad_state[pad_index] = state;
                }
            }
        }

        for (std::size_t index = 0; index < m_pads.size(); ++index) {
            m_pads[index]->set_evaluation(pad_used[index], pad_state[index]);
        }
        const int eligible_count = static_cast<int>(std::count(
            m_evaluation.begin(), m_evaluation.end(), EvaluationState::ELIGIBLE));
        m_result_label->setText(QString("Holders: %1    Eligible: %2")
            .arg(holders.size())
            .arg(eligible_count));
    }

    QGraphicsScene * m_scene{nullptr};
    QLabel * m_selection_label{nullptr};
    QLabel * m_result_label{nullptr};
    QGraphicsLineItem * m_selection_line{nullptr};
    std::vector<PadItem *> m_pads;
    std::vector<QuadcopterItem *> m_quadcopters;
    std::vector<int> m_associated_pad;
    std::vector<bool> m_is_holder;
    std::vector<EvaluationState> m_evaluation;
    int m_selected_id{-1};
    megapad::MegaPadAccessPolicy m_policy;
};
}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    QApplication application(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("megapad_access_policy_test_node");
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);

    QTimer ros_timer;
    QObject::connect(
        &ros_timer, &QTimer::timeout, [executor]() { executor->spin_some(); });
    ros_timer.start(20);

    AccessPolicyWindow window;
    window.show();
    const int result = application.exec();
    rclcpp::shutdown();
    return result;
}
