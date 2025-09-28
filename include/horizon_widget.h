#pragma once
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <deque>
#include <cmath>

struct UiState
{
    // Orientation
    double roll = 0;  // radians
    double pitch = 0; // radians
    double yaw = 0;   // radians (heading)

    // Velocities
    double forward_velocity = 0;  // m/s
    double side_velocity = 0;     // m/s
    double vertical_velocity = 0; // m/s
    double total_velocity = 0;    // m/s (airspeed)

    // Position
    double altitude = 0;  // meters
    double north_pos = 0; // meters
    double east_pos = 0;  // meters

    // Rotation rates
    double roll_rate = 0;  // rad/s
    double pitch_rate = 0; // rad/s
    double yaw_rate = 0;   // rad/s

    // Control inputs
    double aileron = 0;  // -1 to 1
    double elevator = 0; // -1 to 1
    double rudder = 0;   // -1 to 1
    double throttle = 0; // 0 to 1

    // Derived values
    double angle_of_attack = 0; // radians
    double sideslip_angle = 0;  // radians
    double g_force = 1.0;       // g units

    // Time
    double time = 0;
};

class HorizonWidget : public QWidget
{
    Q_OBJECT

public:
    explicit HorizonWidget(QWidget *parent = nullptr);
    void setUiState(const UiState &u);
    ~HorizonWidget() override;

protected:
    void paintEvent(QPaintEvent *) override;

private:
    void drawArtificialHorizon(QPainter &p, const QRect &rect);
    void drawAirspeedIndicator(QPainter &p, const QRect &rect);
    void drawAltimeter(QPainter &p, const QRect &rect);
    void drawHeadingIndicator(QPainter &p, const QRect &rect);
    void drawVerticalSpeedIndicator(QPainter &p, const QRect &rect);
    void drawTurnCoordinator(QPainter &p, const QRect &rect);
    void drawControlPositions(QPainter &p, const QRect &rect);
    void drawDataPanel(QPainter &p, const QRect &rect);
    void drawMiniMap(QPainter &p, const QRect &rect);
    void drawGMeter(QPainter &p, const QRect &rect);
    void drawAngleIndicators(QPainter &p, const QRect &rect);
    void drawThrottleGauge(QPainter &p, const QRect &rect);

    void drawCircularGauge(QPainter &p, const QRect &rect,
                           double value, double min, double max,
                           const QString &label, const QString &unit,
                           const QColor &color);

    UiState ui_;
    std::deque<QPointF> flight_path_;     // Trail of positions for mini-map
    std::deque<double> altitude_history_; // For altitude graph
    std::deque<double> speed_history_;    // For speed graph

    // Animation helpers
    double animated_roll_ = 0;
    double animated_pitch_ = 0;
    double animated_heading_ = 0;
};