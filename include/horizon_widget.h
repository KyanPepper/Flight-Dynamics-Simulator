#pragma once

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QList>
#include <QPointF>

/**
 * Complete state of the aircraft for display purposes
 */
struct UiState
{
    // Orientation (radians)
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    // Velocities (m/s in body frame)
    double forward_velocity = 0.0;
    double side_velocity = 0.0;
    double vertical_velocity = 0.0;
    double total_velocity = 0.0;

    // Position (meters)
    double altitude = 0.0;
    double north_pos = 0.0;
    double east_pos = 0.0;

    // Rotation rates (rad/s)
    double roll_rate = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate = 0.0;

    // Control inputs
    double aileron = 0.0;
    double elevator = 0.0;
    double rudder = 0.0;
    double throttle = 0.0;

    // Derived values
    double angle_of_attack = 0.0;
    double sideslip_angle = 0.0;
    double g_force = 1.0;

    // Time
    double time = 0.0;
};

/**
 * Main display widget showing all flight instruments
 */
class HorizonWidget : public QWidget
{
    Q_OBJECT

public:
    explicit HorizonWidget(QWidget *parent = nullptr);
    ~HorizonWidget();

    void setUiState(const UiState &u);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    // Drawing functions for each instrument
    void drawArtificialHorizon(QPainter &p, const QRect &rect);
    void drawAirspeedIndicator(QPainter &p, const QRect &rect);
    void drawAltimeter(QPainter &p, const QRect &rect);
    void drawHeadingIndicator(QPainter &p, const QRect &rect);
    void drawVerticalSpeedIndicator(QPainter &p, const QRect &rect);
    void drawTurnCoordinator(QPainter &p, const QRect &rect);
    void drawControlPositions(QPainter &p, const QRect &rect);
    void drawThrottleGauge(QPainter &p, const QRect &rect);
    void drawAngleIndicators(QPainter &p, const QRect &rect);
    void drawMiniMap(QPainter &p, const QRect &rect);
    void drawGMeter(QPainter &p, const QRect &rect);
    void drawDataPanel(QPainter &p, const QRect &rect);

    // Helper for circular gauges
    void drawCircularGauge(QPainter &p, const QRect &rect,
                           double value, double min, double max,
                           const QString &label, const QString &unit,
                           const QColor &color);

    // Current state
    UiState ui_;

    // Smoothed values for display
    double animated_roll_ = 0.0;
    double animated_pitch_ = 0.0;
    double animated_heading_ = 0.0;

    // History for trends and mini-map
    QList<QPointF> flight_path_;
    QList<double> altitude_history_;
    QList<double> speed_history_;
};