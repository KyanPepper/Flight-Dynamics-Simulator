#pragma once

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

class ControlPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPanel(QWidget *parent = nullptr);

    // Get current control values
    double getAileron() const { return aileron_value_; }
    double getElevator() const { return elevator_value_; }
    double getRudder() const { return rudder_value_; }
    double getThrottle() const { return throttle_value_; }
    bool isAutopilotEnabled() const { return autopilot_enabled_; }

signals:
    void controlsChanged();
    void resetRequested();
    void autopilotToggled(bool enabled);

private slots:
    void onAileronChanged(int value);
    void onElevatorChanged(int value);
    void onRudderChanged(int value);
    void onThrottleChanged(int value);
    void onResetClicked();
    void onAutopilotToggled(bool checked);
    void centerControls();

private:
    QSlider *aileron_slider_;
    QSlider *elevator_slider_;
    QSlider *rudder_slider_;
    QSlider *throttle_slider_;

    QLabel *aileron_label_;
    QLabel *elevator_label_;
    QLabel *rudder_label_;
    QLabel *throttle_label_;

    QCheckBox *autopilot_checkbox_;
    QPushButton *reset_button_;
    QPushButton *center_button_;

    double aileron_value_ = 0.0;
    double elevator_value_ = 0.05;
    double rudder_value_ = 0.0;
    double throttle_value_ = 0.7;
    bool autopilot_enabled_ = false;

    QSlider *createSlider(int min, int max, int value);
    QGroupBox *createControlGroup(const QString &title, QSlider *slider,
                                  QLabel *label, const QString &units);
};