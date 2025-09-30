#include "control_panel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFont>

ControlPanel::ControlPanel(QWidget *parent) : QWidget(parent)
{
    setWindowTitle("Flight Controls");
    setMinimumWidth(400);

    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // Title
    QLabel *titleLabel = new QLabel("MANUAL FLIGHT CONTROLS");
    QFont titleFont = titleLabel->font();
    titleFont.setPointSize(14);
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    titleLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(titleLabel);

    // Create sliders (range -100 to 100 for controls, 0-100 for throttle)
    aileron_slider_ = createSlider(-100, 100, 0);
    elevator_slider_ = createSlider(-100, 100, 5); // Slight up
    rudder_slider_ = createSlider(-100, 100, 0);
    throttle_slider_ = createSlider(0, 100, 70); // 70% power

    // Create labels
    aileron_label_ = new QLabel("0.00");
    elevator_label_ = new QLabel("0.05");
    rudder_label_ = new QLabel("0.00");
    throttle_label_ = new QLabel("0.70");

    // Connect sliders
    connect(aileron_slider_, &QSlider::valueChanged, this, &ControlPanel::onAileronChanged);
    connect(elevator_slider_, &QSlider::valueChanged, this, &ControlPanel::onElevatorChanged);
    connect(rudder_slider_, &QSlider::valueChanged, this, &ControlPanel::onRudderChanged);
    connect(throttle_slider_, &QSlider::valueChanged, this, &ControlPanel::onThrottleChanged);

    // Add control groups
    mainLayout->addWidget(createControlGroup("Aileron (Roll)", aileron_slider_,
                                             aileron_label_, "[-1.0 = Left, +1.0 = Right]"));
    mainLayout->addWidget(createControlGroup("Elevator (Pitch)", elevator_slider_,
                                             elevator_label_, "[-1.0 = Nose Down, +1.0 = Nose Up]"));
    mainLayout->addWidget(createControlGroup("Rudder (Yaw)", rudder_slider_,
                                             rudder_label_, "[-1.0 = Left, +1.0 = Right]"));
    mainLayout->addWidget(createControlGroup("Throttle", throttle_slider_,
                                             throttle_label_, "[0.0 = Idle, 1.0 = Full Power]"));

    // Autopilot checkbox
    autopilot_checkbox_ = new QCheckBox("Enable Autopilot (Gentle Banking)");
    autopilot_checkbox_->setChecked(false);
    connect(autopilot_checkbox_, &QCheckBox::toggled, this, &ControlPanel::onAutopilotToggled);
    mainLayout->addWidget(autopilot_checkbox_);

    // Buttons
    QHBoxLayout *buttonLayout = new QHBoxLayout();

    center_button_ = new QPushButton("Center Controls");
    center_button_->setToolTip("Reset all controls to neutral/trim position");
    connect(center_button_, &QPushButton::clicked, this, &ControlPanel::centerControls);
    buttonLayout->addWidget(center_button_);

    reset_button_ = new QPushButton("Reset Aircraft");
    reset_button_->setToolTip("Reset aircraft to stable flight at 1000m altitude");
    reset_button_->setStyleSheet("QPushButton { background-color: #ff6666; color: white; font-weight: bold; }");
    connect(reset_button_, &QPushButton::clicked, this, &ControlPanel::onResetClicked);
    buttonLayout->addWidget(reset_button_);

    mainLayout->addLayout(buttonLayout);

    // Instructions
    QLabel *instructions = new QLabel(
        "<b>Instructions:</b><br>"
        "• Use sliders to control the aircraft<br>"
        "• Aileron: Roll left/right<br>"
        "• Elevator: Pitch up/down<br>"
        "• Rudder: Yaw left/right<br>"
        "• Throttle: Engine power<br>"
        "• Start with small inputs!<br>"
        "• Keep speed above 40 m/s to avoid stall");
    instructions->setWordWrap(true);
    instructions->setStyleSheet("QLabel { background-color: #f0f0f0; padding: 10px; border: 1px solid #ccc; }");
    mainLayout->addWidget(instructions);

    mainLayout->addStretch();
}

QSlider *ControlPanel::createSlider(int min, int max, int value)
{
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setMinimum(min);
    slider->setMaximum(max);
    slider->setValue(value);
    slider->setTickPosition(QSlider::TicksBelow);
    slider->setTickInterval((max - min) / 10);
    return slider;
}

QGroupBox *ControlPanel::createControlGroup(const QString &title, QSlider *slider,
                                            QLabel *label, const QString &units)
{
    QGroupBox *group = new QGroupBox(title);
    QVBoxLayout *layout = new QVBoxLayout();

    QHBoxLayout *valueLayout = new QHBoxLayout();
    QLabel *valueLabel = new QLabel("Value:");
    label->setMinimumWidth(60);
    label->setStyleSheet("QLabel { font-weight: bold; color: #0066cc; }");
    valueLayout->addWidget(valueLabel);
    valueLayout->addWidget(label);
    valueLayout->addStretch();

    layout->addLayout(valueLayout);
    layout->addWidget(slider);

    QLabel *unitsLabel = new QLabel(units);
    unitsLabel->setStyleSheet("QLabel { font-size: 9pt; color: #666; }");
    layout->addWidget(unitsLabel);

    group->setLayout(layout);
    return group;
}

void ControlPanel::onAileronChanged(int value)
{
    aileron_value_ = value / 100.0;
    aileron_label_->setText(QString::number(aileron_value_, 'f', 2));
    emit controlsChanged();
}

void ControlPanel::onElevatorChanged(int value)
{
    elevator_value_ = value / 100.0;
    elevator_label_->setText(QString::number(elevator_value_, 'f', 2));
    emit controlsChanged();
}

void ControlPanel::onRudderChanged(int value)
{
    rudder_value_ = value / 100.0;
    rudder_label_->setText(QString::number(rudder_value_, 'f', 2));
    emit controlsChanged();
}

void ControlPanel::onThrottleChanged(int value)
{
    throttle_value_ = value / 100.0;
    throttle_label_->setText(QString::number(throttle_value_, 'f', 2));
    emit controlsChanged();
}

void ControlPanel::centerControls()
{
    aileron_slider_->setValue(0);
    elevator_slider_->setValue(5); // Slight up trim
    rudder_slider_->setValue(0);
    throttle_slider_->setValue(70);
}

void ControlPanel::onResetClicked()
{
    emit resetRequested();
}

void ControlPanel::onAutopilotToggled(bool checked)
{
    autopilot_enabled_ = checked;

    // Disable manual controls when autopilot is on
    aileron_slider_->setEnabled(!checked);
    elevator_slider_->setEnabled(!checked);
    rudder_slider_->setEnabled(!checked);
    throttle_slider_->setEnabled(!checked);

    emit autopilotToggled(checked);
}