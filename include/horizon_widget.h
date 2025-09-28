#pragma once
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <cmath>

struct UiState
{
    double roll = 0;  // radians
    double pitch = 0; // radians
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
    UiState ui_;
};
