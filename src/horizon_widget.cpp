#include "horizon_widget.h"
#include <QFontMetrics>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

HorizonWidget::HorizonWidget(QWidget *parent) : QWidget(parent)
{
    setMinimumSize(1200, 800);
    setStyleSheet("background-color: #1a1a1a;");
}

void HorizonWidget::setUiState(const UiState &u)
{
    ui_ = u;

    animated_roll_ = 0.8 * animated_roll_ + 0.2 * ui_.roll;
    animated_pitch_ = 0.8 * animated_pitch_ + 0.2 * ui_.pitch;
    animated_heading_ = 0.8 * animated_heading_ + 0.2 * ui_.yaw;

    flight_path_.push_back(QPointF(ui_.east_pos, ui_.north_pos));
    if (flight_path_.size() > 500)
        flight_path_.pop_front();

    altitude_history_.push_back(ui_.altitude);
    if (altitude_history_.size() > 100)
        altitude_history_.pop_front();

    speed_history_.push_back(ui_.total_velocity);
    if (speed_history_.size() > 100)
        speed_history_.pop_front();

    update();
}

void HorizonWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    int w = width(), h = height();

    int centerX = w / 2;
    int centerY = h / 2;
    int mainSize = std::min(w, h) * 0.35;
    int smallSize = mainSize * 0.5;
    int tinySize = mainSize * 0.3;

    QRect horizonRect(centerX - mainSize / 2, centerY - mainSize / 2, mainSize, mainSize);
    drawArtificialHorizon(p, horizonRect);

    QRect airspeedRect(50, 50, smallSize, smallSize);
    drawAirspeedIndicator(p, airspeedRect);

    QRect vsiRect(50, 70 + smallSize, smallSize, smallSize);
    drawVerticalSpeedIndicator(p, vsiRect);

    QRect gmeterRect(50, 90 + smallSize * 2, smallSize, tinySize);
    drawGMeter(p, gmeterRect);

    QRect altimeterRect(w - 50 - smallSize, 50, smallSize, smallSize);
    drawAltimeter(p, altimeterRect);

    QRect headingRect(w - 50 - smallSize, 70 + smallSize, smallSize, smallSize);
    drawHeadingIndicator(p, headingRect);

    QRect turnRect(w - 50 - smallSize, 90 + smallSize * 2, smallSize, tinySize);
    drawTurnCoordinator(p, turnRect);

    QRect controlsRect(centerX - 250, h - 150, 200, 120);
    drawControlPositions(p, controlsRect);

    QRect throttleRect(centerX - 25, h - 150, 50, 120);
    drawThrottleGauge(p, throttleRect);

    QRect anglesRect(centerX + 50, h - 150, 200, 120);
    drawAngleIndicators(p, anglesRect);

    QRect mapRect(centerX - 150, 10, 300, 150);
    drawMiniMap(p, mapRect);

    QRect dataRect(10, h - 200, 300, 180);
    drawDataPanel(p, dataRect);
}

void HorizonWidget::drawArtificialHorizon(QPainter &p, const QRect &rect)
{
    p.save();
    p.fillRect(rect, QColor(30, 30, 35));
    p.setClipRect(rect);

    QPoint center = rect.center();
    p.translate(center);
    p.rotate(-animated_roll_ * 180.0 / M_PI);

    double pixelsPerDegree = rect.height() / 60.0;
    double pitchOffset = animated_pitch_ * 180.0 / M_PI * pixelsPerDegree;
    p.translate(0, pitchOffset);

    QLinearGradient skyGradient(0, -rect.height(), 0, 0);
    skyGradient.setColorAt(0, QColor(0, 50, 120));
    skyGradient.setColorAt(0.5, QColor(70, 130, 180));
    skyGradient.setColorAt(1, QColor(135, 206, 235));
    p.fillRect(-rect.width(), -rect.height(), rect.width() * 2, rect.height(), skyGradient);

    QLinearGradient groundGradient(0, 0, 0, rect.height());
    groundGradient.setColorAt(0, QColor(101, 67, 33));
    groundGradient.setColorAt(0.5, QColor(139, 90, 43));
    groundGradient.setColorAt(1, QColor(160, 82, 45));
    p.fillRect(-rect.width(), 0, rect.width() * 2, rect.height(), groundGradient);

    QPen horizonPen(Qt::white, 3);
    p.setPen(horizonPen);
    p.drawLine(-rect.width() / 2, 0, rect.width() / 2, 0);

    p.setFont(QFont("Arial", 10));
    for (int angle = -90; angle <= 90; angle += 10)
    {
        if (angle == 0)
            continue;
        int y = -angle * pixelsPerDegree;
        int lineLength = (angle % 30 == 0) ? 80 : 40;

        if (angle > 0)
            p.setPen(QPen(Qt::white, 2));
        else
            p.setPen(QPen(Qt::white, 1, Qt::DashLine));

        p.drawLine(-lineLength / 2, y, lineLength / 2, y);

        if (angle % 20 == 0)
        {
            p.setPen(Qt::white);
            QString label = QString::number(std::abs(angle));
            p.drawText(-lineLength / 2 - 25, y + 5, label);
            p.drawText(lineLength / 2 + 10, y + 5, label);
        }
    }

    p.restore();

    p.save();
    p.translate(center);
    QPen aircraftPen(Qt::yellow, 4);
    p.setPen(aircraftPen);
    p.drawLine(-60, 0, -20, 0);
    p.drawLine(20, 0, 60, 0);
    p.drawLine(0, -10, 0, 10);
    p.setBrush(Qt::yellow);
    p.drawEllipse(QPoint(0, 0), 4, 4);
    p.restore();

    p.save();
    p.translate(center.x(), rect.top() + 30);
    QPen rollPen(Qt::white, 2);
    p.setPen(rollPen);
    int rollRadius = rect.width() * 0.35;
    p.drawArc(-rollRadius, -rollRadius, rollRadius * 2, rollRadius * 2, 30 * 16, 120 * 16);

    for (int angle = -60; angle <= 60; angle += 30)
    {
        p.save();
        p.rotate(angle);
        p.drawLine(0, -rollRadius, 0, -rollRadius + 10);
        p.restore();
    }

    p.save();
    p.rotate(-animated_roll_ * 180.0 / M_PI);
    QPen pointerPen(Qt::yellow, 3);
    p.setPen(pointerPen);
    p.drawLine(0, -rollRadius + 15, -8, -rollRadius + 25);
    p.drawLine(0, -rollRadius + 15, 8, -rollRadius + 25);
    p.restore();
    p.restore();
}

void HorizonWidget::drawAirspeedIndicator(QPainter &p, const QRect &rect)
{
    drawCircularGauge(p, rect, ui_.total_velocity * 1.94384, 0, 200,
                      "AIRSPEED", "KTS", QColor(0, 255, 0));

    if (speed_history_.size() > 10)
    {
        double trend = speed_history_.back() - speed_history_[speed_history_.size() - 10];
        p.save();
        p.translate(rect.center());
        if (std::abs(trend) > 0.5)
        {
            QPen trendPen(trend > 0 ? Qt::green : Qt::red, 2);
            p.setPen(trendPen);
            int arrow = trend > 0 ? -1 : 1;
            p.drawLine(0, 20, 0, 20 + arrow * 10);
            p.drawLine(0, 20 + arrow * 10, -3, 20 + arrow * 7);
            p.drawLine(0, 20 + arrow * 10, 3, 20 + arrow * 7);
        }
        p.restore();
    }
}

void HorizonWidget::drawAltimeter(QPainter &p, const QRect &rect)
{
    drawCircularGauge(p, rect, ui_.altitude * 3.28084, 0, 10000,
                      "ALTITUDE", "FT", QColor(0, 150, 255));

    if (altitude_history_.size() > 10)
    {
        p.save();
        p.translate(rect.left(), rect.bottom() + 20);
        p.setPen(QPen(QColor(0, 150, 255), 2));
        p.setFont(QFont("Arial", 9));
        double trend = (altitude_history_.back() - altitude_history_[altitude_history_.size() - 10]) * 10;
        QString trendText = QString("Trend: %1 ft/s").arg(trend * 3.28084, 0, 'f', 1);
        p.drawText(0, 0, trendText);
        p.restore();
    }
}

void HorizonWidget::drawHeadingIndicator(QPainter &p, const QRect &rect)
{
    p.save();
    p.fillRect(rect, QColor(40, 40, 45));
    p.setPen(QPen(Qt::white, 2));
    p.drawRect(rect);

    QPoint center = rect.center();
    p.translate(center);
    int radius = rect.width() * 0.4;

    p.save();
    p.rotate(-animated_heading_ * 180.0 / M_PI);
    p.setFont(QFont("Arial", 12, QFont::Bold));
    p.setPen(Qt::white);
    p.drawText(-5, -radius + 20, "N");
    p.drawText(radius - 20, 5, "E");
    p.drawText(-5, radius - 10, "S");
    p.drawText(-radius + 10, 5, "W");

    p.setFont(QFont("Arial", 9));
    for (int deg = 0; deg < 360; deg += 30)
    {
        p.save();
        p.rotate(deg);
        p.drawLine(0, -radius, 0, -radius + 10);
        if (deg % 90 != 0)
        {
            p.translate(0, -radius + 25);
            p.rotate(-deg);
            p.drawText(-10, 0, QString::number(deg));
        }
        p.restore();
    }

    p.setPen(QPen(Qt::gray, 1));
    for (int deg = 0; deg < 360; deg += 10)
    {
        if (deg % 30 != 0)
        {
            p.save();
            p.rotate(deg);
            p.drawLine(0, -radius, 0, -radius + 5);
            p.restore();
        }
    }
    p.restore();

    QPen aircraftPen(Qt::yellow, 3);
    p.setPen(aircraftPen);
    p.drawLine(0, 0, 0, -radius * 0.7);
    p.drawLine(-10, 10, 0, 0);
    p.drawLine(10, 10, 0, 0);

    p.setPen(Qt::white);
    p.setFont(QFont("Arial", 11, QFont::Bold));
    int heading = (int)(animated_heading_ * 180.0 / M_PI) % 360;
    if (heading < 0)
        heading += 360;
    QString hdgText = QString("HDG %1°").arg(heading, 3, 10, QChar('0'));
    p.drawText(-30, -radius - 20, hdgText);
    p.restore();
}

void HorizonWidget::drawVerticalSpeedIndicator(QPainter &p, const QRect &rect)
{
    double vsi_fpm = ui_.vertical_velocity * 196.85;
    drawCircularGauge(p, rect, vsi_fpm, -2000, 2000, "VSI", "FPM",
                      vsi_fpm > 100 ? QColor(0, 255, 0) : (vsi_fpm < -100 ? QColor(255, 0, 0) : QColor(255, 255, 0)));
}

void HorizonWidget::drawTurnCoordinator(QPainter &p, const QRect &rect)
{
    p.save();
    p.fillRect(rect, QColor(40, 40, 45));
    p.setPen(QPen(Qt::white, 2));
    p.drawRect(rect);

    QPoint center = rect.center();
    p.translate(center);

    double turnRate = ui_.yaw_rate;
    p.rotate(-turnRate * 20);

    QPen planePen(Qt::white, 2);
    p.setPen(planePen);
    p.drawLine(-30, 0, 30, 0);
    p.drawLine(0, -10, 0, 10);

    p.resetTransform();
    p.translate(center);

    p.setPen(QPen(Qt::gray, 1));
    p.drawLine(-20, 20, -20, 25);
    p.drawLine(20, 20, 20, 25);

    double slip = ui_.side_velocity * 2;
    int ballX = std::max(-30.0, std::min(30.0, slip * 10));

    p.setPen(QPen(Qt::white, 2));
    p.drawLine(-35, 35, 35, 35);
    p.setBrush(Qt::white);
    p.drawEllipse(QPoint(ballX, 35), 5, 5);

    p.setFont(QFont("Arial", 9));
    p.drawText(-50, -35, "TURN COORD");
    p.restore();
}

void HorizonWidget::drawControlPositions(QPainter &p, const QRect &rect)
{
    p.save();
    p.fillRect(rect, QColor(40, 40, 45));
    p.setPen(QPen(Qt::white, 2));
    p.drawRect(rect);
    p.translate(rect.center());

    p.setFont(QFont("Arial", 10, QFont::Bold));
    p.setPen(Qt::white);
    p.drawText(-40, -50, "CONTROLS");

    int boxSize = 60;
    p.drawRect(-boxSize / 2, -boxSize / 2 + 10, boxSize, boxSize);

    p.setPen(QPen(Qt::gray, 1, Qt::DashLine));
    p.drawLine(-boxSize / 2, 10, boxSize / 2, 10);
    p.drawLine(0, -boxSize / 2 + 10, 0, boxSize / 2 + 10);

    int stickX = ui_.aileron * boxSize / 2;
    int stickY = -ui_.elevator * boxSize / 2;
    p.setBrush(QColor(255, 100, 0));
    p.setPen(QPen(QColor(255, 100, 0), 2));
    p.drawEllipse(QPoint(stickX, stickY + 10), 5, 5);

    p.setPen(QPen(Qt::white, 2));
    p.drawRect(-40, 50, 80, 10);
    int rudderPos = ui_.rudder * 40;
    p.fillRect(rudderPos - 5, 50, 10, 10, QColor(255, 100, 0));

    p.setFont(QFont("Arial", 8));
    p.setPen(Qt::gray);
    p.drawText(-50, -20, "AIL");
    p.drawText(35, -20, QString::number(ui_.aileron, 'f', 2));
    p.drawText(-50, 0, "ELE");
    p.drawText(35, 0, QString::number(ui_.elevator, 'f', 2));
    p.drawText(-50, 48, "RUD");
    p.drawText(35, 48, QString::number(ui_.rudder, 'f', 2));
    p.restore();
}

void HorizonWidget::drawThrottleGauge(QPainter &p, const QRect &rect)
{
    p.save();
    p.fillRect(rect, QColor(40, 40, 45));
    p.setPen(QPen(Qt::white, 2));
    p.drawRect(rect);

    int barHeight = rect.height() - 40;
    int barTop = rect.top() + 30;
    int throttleHeight = ui_.throttle * barHeight;

    p.fillRect(rect.left() + 10, barTop, 30, barHeight, QColor(20, 20, 20));

    QLinearGradient throttleGrad(0, 0, 0, barHeight);
    throttleGrad.setColorAt(0, Qt::red);
    throttleGrad.setColorAt(0.5, Qt::yellow);
    throttleGrad.setColorAt(1, Qt::green);
    p.fillRect(rect.left() + 10, barTop + barHeight - throttleHeight,
               30, throttleHeight, throttleGrad);

    p.setPen(Qt::white);
    p.setFont(QFont("Arial", 8));
    for (int pct = 0; pct <= 100; pct += 25)
    {
        int y = barTop + barHeight - (pct * barHeight / 100);
        p.drawLine(rect.left() + 5, y, rect.left() + 10, y);
        p.drawText(rect.left() + 42, y + 3, QString::number(pct));
    }

    p.setFont(QFont("Arial", 9, QFont::Bold));
    p.drawText(rect.left() + 5, rect.top() + 15, "THR");
    p.drawText(rect.left() + 5, rect.bottom() - 5,
               QString("%1%").arg((int)(ui_.throttle * 100)));
    p.restore();
}

void HorizonWidget::drawAngleIndicators(QPainter &p, const QRect &rect)
{
    p.save();
    p.fillRect(rect, QColor(40, 40, 45));
    p.setPen(QPen(Qt::white, 2));
    p.drawRect(rect);
    p.translate(rect.left() + rect.width() / 2, rect.top() + rect.height() / 2);

    p.setFont(QFont("Arial", 10, QFont::Bold));
    p.setPen(Qt::white);
    p.drawText(-40, -50, "ANGLES");

    p.setFont(QFont("Arial", 9));
    p.drawText(-80, -20, "AoA:");

    double aoa_deg = ui_.angle_of_attack * 180.0 / M_PI;
    QColor aoaColor = (aoa_deg > 15) ? Qt::red : (aoa_deg > 10) ? Qt::yellow
                                                                : Qt::green;
    p.setPen(aoaColor);
    p.drawText(20, -20, QString("%1°").arg(aoa_deg, 0, 'f', 1));

    p.fillRect(-30, -10, 60, 8, QColor(20, 20, 20));
    int aoaBar = std::max(-30, std::min(30, (int)(aoa_deg * 2)));
    p.fillRect(0, -10, aoaBar, 8, aoaColor);

    p.setPen(Qt::white);
    p.drawText(-80, 10, "Slip:");

    double slip_deg = ui_.sideslip_angle * 180.0 / M_PI;
    QColor slipColor = (std::abs(slip_deg) > 10) ? Qt::yellow : Qt::green;
    p.setPen(slipColor);
    p.drawText(20, 10, QString("%1°").arg(slip_deg, 0, 'f', 1));

    p.fillRect(-30, 20, 60, 8, QColor(20, 20, 20));
    int slipBar = std::max(-30, std::min(30, (int)(slip_deg * 2)));
    p.fillRect(0, 20, slipBar, 8, slipColor);

    p.setPen(Qt::white);
    p.drawText(-80, 40, "G:");
    QColor gColor = (ui_.g_force > 4 || ui_.g_force < -1) ? Qt::red : (ui_.g_force > 2 || ui_.g_force < 0) ? Qt::yellow
                                                                                                           : Qt::green;
    p.setPen(gColor);
    p.drawText(20, 40, QString("%1g").arg(ui_.g_force, 0, 'f', 2));
    p.restore();
}

void HorizonWidget::drawMiniMap(QPainter &p, const QRect &rect)
{
    p.save();
    p.fillRect(rect, QColor(20, 20, 25));
    p.setPen(QPen(QColor(100, 100, 100), 1));
    p.drawRect(rect);

    p.setFont(QFont("Arial", 10, QFont::Bold));
    p.setPen(Qt::white);
    p.drawText(rect.left() + 5, rect.top() + 15, "FLIGHT PATH");

    if (flight_path_.size() > 1)
    {
        double minX = flight_path_[0].x(), maxX = minX;
        double minY = flight_path_[0].y(), maxY = minY;

        for (const auto &pt : flight_path_)
        {
            minX = std::min(minX, pt.x());
            maxX = std::max(maxX, pt.x());
            minY = std::min(minY, pt.y());
            maxY = std::max(maxY, pt.y());
        }

        // Add padding - MUCH LARGER SCALE
        double rangeX = std::max(1000.0, maxX - minX); // Minimum 1000m range
        double rangeY = std::max(1000.0, maxY - minY); // Minimum 1000m range

        p.setPen(QPen(QColor(60, 60, 60), 1));
        for (int i = 0; i <= 4; i++)
        {
            int x = rect.left() + 10 + i * (rect.width() - 20) / 4;
            int y = rect.top() + 20 + i * (rect.height() - 30) / 4;
            p.drawLine(rect.left() + 10, y, rect.right() - 10, y);
            p.drawLine(x, rect.top() + 20, x, rect.bottom() - 10);
        }

        QColor trailColor(0, 255, 100);
        for (size_t i = 1; i < flight_path_.size(); i++)
        {
            double alpha = (double)i / flight_path_.size();
            trailColor.setAlphaF(alpha * 0.8);
            p.setPen(QPen(trailColor, 2));

            double x1 = (flight_path_[i - 1].x() - minX) / rangeX;
            double y1 = (flight_path_[i - 1].y() - minY) / rangeY;
            double x2 = (flight_path_[i].x() - minX) / rangeX;
            double y2 = (flight_path_[i].y() - minY) / rangeY;

            int px1 = rect.left() + 10 + x1 * (rect.width() - 20);
            int py1 = rect.bottom() - 10 - y1 * (rect.height() - 30);
            int px2 = rect.left() + 10 + x2 * (rect.width() - 20);
            int py2 = rect.bottom() - 10 - y2 * (rect.height() - 30);

            p.drawLine(px1, py1, px2, py2);
        }

        if (!flight_path_.empty())
        {
            double x = (flight_path_.back().x() - minX) / rangeX;
            double y = (flight_path_.back().y() - minY) / rangeY;
            int px = rect.left() + 10 + x * (rect.width() - 20);
            int py = rect.bottom() - 10 - y * (rect.height() - 30);

            p.save();
            p.translate(px, py);
            p.rotate(-ui_.yaw * 180.0 / M_PI);
            p.setPen(QPen(Qt::yellow, 3));
            p.drawLine(0, -8, 0, 4);
            p.drawLine(-6, 0, 6, 0);
            p.drawLine(-3, 4, 3, 4);
            p.restore();
        }

        p.setPen(Qt::white);
        p.setFont(QFont("Arial", 8));
        QString scale = QString("Scale: %1m").arg((int)rangeX);
        p.drawText(rect.right() - 80, rect.bottom() - 5, scale);
    }
    p.restore();
}

void HorizonWidget::drawGMeter(QPainter &p, const QRect &rect)
{
    p.save();
    p.fillRect(rect, QColor(40, 40, 45));
    p.setPen(QPen(Qt::white, 2));
    p.drawRect(rect);
    p.translate(rect.center());

    p.setFont(QFont("Arial", 10, QFont::Bold));
    p.setPen(Qt::white);
    p.drawText(-20, -rect.height() / 2 + 15, "G-METER");

    int barWidth = rect.width() - 40;
    int barHeight = 20;
    p.fillRect(-barWidth / 2, -10, barWidth, barHeight, QColor(20, 20, 20));

    p.fillRect(-barWidth / 2, -10, barWidth / 6, barHeight, QColor(255, 0, 0, 100));
    p.fillRect(-barWidth / 3, -10, barWidth * 2 / 3, barHeight, QColor(0, 255, 0, 100));
    p.fillRect(barWidth / 3, -10, barWidth / 6, barHeight, QColor(255, 0, 0, 100));

    p.setPen(Qt::white);
    p.setFont(QFont("Arial", 7));
    for (int g = -2; g <= 6; g++)
    {
        int x = (g + 2) * barWidth / 8 - barWidth / 2;
        p.drawLine(x, -10, x, 10);
        p.drawText(x - 5, 25, QString::number(g));
    }

    double gPos = std::max(-2.0, std::min(6.0, ui_.g_force));
    int pointerX = (gPos + 2) * barWidth / 8 - barWidth / 2;

    QColor gColor = (ui_.g_force > 4 || ui_.g_force < -1) ? Qt::red : (ui_.g_force > 2 || ui_.g_force < 0) ? Qt::yellow
                                                                                                           : Qt::green;
    p.setPen(QPen(gColor, 3));
    p.setBrush(gColor);

    QPolygon pointer;
    pointer << QPoint(pointerX, -15) << QPoint(pointerX - 5, -20) << QPoint(pointerX + 5, -20);
    p.drawPolygon(pointer);

    p.drawText(-15, 45, QString("%1g").arg(ui_.g_force, 0, 'f', 2));
    p.restore();
}

void HorizonWidget::drawDataPanel(QPainter &p, const QRect &rect)
{
    p.save();
    p.fillRect(rect, QColor(30, 30, 35));
    p.setPen(QPen(QColor(100, 100, 100), 1));
    p.drawRect(rect);

    p.setFont(QFont("Arial", 11, QFont::Bold));
    p.setPen(Qt::white);
    p.drawText(rect.left() + 10, rect.top() + 20, "FLIGHT DATA");

    int y = rect.top() + 40;
    int lineHeight = 15;

    auto drawDataRow = [&](const QString &label, const QString &value, const QColor &color = Qt::white)
    {
        p.setPen(QColor(150, 150, 150));
        p.drawText(rect.left() + 10, y, label);
        p.setPen(color);
        p.drawText(rect.left() + 130, y, value);
        y += lineHeight;
    };

    double tas = ui_.total_velocity * 1.94384;
    double alt_ft = ui_.altitude * 3.28084;
    double climb_rate = ui_.vertical_velocity * 196.85;

    drawDataRow("Time:", QString("%1 s").arg(ui_.time, 0, 'f', 1));
    drawDataRow("Airspeed:", QString("%1 kts").arg(tas, 0, 'f', 1),
                tas < 40 ? Qt::red : tas < 50 ? Qt::yellow
                                              : Qt::green);
    drawDataRow("Altitude:", QString("%1 ft").arg(alt_ft, 0, 'f', 0),
                alt_ft < 100 ? Qt::yellow : Qt::green);
    drawDataRow("Climb Rate:", QString("%1 fpm").arg(climb_rate, 0, 'f', 0),
                climb_rate > 500 ? Qt::green : climb_rate < -500 ? Qt::red
                                                                 : Qt::white);
    drawDataRow("Heading:", QString("%1°").arg(((int)(ui_.yaw * 180.0 / M_PI) % 360 + 360) % 360, 3, 10, QChar('0')));
    drawDataRow("Bank Angle:", QString("%1°").arg(ui_.roll * 180.0 / M_PI, 0, 'f', 1),
                std::abs(ui_.roll) > 1.0 ? Qt::yellow : Qt::white);
    drawDataRow("Pitch:", QString("%1°").arg(ui_.pitch * 180.0 / M_PI, 0, 'f', 1));
    drawDataRow("AoA:", QString("%1°").arg(ui_.angle_of_attack * 180.0 / M_PI, 0, 'f', 1),
                ui_.angle_of_attack > 0.26 ? Qt::red : ui_.angle_of_attack > 0.17 ? Qt::yellow
                                                                                  : Qt::green);
    drawDataRow("Sideslip:", QString("%1°").arg(ui_.sideslip_angle * 180.0 / M_PI, 0, 'f', 1),
                std::abs(ui_.sideslip_angle) > 0.17 ? Qt::yellow : Qt::white);
    drawDataRow("G-Force:", QString("%1 g").arg(ui_.g_force, 0, 'f', 2),
                ui_.g_force > 3 || ui_.g_force < -0.5 ? Qt::yellow : Qt::white);

    p.restore();
}

void HorizonWidget::drawCircularGauge(QPainter &p, const QRect &rect,
                                      double value, double min, double max,
                                      const QString &label, const QString &unit,
                                      const QColor &color)
{
    p.save();

    p.fillRect(rect, QColor(40, 40, 45));
    p.setPen(QPen(Qt::white, 2));
    p.drawRect(rect);

    QPoint center = rect.center();
    p.translate(center);

    int radius = rect.width() * 0.35;

    p.setPen(QPen(QColor(60, 60, 60), 8));
    p.drawArc(-radius, -radius, radius * 2, radius * 2, 225 * 16, -270 * 16);

    p.setPen(QPen(Qt::white, 2));
    for (int i = 0; i <= 10; i++)
    {
        double angle = 225 - i * 27;
        double rad = angle * M_PI / 180.0;

        int x1 = std::cos(rad) * (radius - 10);
        int y1 = -std::sin(rad) * (radius - 10);
        int x2 = std::cos(rad) * (radius - 5);
        int y2 = -std::sin(rad) * (radius - 5);

        p.drawLine(x1, y1, x2, y2);

        if (i % 2 == 0)
        {
            p.setFont(QFont("Arial", 8));
            double scaleValue = min + (max - min) * i / 10.0;
            QString text = QString::number((int)scaleValue);
            int textX = std::cos(rad) * (radius - 25) - 10;
            int textY = -std::sin(rad) * (radius - 25) + 5;
            p.drawText(textX, textY, text);
        }
    }

    double ratio = std::max(0.0, std::min(1.0, (value - min) / (max - min)));
    int arcLength = -ratio * 270 * 16;

    QPen arcPen(color, 6);
    p.setPen(arcPen);
    p.drawArc(-radius + 5, -radius + 5, (radius - 5) * 2, (radius - 5) * 2, 225 * 16, arcLength);

    double needleAngle = 225 - ratio * 270;
    double needleRad = needleAngle * M_PI / 180.0;

    p.setPen(QPen(Qt::white, 3));
    p.drawLine(0, 0,
               std::cos(needleRad) * (radius - 15),
               -std::sin(needleRad) * (radius - 15));

    p.setBrush(Qt::white);
    p.setPen(Qt::NoPen);
    p.drawEllipse(QPoint(0, 0), 5, 5);

    p.setPen(Qt::white);
    p.setFont(QFont("Arial", 10, QFont::Bold));
    p.drawText(-30, -radius - 10, label);

    p.setFont(QFont("Arial", 12, QFont::Bold));
    p.setPen(color);
    QString valueText = QString("%1 %2").arg(value, 0, 'f', 0).arg(unit);
    QFontMetrics fm(p.font());

    // FIXED: Use width() instead of horizontalAdvance() for Qt compatibility
#if QT_VERSION >= QT_VERSION_CHECK(5, 11, 0)
    int textWidth = fm.horizontalAdvance(valueText);
#else
    int textWidth = fm.width(valueText);
#endif

    p.drawText(-textWidth / 2, radius + 25, valueText);

    p.restore();
}

HorizonWidget::~HorizonWidget()
{
}