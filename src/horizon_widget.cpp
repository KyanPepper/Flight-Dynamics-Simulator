#include "horizon_widget.h"

HorizonWidget::HorizonWidget(QWidget *parent) : QWidget(parent)
{
    setMinimumSize(420, 420); // Set minimum window size
}

// Called by main thread to update display data
void HorizonWidget::setUiState(const UiState &u)
{
    ui_ = u;
    update();
}

// Qt calls this whenever the widget needs repainting
void HorizonWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true); // Smooth lines

    int w = width(), h = height();

    // Move origin to center of widget
    p.translate(w / 2.0, h / 2.0);

    // === Draw background (dark sky) ===
    p.fillRect(-w, -h, w * 2, h * 2, QColor(20, 20, 25));

    // === Draw rotating horizon ===
    p.save(); // Save current transform

    // Rotate by roll angle (opposite direction for correct visual)
    p.rotate(-ui_.roll * 180.0 / M_PI); // Convert radians to degrees

    // Move up/down based on pitch
    p.translate(0, ui_.pitch * 180.0 / M_PI * 2.0);

    // Sky (blue) - upper half
    p.fillRect(-w, -h, w * 2, h, QColor(70, 130, 180));

    // Ground (brown) - lower half
    p.fillRect(-w, 0, w * 2, h, QColor(139, 69, 19));

    // Horizon line (white)
    QPen pen(Qt::white, 3);
    p.setPen(pen);
    p.drawLine(-w, 0, w, 0);

    // Pitch ladder (shows angle markers)
    for (int ang = -60; ang <= 60; ang += 10)
    {
        int len = (ang % 30 == 0) ? 60 : 30; // Longer lines every 30°
        int y = -ang * 2;                    // Scale for visibility
        p.drawLine(-len, y, len, y);
    }

    p.restore(); // Restore transform

    // === Draw fixed aircraft symbol (stays centered) ===
    QPen pen2(Qt::white, 4);
    p.setPen(pen2);
    p.drawLine(-40, 0, 40, 0); // Wings
    p.drawLine(0, -8, 0, 8);   // Body
}
HorizonWidget::~HorizonWidget()
{
    // No special cleanup needed
}