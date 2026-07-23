#ifndef VECTOR_FIELD_H
#define VECTOR_FIELD_H

#ifndef Q_MOC_RUN
#include <QColor>
#include <QLabel>
#include <QLineEdit>
#include <QList>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QWidget>
#endif

namespace avt_341 {
namespace rviz_plugins {

/// A single labelled vector display laid out as
///   "<Label>: [x] <n> [y] <n> [z] <n>"
/// where the axis tags are colored boxes (red/green/blue by default) and the
/// values are read-only fields updated via setValues(). Re-usable for any 1-3
/// component vector (pose, velocity, etc.).
///
/// To line up a column of fields (so every "<Label>:" shares one minimum width
/// and the value rows align), build them and pass them to alignLabels().
class VectorField: public QWidget
{

Q_OBJECT
public:
    /// \param label         Text shown before the values (a ":" is appended).
    /// \param num_entries   Number of vector components to show, clamped to 1-3.
    /// \param axis_labels   Text inside each colored box (defaults to x, y, z).
    /// \param axis_tooltips Tooltip shown over each box and its value field.
    /// \param axis_colors   Background color of each box (defaults to light gray).
    VectorField( const QString& label,
                 int num_entries = 3,
                 const QStringList& axis_labels = { "x", "y", "z" },
                 const QStringList& axis_tooltips = { "position x", "position y", "position z" },
                 const QList<QColor>& axis_colors = { QColor( 220, 220, 220 ),
                                                      QColor( 220, 220, 220 ),
                                                      QColor( 220, 220, 220 ) },
                 QWidget* parent = nullptr );

    /// Update the displayed values (one per entry; extra values are ignored).
    void setValues( const QVector<double>& values );

    /// Force a fixed width on this field's label so a column of fields can
    /// share a common label width (see alignLabels()).
    void setLabelWidth( int width );

    /// Natural width the label would like.
    int labelWidthHint() const;

    /// Set every field's label to the widest label's width, so the value
    /// columns line up across rows.
    static void alignLabels( const QList<VectorField*>& fields );

protected:
    // QT Widgets
    QLabel* label_;
    QVector<QLineEdit*> value_edits_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // VECTOR_FIELD_H
