#ifndef MATRIX_FIELD_H
#define MATRIX_FIELD_H

#ifndef Q_MOC_RUN
#include <QColor>
#include <QLabel>
#include <QString>
#include <QVector>
#include <QWidget>
#endif

namespace avt_341 {
namespace rviz_plugins {

/// A labelled 3x3 matrix display laid out as "<Label>: <3x3 grid>", where each
/// cell shows a number whose background is colored by two thresholds:
///   value > high_threshold   -> red
///   value > medium_threshold -> orange
///   otherwise                -> green
/// Cell values are updated via setValues() (row-major, up to 9 entries).
class MatrixField: public QWidget
{

Q_OBJECT
public:
    /// \param label            Text shown before the matrix (a ":" is appended).
    /// \param medium_threshold Values above this color a cell orange.
    /// \param high_threshold   Values above this color a cell red.
    MatrixField( const QString& label, double medium_threshold, double high_threshold,
                 QWidget* parent = nullptr );

    /// Update the 3x3 cell values (row-major; extra/missing values are ignored).
    void setValues( const QVector<double>& values );

    /// Force a fixed width on this field's label so a column of fields can share
    /// a common label width (mirrors VectorField, for aligning values across
    /// rows of mixed field types).
    void setLabelWidth( int width );

    /// Natural width the label would like.
    int labelWidthHint() const;

protected:
    // Background color for a value according to the configured thresholds.
    QColor colorForValue( double value ) const;

    // Set a single cell's text and threshold color (index is row-major, 0..8).
    void setCell( int index, double value );

    double medium_threshold_;
    double high_threshold_;

    // QT Widgets
    QLabel* label_;
    QVector<QLabel*> cells_;

};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MATRIX_FIELD_H
