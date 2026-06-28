#include <avt_341_rviz_plugins/primitives/matrix_field.h>

#include <algorithm>

#include <QGridLayout>
#include <QHBoxLayout>

namespace
{

// The matrix is fixed at 3x3.
constexpr int kMatrixDimension = 3;
constexpr int kCellCount = kMatrixDimension * kMatrixDimension;

// Threshold colors (white text reads on all three).
const QColor kHighColor( 220, 53, 69 );    // red    (> high_threshold)
const QColor kMediumColor( 230, 126, 34 ); // orange (> medium_threshold)
const QColor kLowColor( 40, 167, 69 );     // green  (otherwise)

}  // namespace

namespace avt_341::rviz_plugins
{

MatrixField::MatrixField( const QString& label, double medium_threshold,
                          double high_threshold, QWidget* parent )
    : QWidget( parent ),
      medium_threshold_( medium_threshold ),
      high_threshold_( high_threshold )
{
    QHBoxLayout* layout = new QHBoxLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );

    // "<Label>:" before the matrix.
    label_ = new QLabel( label + ":" );
    layout->addWidget( label_ );

    // 3x3 grid of number cells, each colored by its value.
    QGridLayout* matrix_layout = new QGridLayout;
    matrix_layout->setContentsMargins( 0, 0, 0, 0 );
    matrix_layout->setSpacing( 3 );

    for ( int index = 0; index < kCellCount; ++index )
    {
        QLabel* cell = new QLabel();
        cell->setAlignment( Qt::AlignCenter );
        cell->setMinimumWidth( 44 );
        cells_.append( cell );
        matrix_layout->addWidget( cell, index / kMatrixDimension,
                                  index % kMatrixDimension );
        setCell( index, 0.0 );  // placeholder value + color
    }

    layout->addLayout( matrix_layout );
    layout->addStretch();  // keep the matrix compact (left-aligned)
    setLayout( layout );
}

void MatrixField::setValues( const QVector<double>& values )
{
    const int n = std::min( values.size(), cells_.size() );
    for ( int index = 0; index < n; ++index )
    {
        setCell( index, values[index] );
    }
}

void MatrixField::setLabelWidth( int width )
{
    label_->setFixedWidth( width );
}

int MatrixField::labelWidthHint() const
{
    return label_->sizeHint().width();
}

QColor MatrixField::colorForValue( double value ) const
{
    if ( value > high_threshold_ )
    {
        return kHighColor;
    }
    if ( value > medium_threshold_ )
    {
        return kMediumColor;
    }
    return kLowColor;
}

void MatrixField::setCell( int index, double value )
{
    QLabel* cell = cells_[index];
    cell->setText( QString::number( value ) );
    cell->setStyleSheet(
        QString( "background-color: %1; color: white; padding: 2px 6px; "
                 "border-radius: 2px;" )
            .arg( colorForValue( value ).name() ) );
}

}
