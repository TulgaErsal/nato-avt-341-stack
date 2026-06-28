#include <avt_341_rviz_plugins/primitives/vector_field.h>

#include <algorithm>

#include <QHBoxLayout>

namespace avt_341::rviz_plugins
{

VectorField::VectorField( const QString& label, int num_entries,
                          const QStringList& axis_labels, const QStringList& axis_tooltips,
                          const QList<QColor>& axis_colors, QWidget* parent )
    : QWidget( parent )
{
    // Clamp the number of entries to the supported range [1, 3].
    num_entries = std::max( 1, std::min( num_entries, 3 ) );

    QHBoxLayout* layout = new QHBoxLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );

    // "<Label>:" — kept tight so a column of fields can share a label width.
    label_ = new QLabel( label + ":" );
    layout->addWidget( label_ );

    // One colored axis box + read-only value field per entry. The value fields
    // expand by default, so the value row fills the remaining width.
    for ( int i = 0; i < num_entries; ++i )
    {
        const QString axis = i < axis_labels.size() ? axis_labels.at( i ) : QString();
        const QColor color = i < axis_colors.size() ? axis_colors.at( i ) : QColor( Qt::gray );
        const QString tooltip = i < axis_tooltips.size() ? axis_tooltips.at( i ) : QString();

        QLabel* axis_box = new QLabel( axis );
        axis_box->setAlignment( Qt::AlignCenter );
        axis_box->setToolTip( tooltip );
        // Dark text on light backgrounds, white on dark, so the axis tag stays
        // readable whatever the box color is.
        const double luminance =
            0.299 * color.red() + 0.587 * color.green() + 0.114 * color.blue();
        const QString text_color = luminance > 150.0 ? QStringLiteral( "#333333" )
                                                      : QStringLiteral( "white" );
        axis_box->setStyleSheet(
            QString( "background-color: %1; color: %2; font-weight: bold; "
                     "padding: 2px 6px; border-radius: 2px;" )
                .arg( color.name(), text_color ) );

        QLineEdit* value_edit = new QLineEdit( "0.00" );
        value_edit->setReadOnly( true );
        value_edit->setAlignment( Qt::AlignRight );
        value_edit->setToolTip( tooltip );
        value_edits_.append( value_edit );

        // Keep each colored box flush against its value field; the spacing
        // between entries (and after the label) still comes from the outer layout.
        QHBoxLayout* entry_layout = new QHBoxLayout;
        entry_layout->setContentsMargins( 0, 0, 0, 0 );
        entry_layout->setSpacing( 0 );
        entry_layout->addWidget( axis_box );
        entry_layout->addWidget( value_edit );
        layout->addLayout( entry_layout );
    }

    setLayout( layout );
}

void VectorField::setValues( const QVector<double>& values )
{
    const int n = std::min( values.size(), value_edits_.size() );
    for ( int i = 0; i < n; ++i )
    {
        value_edits_[i]->setText( QString::number( values[i], 'f', 2 ) );
    }
}

void VectorField::setLabelWidth( int width )
{
    label_->setFixedWidth( width );
}

int VectorField::labelWidthHint() const
{
    return label_->sizeHint().width();
}

void VectorField::alignLabels( const QList<VectorField*>& fields )
{
    int max_width = 0;
    for ( VectorField* field : fields )
    {
        max_width = std::max( max_width, field->labelWidthHint() );
    }
    for ( VectorField* field : fields )
    {
        field->setLabelWidth( max_width );
    }
}

}
