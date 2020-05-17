/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#include "rqt_plugins/qtpositionsliders.h"

// Qt core
#include <QTextStream>
#include <QMetaType>

// Qt gui
#include <QHeaderView>

namespace platform_sigma_plugins_ns {
	
	QtPositionSliders::QtPositionSliders( QWidget *parent) : QWidget(parent), slider_j0_(0), slider_j1_(0), slider_j2_(0), slider_j3_(0), slider_j4_(0), slider_j5_(0), slider_j6_(0),
	line_j0_(0), line_j1_(0), line_j2_(0), line_j3_(0), line_j4_(0), line_j5_(0), line_j6_(0), table_widget_global_(0), vlayout_global_(0)
	{
		createWidget_();
	}
	
	QtPositionSliders::~QtPositionSliders()
	{
		disconnect( slider_j0_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ0(double)) );
		disconnect( slider_j1_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ1(double)) );
		disconnect( slider_j2_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ2(double)) );
		disconnect( slider_j3_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ3(double)) );
		disconnect( slider_j4_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ4(double)) );
		disconnect( slider_j5_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ5(double)) );
		disconnect( slider_j6_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ6(double)) );
		
		disconnect( line_j0_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ0()) );
		disconnect( line_j1_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ1()) );
		disconnect( line_j2_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ2()) );
		disconnect( line_j3_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ3()) );
		disconnect( line_j4_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ4()) );
		disconnect( line_j5_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ5()) );
		disconnect( line_j6_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ6()) );
		
		// Deselects all selected items
		table_widget_global_->clearSelection();

		// Disconnect all signals from table widget ! important !
		table_widget_global_->disconnect();

		// Remove all items
		table_widget_global_->clearContents();

		// Set row count to 0 (remove rows)
		table_widget_global_->setRowCount(0);
		
		vlayout_global_->removeWidget(table_widget_global_);
		
		delete table_widget_global_;
		
		delete vlayout_global_;
		
	}
	
	void QtPositionSliders::createWidget_()
	{
		QSizePolicy fixed_policy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	
		vlayout_global_ = new QVBoxLayout();
		vlayout_global_->setObjectName("vertical_layout_global");
		
		table_widget_global_ = new QTableWidget();
		table_widget_global_->setObjectName("table_widget_global");
		table_widget_global_->setRowCount(7);
		table_widget_global_->setColumnCount(4);
		table_widget_global_->horizontalHeader()->setStretchLastSection(true);
		table_widget_global_->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
		
		//Set Header Label Texts Here
		table_widget_global_->setHorizontalHeaderLabels(QString("Joint name;Edit Position;Joint State;Position").split(";"));
		
		QString name_of_joint;
		QTextStream stream_name_of_joint(&name_of_joint);
		
		for (size_t i=0; i<7; i++)
		{
			stream_name_of_joint << "Joint" << i;
			table_widget_global_->setItem(i,0,new QTableWidgetItem(stream_name_of_joint.readAll()));
			table_widget_global_->item(i,0)->setFlags(Qt::ItemIsEnabled );
			table_widget_global_->setItem(i,2,new QTableWidgetItem("0"));
			table_widget_global_->item(i,2)->setFlags(Qt::ItemIsEnabled );
			
			stream_name_of_joint.flush();
		}
		
		slider_j0_ = new QwtSlider(this, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j0_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j0_->setValue( 0 );
		connect( slider_j0_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ0(double)) );
		
		QSize size_of_slider = slider_j0_->sizeHint();
		
		table_widget_global_->setCellWidget(0, 3, slider_j0_);
		
		slider_j1_ = new QwtSlider(this, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j1_->setRange((-120 * M_PI / 180), (120 * M_PI / 180), 0.1, 1);
		slider_j1_->setValue( 0 );
		connect( slider_j1_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ1(double)) );
		
		table_widget_global_->setCellWidget(1, 3, slider_j1_);
		
		slider_j2_ = new QwtSlider(this, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j2_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j2_->setValue( 0 );
		connect( slider_j2_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ2(double)) );
		
		table_widget_global_->setCellWidget(2, 3, slider_j2_);
		
		slider_j3_ = new QwtSlider(this, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j3_->setRange((-120 * M_PI / 180), (120 * M_PI / 180), 0.1, 1);
		slider_j3_->setValue( 0 );
		connect( slider_j3_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ3(double)) );
		
		table_widget_global_->setCellWidget(3, 3, slider_j3_);
		
		slider_j4_ = new QwtSlider(this, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j4_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j4_->setValue( 0 );
		connect( slider_j4_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ4(double)) );
		
		table_widget_global_->setCellWidget(4, 3, slider_j4_);
		
		slider_j5_ = new QwtSlider(this, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j5_->setRange((-120 * M_PI / 180), (120 * M_PI / 180), 0.1, 1);
		slider_j5_->setValue( 0 );
		connect( slider_j5_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ5(double)) );
		
		table_widget_global_->setCellWidget(5, 3, slider_j5_);
		
		slider_j6_ = new QwtSlider(this, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j6_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j6_->setValue( 0 );
		connect( slider_j6_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ6(double)) );
		
		table_widget_global_->setCellWidget(6, 3, slider_j6_);
		
		line_j0_ = new QLineEdit();
		line_j0_->setObjectName("line_j0_");
		line_j0_->setSizePolicy(fixed_policy);
		line_j0_->setText(QString::number(slider_j0_->value() ));
		connect( line_j0_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ0()) );
		
		table_widget_global_->setCellWidget(0, 1, line_j0_);
		
		line_j1_ = new QLineEdit();
		line_j1_->setObjectName("line_j1_");
		line_j1_->setSizePolicy(fixed_policy);
		line_j1_->setText(QString::number(slider_j1_->value() ));
		connect( line_j1_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ1()) );
		
		table_widget_global_->setCellWidget(1, 1, line_j1_);
		
		line_j2_ = new QLineEdit();
		line_j2_->setObjectName("line_j2_");
		line_j2_->setSizePolicy(fixed_policy);
		line_j2_->setText(QString::number(slider_j2_->value() ));
		connect( line_j2_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ2()) );
		
		table_widget_global_->setCellWidget(2, 1, line_j2_);
		
		line_j3_ = new QLineEdit();
		line_j3_->setObjectName("line_j3_");
		line_j3_->setSizePolicy(fixed_policy);
		line_j3_->setText(QString::number(slider_j3_->value() ));
		connect( line_j3_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ3()) );
		
		table_widget_global_->setCellWidget(3, 1, line_j3_);
		
		line_j4_ = new QLineEdit();
		line_j4_->setObjectName("line_j4_");
		line_j4_->setSizePolicy(fixed_policy);
		line_j4_->setText(QString::number(slider_j4_->value() ));
		connect( line_j4_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ4()) );
		
		table_widget_global_->setCellWidget(4, 1, line_j4_);
		
		line_j5_ = new QLineEdit();
		line_j5_->setObjectName("line_j5_");
		line_j5_->setSizePolicy(fixed_policy);
		line_j5_->setText(QString::number(slider_j5_->value() ));
		connect( line_j5_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ5()) );
		
		table_widget_global_->setCellWidget(5, 1, line_j5_);
		
		line_j6_ = new QLineEdit();
		line_j6_->setObjectName("line_j6_");
		line_j6_->setSizePolicy(fixed_policy);
		line_j6_->setText(QString::number(slider_j6_->value() ));
		connect( line_j6_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ6()) );
		
		table_widget_global_->setCellWidget(6, 1, line_j6_);
		
		vlayout_global_->addWidget(table_widget_global_);
		
		setLayout(vlayout_global_);
	}
	
	void QtPositionSliders::setValueLineJ0(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(0, 1)))->setText(QString::number( value ));
	}
	
	void QtPositionSliders::updateValueSliderJ0()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(0, 3)))->setValue(line_j0_->text().toDouble());
	}
	
	void QtPositionSliders::setValueLineJ1(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(1, 1)))->setText(QString::number( value ));
	}
	
	void QtPositionSliders::updateValueSliderJ1()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(1, 3)))->setValue(line_j1_->text().toDouble());
	}
	
	void QtPositionSliders::setValueLineJ2(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(2, 1)))->setText(QString::number( value ));
	}
	
	void QtPositionSliders::updateValueSliderJ2()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(2, 3)))->setValue(line_j2_->text().toDouble());
	}
	
	void QtPositionSliders::setValueLineJ3(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(3, 1)))->setText(QString::number( value ));
	}
	
	void QtPositionSliders::updateValueSliderJ3()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(3, 3)))->setValue(line_j3_->text().toDouble());
	}
	
	void QtPositionSliders::setValueLineJ4(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(4, 1)))->setText(QString::number( value ));
	}
	
	void QtPositionSliders::updateValueSliderJ4()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(4, 3)))->setValue(line_j4_->text().toDouble());
	}
	
	void QtPositionSliders::setValueLineJ5(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(5, 1)))->setText(QString::number( value ));
	}
	
	void QtPositionSliders::updateValueSliderJ5()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(5, 3)))->setValue(line_j5_->text().toDouble());
	}
	
	void QtPositionSliders::setValueLineJ6(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(6, 1)))->setText(QString::number( value ));
	}
	
	void QtPositionSliders::updateValueSliderJ6()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(6, 3)))->setValue(line_j6_->text().toDouble());
	}
	
	void QtPositionSliders::updateLabelJs(const QVector<double> & positions)
	{
		for (size_t i=0; i<positions.size(); i++)
		{
			table_widget_global_->item(i,2)->setText(QString::number(positions[i],'f',5));
		}
	}
	
	void QtPositionSliders::updateSliders(const QVector<double> & positions)
	{
		slider_j0_->setValue(positions[0]);
		slider_j1_->setValue(positions[1]);
		slider_j2_->setValue(positions[2]);
		slider_j3_->setValue(positions[3]);
		slider_j4_->setValue(positions[4]);
		slider_j5_->setValue(positions[5]);
		slider_j6_->setValue(positions[6]);
	}
}
