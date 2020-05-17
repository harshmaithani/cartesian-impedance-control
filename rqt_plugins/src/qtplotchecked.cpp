/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#include "rqt_plugins/qtplotchecked.h"
#include <algorithm>
	
namespace platform_sigma_plugins_ns {

	QtPlotChecked::QtPlotChecked( QWidget *parent, const QString& title_plot, const QString& axis_title_left, const QString& axis_title_bottom, const QPair<double, double>& axis_left_scales )
        : QWidget(parent), title_plot_(title_plot), axis_title_left_(axis_title_left), axis_title_bottom_(axis_title_bottom), axis_left_scales_(axis_left_scales), 
        plot_(0), plot_grid_(0), plot_legend_(0), hlayout_gobal_(0), vlayout_global_(0), vlayout_cb_curves_(0)
    {
		setVectCurveColor_();
			
		plot_ = new QwtPlot();
		plot_->setCanvasBackground(Qt::white);
		plot_->setTitle(title_plot_);
		plot_->setAxisTitle(QwtPlot::yLeft, axis_title_left_);
		plot_->setAxisTitle(QwtPlot::xBottom, axis_title_bottom_);
		plot_->setAxisScale(QwtPlot::yLeft, axis_left_scales_.first, axis_left_scales_.second);
		
		plot_grid_ = new QwtPlotGrid();
		plot_grid_->setPen(QPen(QColor(196,196,196)));
		plot_grid_->attach(plot_);
		
		plot_legend_ = new QwtLegend();
		plot_->insertLegend(plot_legend_, QwtPlot::BottomLegend);
		
		QString curve_name;
		
		curve_size_ = 4;
		
		vlayout_cb_curves_ = new QVBoxLayout();
		vect_cb_curves_.resize(7);
		
		for (size_t i=0; i<7; i++)
		{
			map_curve_[i] = new QwtPlotCurve(QString("Joint%1").arg(i));
			list_cb_curves_.append(QString("Joint%1").arg(i));
			
			vect_cb_curves_[i] = new QCheckBox(list_cb_curves_[i]);
			vect_cb_curves_[i]->setChecked(true);
			
			vlayout_cb_curves_->addWidget(vect_cb_curves_[i]);
			
			map_data_curve_[i].resize(50);
			vect_time_curve_.resize(50);
			
			map_curve_[i]->setPen(QPen(QColor(vect_curve_color_[i]), curve_size_, Qt::SolidLine));
			map_curve_[i]->setRawSamples(vect_time_curve_.data(), map_data_curve_[i].data(), 50);
			map_curve_[i]->attach(plot_);

		}
		
		connect( vect_cb_curves_[0], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ0_(int)) );
		connect( vect_cb_curves_[1], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ1_(int)) );
		connect( vect_cb_curves_[2], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ2_(int)) );
		connect( vect_cb_curves_[3], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ3_(int)) );
		connect( vect_cb_curves_[4], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ4_(int)) );
		connect( vect_cb_curves_[5], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ5_(int)) );
		connect( vect_cb_curves_[6], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ6_(int)) );
		
		plot_->show();
		plot_->replot();
		
		hlayout_gobal_ = new QHBoxLayout();
		hlayout_gobal_->addWidget(plot_);
		hlayout_gobal_->addLayout(vlayout_cb_curves_);
		
		vlayout_global_ = new QVBoxLayout();
		vlayout_global_->addLayout(hlayout_gobal_);
		
		setLayout(vlayout_global_);
    }
    
    QtPlotChecked::~QtPlotChecked()
    {
		disconnect( vect_cb_curves_[0], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ0_(int)) );
		disconnect( vect_cb_curves_[1], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ1_(int)) );
		disconnect( vect_cb_curves_[2], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ2_(int)) );
		disconnect( vect_cb_curves_[3], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ3_(int)) );
		disconnect( vect_cb_curves_[4], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ4_(int)) );
		disconnect( vect_cb_curves_[5], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ5_(int)) );
		disconnect( vect_cb_curves_[6], SIGNAL(stateChanged(int)), this, SLOT(checkedSlotJ6_(int)) );
		
		for (size_t i=0; i<7; i++)
		{
			vlayout_cb_curves_->removeWidget(vect_cb_curves_[i]);
			delete vect_cb_curves_[i];
			
			map_curve_[i]->detach();
			delete map_curve_[i];
		}
		
		delete vlayout_cb_curves_;
		hlayout_gobal_->removeWidget(plot_);
		
		delete hlayout_gobal_;

		plot_grid_->detach();
		
		delete plot_grid_;
		delete plot_legend_;
		delete plot_;
		
		delete vlayout_global_;
	}
    
    void QtPlotChecked::checkedSlotJ0_(int value)
    {
		if (value == Qt::Checked)
		{
			map_curve_[0]->show();
		}
		else
		{
			map_curve_[0]->hide();
		}
	}
	
	void QtPlotChecked::checkedSlotJ1_(int value)
    {
		if (value == Qt::Checked)
		{
			map_curve_[1]->show();
		}
		else
		{
			map_curve_[1]->hide();
		}
	}
	
	void QtPlotChecked::checkedSlotJ2_(int value)
    {
		if (value == Qt::Checked)
		{
			map_curve_[2]->show();
		}
		else
		{
			map_curve_[2]->hide();
		}
	}
	
	void QtPlotChecked::checkedSlotJ3_(int value)
    {
		if (value == Qt::Checked)
		{
			map_curve_[3]->show();
		}
		else
		{
			map_curve_[3]->hide();
		}
	}
	
	void QtPlotChecked::checkedSlotJ4_(int value)
    {
		if (value == Qt::Checked)
		{
			map_curve_[4]->show();
		}
		else
		{
			map_curve_[4]->hide();
		}
	}
	
	void QtPlotChecked::checkedSlotJ5_(int value)
    {
		if (value == Qt::Checked)
		{
			map_curve_[5]->show();
		}
		else
		{
			map_curve_[5]->hide();
		}
	}
	
	void QtPlotChecked::checkedSlotJ6_(int value)
    {
		if (value == Qt::Checked)
		{
			map_curve_[6]->show();
		}
		else
		{
			map_curve_[6]->hide();
		}
	}
    
    void QtPlotChecked::setVectCurveColor_()
    {
		vect_curve_color_.clear();
		vect_curve_color_.append(Qt::black);
		vect_curve_color_.append(Qt::green);
		vect_curve_color_.append(Qt::red);
		vect_curve_color_.append(Qt::yellow);
		vect_curve_color_.append(Qt::blue);
		vect_curve_color_.append(Qt::cyan);
		vect_curve_color_.append(Qt::gray);
	}

	void QtPlotChecked::updateDataCurves(QVector<double> values, double timeDuration)
	{
		if (vect_time_curve_.size() > 49)
		{
			vect_time_curve_.remove(0);
			
			for (size_t i=0; i<7; i++)
			{
				map_data_curve_[i].remove(0);
			}
		}
		
		vect_time_curve_.append(timeDuration);
		
		for (size_t i=0; i<7; i++)
		{				
			map_data_curve_[i].append(values[i]);	
		}
	}
	
	void QtPlotChecked::updateAxisScale()
	{
		if (plot_)
		{
			plot_->setAxisScale(QwtPlot::xBottom, vect_time_curve_[0], vect_time_curve_[vect_time_curve_.size()-1]);
		
			plot_->replot();
		}
	}
    
} // End of namespace
