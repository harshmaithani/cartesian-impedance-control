/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#ifndef Qt_Plot_Checked_Widget_H
#define Qt_Plot_Checked_Widget_H

// Qt graphics
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCheckBox>

// Qt core
#include <QPair>
#include <QMap>
#include <QVector>

// Qwt graphics
#include <qwt_plot_grid.h>
#include <qwt_legend.h>
#include <qwt_symbol.h>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>

// ros
#include "ros/ros.h"

namespace platform_sigma_plugins_ns {
	
	class QtPlotChecked : public QWidget
	{
		Q_OBJECT
		
		public:
			QtPlotChecked( QWidget *parent = 0, const QString& title_plot =  QString(), const QString& axis_title_left =  QString(), const QString& axis_title_bottom =  QString(), const QPair<double, double>& axis_left_scales = QPair<double,double>(0.0,0.0) );
			~QtPlotChecked();
			
			void updateDataCurves(QVector<double> values, double timeDuration);
			void updateAxisScale(); 
			
		private:
			QPair<double,double> axis_left_scales_;
			QString title_plot_;
			QString axis_title_left_, axis_title_bottom_;
			
			QMap<int,QVector<double> > map_data_curve_;
			QMap<int,QVector<double> > map_time_curve_;
			QVector<double> vect_time_curve_;
			
			QMap<int,QwtPlotCurve *> map_curve_;
			QStringList  			list_cb_curves_;
			QVector<QCheckBox *> 		vect_cb_curves_;
			//QMap<QString,QwtSymbol*> map_curve_symbol_;
			
			QwtPlot      	*plot_;
			QwtPlotGrid 	*plot_grid_;
			QwtLegend		*plot_legend_;
			
			QVBoxLayout		*vlayout_global_, *vlayout_cb_curves_;
			QHBoxLayout		*hlayout_gobal_;
			
			QVector<QColor> vect_curve_color_;
			int curve_size_;
			
			void setVectCurveColor_();	
		
		private slots:	
			void checkedSlotJ0_(int value);
			void checkedSlotJ1_(int value);
			void checkedSlotJ2_(int value);
			void checkedSlotJ3_(int value);
			void checkedSlotJ4_(int value);
			void checkedSlotJ5_(int value);
			void checkedSlotJ6_(int value);
			
		
	}; // End of class
	
} // End of namespace
    
#endif 
