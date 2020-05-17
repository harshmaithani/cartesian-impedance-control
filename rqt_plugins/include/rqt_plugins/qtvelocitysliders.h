/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#ifndef Qt_Velocity_Sliders_Widget_H
#define Qt_Velocity_Sliders_Widget_H

// Qt graphics
#include <QWidget>
#include <QLineEdit>
#include <QTableWidget>
#include <QVBoxLayout>

// Qwt graphics
#include <qwt_slider.h>


namespace platform_sigma_plugins_ns {
	
	class QtVelocitySliders : public QWidget
	{
		Q_OBJECT
		
		public:
			QtVelocitySliders( QWidget *parent = 0 );
			~QtVelocitySliders();
			
			void updateSliders(const QVector<double> & velocities);
			
			QwtSlider* slider_j0_, *slider_j1_, *slider_j2_, *slider_j3_, *slider_j4_, *slider_j5_, *slider_j6_;
			QLineEdit* line_j0_, *line_j1_, *line_j2_, *line_j3_, *line_j4_, *line_j5_, *line_j6_;
			
		public slots:
			void setValueLineJ0(double value);
			void setValueLineJ1(double value);
			void setValueLineJ2(double value);
			void setValueLineJ3(double value);
			void setValueLineJ4(double value);
			void setValueLineJ5(double value);
			void setValueLineJ6(double value);
			
			void updateValueSliderJ0();
			void updateValueSliderJ1();
			void updateValueSliderJ2();
			void updateValueSliderJ3();
			void updateValueSliderJ4();
			void updateValueSliderJ5();
			void updateValueSliderJ6();
			
		private:
			QTableWidget* table_widget_global_;
			QVBoxLayout* vlayout_global_;
			
			void createWidget_();
		
	}; // End of class
	
} // End of namespace

#endif
